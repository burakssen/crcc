use crate::collision_checker::engine::parry::collision_object::simple::ParrySimpleCollisionObject;
use crate::collision_object::CollisionObject;
use nalgebra::{Isometry2, Point2};
use parry2d_f64::query::{
    NonlinearRigidMotion, Unsupported, cast_shapes_nonlinear, intersection_test,
};
use parry2d_f64::shape::{Compound, TriMesh, TriMeshBuilderError};

mod simple;

pub struct ParryCollisionObject(pub(super) ParryCollisionObjectInner);

impl From<CollisionObject> for ParryCollisionObject {
    fn from(value: CollisionObject) -> Self {
        Self(value.into())
    }
}

pub struct ParryCollisionObjectInner {
    tri_mesh_compound: Option<Compound>,
    generic_compound: Option<Compound>,
}

impl ParryCollisionObjectInner {
    pub fn collides(
        &self,
        pos_self: &Isometry2<f64>,
        other: &Self,
        pos_other: &Isometry2<f64>,
    ) -> Result<bool, Unsupported> {
        for comp_self in [&self.generic_compound, &self.tri_mesh_compound]
            .into_iter()
            .flatten()
        {
            for comp_other in [&other.generic_compound, &other.tri_mesh_compound]
                .into_iter()
                .flatten()
            {
                if intersection_test(pos_self, comp_self, pos_other, comp_other)? {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    pub fn collides_continuous(
        &self,
        start_pos_self: &Isometry2<f64>,
        end_pos_self: &Isometry2<f64>,
        other: &Self,
        start_pos_other: &Isometry2<f64>,
        end_pos_other: &Isometry2<f64>,
    ) -> Result<bool, Unsupported> {
        let motion_self = motion_from_start_end(*start_pos_self, *end_pos_self);
        let motion_other = motion_from_start_end(*start_pos_other, *end_pos_other);
        for comp_self in [&self.generic_compound, &self.tri_mesh_compound]
            .into_iter()
            .flatten()
        {
            for comp_other in [&other.generic_compound, &other.tri_mesh_compound]
                .into_iter()
                .flatten()
            {
                if cast_shapes_nonlinear(
                    &motion_self,
                    comp_self,
                    &motion_other,
                    comp_other,
                    0.0,
                    1.0,
                    true,
                )?
                .is_some()
                {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }
}

fn motion_from_start_end(start: Isometry2<f64>, end: Isometry2<f64>) -> NonlinearRigidMotion {
    let velocity = end.translation.vector - start.translation.vector;
    let angular_velocity = end.rotation.angle() - start.rotation.angle();
    NonlinearRigidMotion::new(start, Point2::origin(), velocity, angular_velocity)
}

impl From<CollisionObject> for ParryCollisionObjectInner {
    fn from(value: CollisionObject) -> Self {
        let mut tri_meshes = Vec::new();
        let mut generic_objects = Vec::new();
        for simple in value {
            let converted = ParrySimpleCollisionObject::from(simple);
            match converted {
                ParrySimpleCollisionObject::Empty => { /* intentionally left blank */ }
                ParrySimpleCollisionObject::TriMesh(mesh) => {
                    tri_meshes.push(*mesh);
                }
                ParrySimpleCollisionObject::Generic { .. } => {
                    generic_objects.push(
                        converted
                            .into_shared_shape()
                            .expect("Should not be an empty shape."),
                    );
                }
            }
        }

        let tri_mesh_compound = if tri_meshes.is_empty() {
            None
        } else {
            let merged_trimesh =
                merge_trimeshes(tri_meshes).expect("Merging trimeshes should succeed");
            Some(
                Compound::decompose_trimesh(&merged_trimesh)
                    .expect("Decomposing trimesh should succeed"),
            )
        };
        let generic_compound = if generic_objects.is_empty() {
            None
        } else {
            Some(Compound::new(generic_objects))
        };

        Self {
            tri_mesh_compound,
            generic_compound,
        }
    }
}

fn merge_trimeshes(
    meshes: impl IntoIterator<Item = TriMesh>,
) -> Result<TriMesh, TriMeshBuilderError> {
    // parry can only merge two meshes at a time
    // calling this function repeatedly to merge all meshes can be inefficient
    // so we provide a custom implementation here
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut offset = 0;
    for mesh in meshes {
        let mesh_vertices = mesh.vertices();
        let mesh_indices = mesh.indices();
        vertices.extend_from_slice(mesh_vertices);
        indices.extend(
            mesh_indices
                .iter()
                .map(|[i0, i1, i2]| [i0 + offset, i1 + offset, i2 + offset]),
        );
        offset += mesh_vertices.len() as u32;
    }
    TriMesh::new(vertices, indices)
}
