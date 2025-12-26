use crate::collision_checker::engine::parry::collision_object::simple::ParrySimpleCollisionObject;
use crate::collision_object::StaticCollisionObject;
use nalgebra::Isometry2;
use parry2d_f64::query::{Unsupported, intersection_test};
use parry2d_f64::shape::{Compound, TriMesh, TriMeshBuilderError};

mod simple;

pub struct ParryCollisionObject(pub(super) ParryCollisionObjectInner);

impl From<StaticCollisionObject> for ParryCollisionObject {
    fn from(value: StaticCollisionObject) -> Self {
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
        self_pos: &Isometry2<f64>,
        other: &Self,
        other_pos: &Isometry2<f64>,
    ) -> Result<bool, Unsupported> {
        let mut unsupported = false;
        for self_comp in [&self.generic_compound, &self.tri_mesh_compound]
            .into_iter()
            .flatten()
        {
            for other_comp in [&other.generic_compound, &other.tri_mesh_compound]
                .into_iter()
                .flatten()
            {
                let collides = intersection_test(self_pos, self_comp, other_pos, other_comp);
                unsupported |= matches!(collides, Err(Unsupported));
                if let Ok(true) = collides {
                    return Ok(true);
                }
            }
        }
        if unsupported {
            Err(Unsupported)
        } else {
            Ok(false)
        }
    }
}

impl From<StaticCollisionObject> for ParryCollisionObjectInner {
    fn from(value: StaticCollisionObject) -> Self {
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
