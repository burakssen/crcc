use crate::collision_checker::engine::parry::simple::ParrySimpleCollisionObject;
use crate::collision_object::CollisionObject;
use glamx::{DPose2, DVec2};
use parry2d_f64::query::{
    NonlinearRigidMotion, Unsupported, cast_shapes_nonlinear, distance, intersection_test,
};
use parry2d_f64::shape::{Compound, TriMesh, TriMeshBuilderError};
use std::f64::consts::{PI, TAU};

// Most collision objects will be non-trivial, so boxing them would be counter-productive.
// See https://rust-lang.github.io/rust-clippy/rust-1.92.0/index.html#large_enum_variant
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug)]
pub enum ParryCollisionObjectInner {
    Empty,
    FullSpace,
    NonTrivial(NonTrivial),
}

#[derive(Clone, Debug)]
pub struct NonTrivial {
    tri_mesh_compound: Option<Compound>,
    generic_compound: Option<Compound>,
}

impl ParryCollisionObjectInner {
    pub fn collides(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<bool, Unsupported> {
        match (self, other) {
            (ParryCollisionObjectInner::Empty, _) | (_, ParryCollisionObjectInner::Empty) => {
                Ok(false)
            }
            (ParryCollisionObjectInner::FullSpace, _)
            | (_, ParryCollisionObjectInner::FullSpace) => Ok(true),
            (
                ParryCollisionObjectInner::NonTrivial(slf),
                ParryCollisionObjectInner::NonTrivial(other),
            ) => slf.collides(pos_self, other, pos_other),
        }
    }

    pub fn collides_continuous(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> Result<bool, Unsupported> {
        match (self, other) {
            (ParryCollisionObjectInner::Empty, _) | (_, ParryCollisionObjectInner::Empty) => {
                Ok(false)
            }
            (ParryCollisionObjectInner::FullSpace, _)
            | (_, ParryCollisionObjectInner::FullSpace) => Ok(true),
            (
                ParryCollisionObjectInner::NonTrivial(slf),
                ParryCollisionObjectInner::NonTrivial(other),
            ) => slf.collides_continuous(
                start_pos_self,
                end_pos_self,
                other,
                start_pos_other,
                end_pos_other,
            ),
        }
    }

    pub fn distance(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<f64, Unsupported> {
        match (self, other) {
            (ParryCollisionObjectInner::Empty, _) | (_, ParryCollisionObjectInner::Empty) => {
                Err(Unsupported)
            }
            (ParryCollisionObjectInner::FullSpace, _)
            | (_, ParryCollisionObjectInner::FullSpace) => Ok(0.0),
            (
                ParryCollisionObjectInner::NonTrivial(slf),
                ParryCollisionObjectInner::NonTrivial(other),
            ) => slf.distance(pos_self, other, pos_other),
        }
    }
}

impl NonTrivial {
    pub fn collides(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<bool, Unsupported> {
        for comp_self in [&self.generic_compound, &self.tri_mesh_compound]
            .into_iter()
            .flatten()
        {
            for comp_other in [&other.generic_compound, &other.tri_mesh_compound]
                .into_iter()
                .flatten()
            {
                if intersection_test(&pos_self, comp_self, &pos_other, comp_other)? {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    pub fn collides_continuous(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> Result<bool, Unsupported> {
        if self.collides(start_pos_self, other, start_pos_other)?
            || self.collides(end_pos_self, other, end_pos_other)?
        {
            return Ok(true);
        }

        let motion_self = motion_from_start_end(start_pos_self, end_pos_self);
        let motion_other = motion_from_start_end(start_pos_other, end_pos_other);
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

    pub fn distance(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<f64, Unsupported> {
        let mut min_distance = f64::INFINITY;
        for comp_self in [&self.generic_compound, &self.tri_mesh_compound]
            .into_iter()
            .flatten()
        {
            for comp_other in [&other.generic_compound, &other.tri_mesh_compound]
                .into_iter()
                .flatten()
            {
                min_distance =
                    min_distance.min(distance(&pos_self, comp_self, &pos_other, comp_other)?);
                if min_distance == 0.0 {
                    return Ok(0.0);
                }
            }
        }
        if min_distance.is_finite() {
            Ok(min_distance)
        } else {
            Err(Unsupported)
        }
    }
}

fn motion_from_start_end(start: DPose2, end: DPose2) -> NonlinearRigidMotion {
    let velocity = end.translation - start.translation;
    let angular_velocity = shortest_angle_delta(start.rotation.angle(), end.rotation.angle());
    NonlinearRigidMotion::new(start, DVec2::ZERO, velocity, angular_velocity)
}

fn shortest_angle_delta(start: f64, end: f64) -> f64 {
    (end - start + PI).rem_euclid(TAU) - PI
}

impl From<CollisionObject> for ParryCollisionObjectInner {
    fn from(value: CollisionObject) -> Self {
        let mut tri_meshes = Vec::new();
        let mut generic_objects = Vec::new();
        for simple in value {
            let converted = ParrySimpleCollisionObject::from(simple);
            match converted {
                ParrySimpleCollisionObject::Empty => { /* intentionally left blank */ }
                ParrySimpleCollisionObject::FullSpace => {
                    // immediately return FullSpace, as it dominates all other objects
                    return Self::FullSpace;
                }
                ParrySimpleCollisionObject::TriMesh(mesh) => {
                    tri_meshes.push(*mesh);
                }
                ParrySimpleCollisionObject::Shape { .. } => {
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

        match (&tri_mesh_compound, &generic_compound) {
            (None, None) => Self::Empty,
            _ => Self::NonTrivial(NonTrivial {
                tri_mesh_compound,
                generic_compound,
            }),
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
