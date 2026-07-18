use crate::collision_checker::engine::parry::simple::ParrySimpleCollisionObject;
use crate::collision_object::CollisionObject;
use glamx::{DPose2, DVec2};
use parry2d_f64::bounding_volume::{Aabb, BoundingVolume};
use parry2d_f64::query::{
    NonlinearRigidMotion, Unsupported, cast_shapes_nonlinear, distance, intersection_test,
};
use parry2d_f64::shape::{Compound, Shape, TriMesh, TriMeshBuilderError};
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
                let aabb_self = Self::swept_aabb(comp_self, start_pos_self, end_pos_self);
                let aabb_other = Self::swept_aabb(comp_other, start_pos_other, end_pos_other);
                if !aabb_self.intersects(&aabb_other) {
                    continue;
                }

                let shapes_self = comp_self.shapes();
                let shapes_other = comp_other.shapes();
                if shapes_self.len() == 1 && shapes_other.len() == 1 {
                    let (local_pose_self, shape_self) = &shapes_self[0];
                    let (local_pose_other, shape_other) = &shapes_other[0];

                    if let (
                        parry2d_f64::shape::TypedShape::Ball(ball_self),
                        parry2d_f64::shape::TypedShape::Ball(ball_other),
                    ) = (shape_self.as_typed_shape(), shape_other.as_typed_shape())
                        && ((start_pos_self.rotation.angle() - end_pos_self.rotation.angle()).abs()
                            <= 1e-12
                            || local_pose_self.translation.length_squared() <= 1e-24)
                        && ((start_pos_other.rotation.angle() - end_pos_other.rotation.angle())
                            .abs()
                            <= 1e-12
                            || local_pose_other.translation.length_squared() <= 1e-24)
                    {
                        let s1 = start_pos_self * local_pose_self.translation;
                        let e1 = end_pos_self * local_pose_self.translation;
                        let s2 = start_pos_other * local_pose_other.translation;
                        let e2 = end_pos_other * local_pose_other.translation;

                        let d_s = s1 - s2;
                        let r = ball_self.radius + ball_other.radius;
                        let r_sq = r * r;

                        if d_s.length_squared() <= r_sq {
                            return Ok(true);
                        }

                        let v_self = e1 - s1;
                        let v_other = e2 - s2;
                        let v_d = v_self - v_other;

                        let a = v_d.length_squared();
                        if a > 1e-12 {
                            let b = 2.0 * d_s.dot(v_d);
                            let c = d_s.length_squared() - r_sq;
                            let discriminant = b * b - 4.0 * a * c;
                            if discriminant >= 0.0 {
                                let t = (-b - discriminant.sqrt()) / (2.0 * a);
                                if (0.0..=1.0).contains(&t) {
                                    return Ok(true);
                                }
                            }
                        }
                        continue;
                    }
                }

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

    fn swept_aabb(comp: &Compound, start: DPose2, end: DPose2) -> Aabb {
        let start_angle = start.rotation.angle();
        let end_angle = end.rotation.angle();
        if (start_angle - end_angle).abs() <= 1e-12 {
            let aabb_start = comp.compute_aabb(&start);
            let aabb_end = comp.compute_aabb(&end);
            return aabb_start.merged(&aabb_end);
        }

        let local_aabb = comp.compute_aabb(&DPose2::IDENTITY);
        let r = local_aabb.mins.length().max(local_aabb.maxs.length());

        let min_x = start.translation.x.min(end.translation.x) - r;
        let min_y = start.translation.y.min(end.translation.y) - r;
        let max_x = start.translation.x.max(end.translation.x) + r;
        let max_y = start.translation.y.max(end.translation.y) + r;

        Aabb::new(DVec2::new(min_x, min_y), DVec2::new(max_x, max_y))
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
