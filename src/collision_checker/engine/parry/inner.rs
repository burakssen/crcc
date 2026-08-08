use crate::collision_checker::engine::parry::simple::ParrySimpleCollisionObject;
use crate::collision_object::CollisionObject;
use glamx::{DPose2, DVec2};
use parry2d_f64::bounding_volume::{Aabb, BoundingVolume};
use parry2d_f64::query::{
    NonlinearRigidMotion, Unsupported, cast_shapes_nonlinear, distance, intersection_test,
};
use parry2d_f64::shape::{Compound, Shape, SharedShape, TriMesh};
use std::f64::consts::{PI, TAU};
use std::ops::{Add, Div, Mul, Neg, Sub};

const ROTATION_EPSILON: f64 = 1e-12;
const LOCAL_OFFSET_EPSILON_SQUARED: f64 = 1e-24;
const QUADRATIC_EPSILON: f64 = 1e-12;

// Most collision objects will be non-trivial, so boxing them would be
// counterproductive.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug)]
pub enum ParryCollisionObjectInner {
    Empty,
    FullSpace,
    // ponytail: Keep conversion infallible while surfacing invalid prepared geometry on use.
    Invalid,
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
            (Self::Invalid, _) | (_, Self::Invalid) => Err(Unsupported),
            (Self::Empty, _) | (_, Self::Empty) => Ok(false),
            (Self::FullSpace, _) | (_, Self::FullSpace) => Ok(true),
            (Self::NonTrivial(left), Self::NonTrivial(right)) => {
                left.collides(pos_self, right, pos_other)
            }
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
            (Self::Invalid, _) | (_, Self::Invalid) => Err(Unsupported),
            (Self::Empty, _) | (_, Self::Empty) => Ok(false),
            (Self::FullSpace, _) | (_, Self::FullSpace) => Ok(true),
            (Self::NonTrivial(left), Self::NonTrivial(right)) => left.collides_continuous(
                start_pos_self,
                end_pos_self,
                right,
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
            (Self::Invalid | Self::Empty, _) | (_, Self::Invalid | Self::Empty) => Err(Unsupported),
            (Self::FullSpace, _) | (_, Self::FullSpace) => Ok(0.0),
            (Self::NonTrivial(left), Self::NonTrivial(right)) => {
                left.distance(pos_self, right, pos_other)
            }
        }
    }
}

impl NonTrivial {
    fn compounds(&self) -> impl Iterator<Item = &Compound> {
        [
            self.generic_compound.as_ref(),
            self.tri_mesh_compound.as_ref(),
        ]
        .into_iter()
        .flatten()
    }

    pub fn collides(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<bool, Unsupported> {
        for compound_self in self.compounds() {
            for compound_other in other.compounds() {
                if intersection_test(&pos_self, compound_self, &pos_other, compound_other)? {
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

        for compound_self in self.compounds() {
            for compound_other in other.compounds() {
                let aabb_self = Self::swept_aabb(compound_self, start_pos_self, end_pos_self);
                let aabb_other = Self::swept_aabb(compound_other, start_pos_other, end_pos_other);

                if !aabb_self.intersects(&aabb_other) {
                    continue;
                }

                if let Some(collides) = moving_ball_pair_collides(
                    compound_self,
                    start_pos_self,
                    end_pos_self,
                    compound_other,
                    start_pos_other,
                    end_pos_other,
                ) {
                    if collides {
                        return Ok(true);
                    }

                    continue;
                }

                if cast_shapes_nonlinear(
                    &motion_self,
                    compound_self,
                    &motion_other,
                    compound_other,
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

    fn swept_aabb(compound: &Compound, start: DPose2, end: DPose2) -> Aabb {
        let start_angle = start.rotation.angle();
        let end_angle = end.rotation.angle();

        if start_angle.sub(end_angle).abs() <= ROTATION_EPSILON {
            let start_aabb = compound.compute_aabb(&start);
            let end_aabb = compound.compute_aabb(&end);

            return start_aabb.merged(&end_aabb);
        }

        let local_aabb = compound.compute_aabb(&DPose2::IDENTITY);
        let radius = local_aabb.mins.length().max(local_aabb.maxs.length());

        let minimum = DVec2::new(
            start.translation.x.min(end.translation.x).sub(radius),
            start.translation.y.min(end.translation.y).sub(radius),
        );

        let maximum = DVec2::new(
            start.translation.x.max(end.translation.x).add(radius),
            start.translation.y.max(end.translation.y).add(radius),
        );

        Aabb::new(minimum, maximum)
    }

    pub fn distance(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<f64, Unsupported> {
        let mut minimum_distance = f64::INFINITY;

        for compound_self in self.compounds() {
            for compound_other in other.compounds() {
                minimum_distance = minimum_distance.min(distance(
                    &pos_self,
                    compound_self,
                    &pos_other,
                    compound_other,
                )?);

                if minimum_distance <= 0.0 {
                    return Ok(0.0);
                }
            }
        }

        minimum_distance
            .is_finite()
            .then_some(minimum_distance)
            .ok_or(Unsupported)
    }
}

fn moving_ball_pair_collides(
    compound_self: &Compound,
    start_pos_self: DPose2,
    end_pos_self: DPose2,
    compound_other: &Compound,
    start_pos_other: DPose2,
    end_pos_other: DPose2,
) -> Option<bool> {
    let [self_entry] = compound_self.shapes() else {
        return None;
    };

    let [other_entry] = compound_other.shapes() else {
        return None;
    };

    let (local_pose_self, shape_self) = self_entry;
    let (local_pose_other, shape_other) = other_entry;

    let (
        parry2d_f64::shape::TypedShape::Ball(ball_self),
        parry2d_f64::shape::TypedShape::Ball(ball_other),
    ) = (shape_self.as_typed_shape(), shape_other.as_typed_shape())
    else {
        return None;
    };

    let self_rotation_supported = start_pos_self
        .rotation
        .angle()
        .sub(end_pos_self.rotation.angle())
        .abs()
        <= ROTATION_EPSILON
        || local_pose_self.translation.length_squared() <= LOCAL_OFFSET_EPSILON_SQUARED;

    let other_rotation_supported = start_pos_other
        .rotation
        .angle()
        .sub(end_pos_other.rotation.angle())
        .abs()
        <= ROTATION_EPSILON
        || local_pose_other.translation.length_squared() <= LOCAL_OFFSET_EPSILON_SQUARED;

    if !self_rotation_supported || !other_rotation_supported {
        return None;
    }

    let start_self = start_pos_self.mul(local_pose_self.translation);
    let end_self = end_pos_self.mul(local_pose_self.translation);
    let start_other = start_pos_other.mul(local_pose_other.translation);
    let end_other = end_pos_other.mul(local_pose_other.translation);

    let relative_start = start_self.sub(start_other);
    let radius = ball_self.radius.add(ball_other.radius);
    let radius_squared = radius.mul(radius);

    if relative_start.length_squared() <= radius_squared {
        return Some(true);
    }

    let velocity_self = end_self.sub(start_self);
    let velocity_other = end_other.sub(start_other);
    let relative_velocity = velocity_self.sub(velocity_other);

    let quadratic_a = relative_velocity.length_squared();

    if quadratic_a <= QUADRATIC_EPSILON {
        return Some(false);
    }

    let quadratic_b = 2.0_f64.mul(relative_start.dot(relative_velocity));
    let quadratic_c = relative_start.length_squared().sub(radius_squared);

    let discriminant = quadratic_b
        .mul(quadratic_b)
        .sub(4.0_f64.mul(quadratic_a).mul(quadratic_c));

    if discriminant < 0.0 {
        return Some(false);
    }

    let denominator = 2.0_f64.mul(quadratic_a);
    let collision_time = quadratic_b.neg().sub(discriminant.sqrt()).div(denominator);

    Some((0.0..=1.0).contains(&collision_time))
}

fn motion_from_start_end(start: DPose2, end: DPose2) -> NonlinearRigidMotion {
    let velocity = end.translation.sub(start.translation);

    let angular_velocity = shortest_angle_delta(start.rotation.angle(), end.rotation.angle());

    NonlinearRigidMotion::new(start, DVec2::ZERO, velocity, angular_velocity)
}

fn shortest_angle_delta(start: f64, end: f64) -> f64 {
    end.sub(start).add(PI).rem_euclid(TAU).sub(PI)
}

impl From<CollisionObject> for ParryCollisionObjectInner {
    fn from(value: CollisionObject) -> Self {
        let mut tri_meshes = Vec::new();
        let mut generic_objects = Vec::new();

        for simple in value {
            match ParrySimpleCollisionObject::from(simple) {
                ParrySimpleCollisionObject::Empty => {}

                ParrySimpleCollisionObject::FullSpace => {
                    return Self::FullSpace;
                }

                ParrySimpleCollisionObject::Invalid => {
                    return Self::Invalid;
                }

                ParrySimpleCollisionObject::TriMesh(mesh) => {
                    tri_meshes.push(*mesh);
                }

                ParrySimpleCollisionObject::Shape { shape, position } => {
                    generic_objects.push((position, SharedShape(shape.into())));
                }
            }
        }

        let tri_mesh_compound = if tri_meshes.is_empty() {
            None
        } else {
            let Some(merged_tri_mesh) = merge_trimeshes(tri_meshes) else {
                return Self::Invalid;
            };

            let Some(compound) = Compound::decompose_trimesh(&merged_tri_mesh) else {
                return Self::Invalid;
            };

            Some(compound)
        };

        let generic_compound =
            (!generic_objects.is_empty()).then(|| Compound::new(generic_objects));

        if tri_mesh_compound.is_none() && generic_compound.is_none() {
            Self::Empty
        } else {
            Self::NonTrivial(NonTrivial {
                tri_mesh_compound,
                generic_compound,
            })
        }
    }
}

fn merge_trimeshes(meshes: impl IntoIterator<Item = TriMesh>) -> Option<TriMesh> {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut offset = 0_u32;

    for mesh in meshes {
        let mesh_vertices = mesh.vertices();
        let vertex_count = u32::try_from(mesh_vertices.len()).ok()?;

        for &[index_0, index_1, index_2] in mesh.indices() {
            indices.push([
                index_0.checked_add(offset)?,
                index_1.checked_add(offset)?,
                index_2.checked_add(offset)?,
            ]);
        }

        vertices.extend_from_slice(mesh_vertices);
        offset = offset.checked_add(vertex_count)?;
    }

    TriMesh::new(vertices, indices).ok()
}
