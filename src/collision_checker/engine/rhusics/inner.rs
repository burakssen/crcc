use crate::collision_checker::engine::rhusics::simple::{
    HalfSpaceComponent, RhusicsCoreCollisionComponent, RhusicsCoreSimpleCollisionObject,
};
use crate::collision_object::CollisionObject;
use crate::collision_object::simple::rotation_changed;
use cgmath::{Basis2, Point2, Rad, Rotation2, Transform, Vector2};
use collision::algorithm::minkowski::GJK2;
use collision::{CollisionStrategy, Primitive};
use glamx::{DPose2, DVec2};
use rhusics_core::Pose;
use rhusics_core::collide2d::BodyPose2;
use std::fmt;
use std::ops::{Add, Mul, Neg, Sub};

const HALF_SPACE_EPSILON: f64 = 1e-9;

#[allow(clippy::large_enum_variant)]
#[derive(Clone)]
pub enum RhusicsCoreCollisionObjectInner {
    Empty,
    FullSpace,
    NonTrivial(NonTrivial),
}

impl fmt::Debug for RhusicsCoreCollisionObjectInner {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(formatter, "Empty"),
            Self::FullSpace => write!(formatter, "FullSpace"),
            Self::NonTrivial(_) => {
                write!(formatter, "NonTrivial([Components])")
            }
        }
    }
}

#[derive(Clone)]
pub struct NonTrivial {
    finite: Vec<(collision::primitive::Primitive2<f64>, BodyPose2<f64>)>,
    finite_motion_radii: Vec<f64>,
    half_spaces: Vec<HalfSpaceComponent>,
}

fn glam_to_cgmath_pose(pose: &DPose2) -> BodyPose2<f64> {
    BodyPose2::new(
        Point2::new(pose.translation.x, pose.translation.y),
        Basis2::from_angle(Rad(pose.rotation.angle())),
    )
}

impl RhusicsCoreCollisionObjectInner {
    #[must_use]
    pub fn collides(&self, pos_self: DPose2, other: &Self, pos_other: DPose2) -> bool {
        match (self, other) {
            (Self::Empty, _) | (_, Self::Empty) => false,

            (Self::FullSpace, _) | (_, Self::FullSpace) => true,

            (Self::NonTrivial(left), Self::NonTrivial(right)) => {
                left.collides(pos_self, right, pos_other)
            }
        }
    }

    #[must_use]
    pub fn collides_continuous(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> bool {
        match (self, other) {
            (Self::Empty, _) | (_, Self::Empty) => false,

            (Self::FullSpace, _) | (_, Self::FullSpace) => true,

            (Self::NonTrivial(left), Self::NonTrivial(right)) => left.collides_continuous(
                start_pos_self,
                end_pos_self,
                right,
                start_pos_other,
                end_pos_other,
            ),
        }
    }
}

impl NonTrivial {
    #[must_use]
    pub fn collides(&self, pos_self: DPose2, other: &Self, pos_other: DPose2) -> bool {
        let gjk = GJK2::new();
        let self_pose = glam_to_cgmath_pose(&pos_self);
        let other_pose = glam_to_cgmath_pose(&pos_other);

        if !self.finite.is_empty()
            && !other.finite.is_empty()
            && gjk
                .intersection_complex(
                    &CollisionStrategy::CollisionOnly,
                    &self.finite,
                    &self_pose,
                    &other.finite,
                    &other_pose,
                )
                .is_some()
        {
            return true;
        }

        self.half_spaces_collide(pos_self, other, pos_other)
    }

    #[must_use]
    pub fn collides_continuous(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> bool {
        if self.collides(start_pos_self, other, start_pos_other)
            || self.collides(end_pos_self, other, end_pos_other)
        {
            return true;
        }

        let moves = start_pos_self != end_pos_self || start_pos_other != end_pos_other;

        if !moves {
            return false;
        }

        /*
         * Continuous collision handling for analytic half-spaces
         * is conservative. If either object contains one and no
         * endpoint collision was found, report a possible collision.
         */
        if !self.half_spaces.is_empty() || !other.half_spaces.is_empty() {
            return true;
        }

        if rotation_changed(start_pos_self, end_pos_self)
            || rotation_changed(start_pos_other, end_pos_other)
        {
            return self.finite_motion_bounds_overlap(
                start_pos_self,
                end_pos_self,
                other,
                start_pos_other,
                end_pos_other,
            );
        }

        let gjk = GJK2::new();

        let self_start = glam_to_cgmath_pose(&start_pos_self);
        let self_end = glam_to_cgmath_pose(&end_pos_self);
        let other_start = glam_to_cgmath_pose(&start_pos_other);
        let other_end = glam_to_cgmath_pose(&end_pos_other);

        gjk.intersection_complex_time_of_impact(
            &CollisionStrategy::CollisionOnly,
            &self.finite,
            &self_start..&self_end,
            &other.finite,
            &other_start..&other_end,
        )
        .is_some()
    }

    fn half_spaces_collide(&self, self_pose: DPose2, other: &Self, other_pose: DPose2) -> bool {
        self.half_spaces.iter().any(|left| {
            other
                .half_spaces
                .iter()
                .any(|right| half_spaces_collide(left, self_pose, right, other_pose))
                || other
                    .finite
                    .iter()
                    .any(|finite| half_space_hits_finite(left, self_pose, finite, other_pose))
        }) || other.half_spaces.iter().any(|right| {
            self.finite
                .iter()
                .any(|finite| half_space_hits_finite(right, other_pose, finite, self_pose))
        })
    }

    fn finite_motion_bounds_overlap(
        &self,
        start: DPose2,
        end: DPose2,
        other: &Self,
        other_start: DPose2,
        other_end: DPose2,
    ) -> bool {
        self.finite
            .iter()
            .zip(&self.finite_motion_radii)
            .any(|(finite, radius)| {
                let (left_min, left_max) = finite_motion_bound(finite, *radius, start, end);

                other.finite.iter().zip(&other.finite_motion_radii).any(
                    |(other_finite, other_radius)| {
                        let (right_min, right_max) = finite_motion_bound(
                            other_finite,
                            *other_radius,
                            other_start,
                            other_end,
                        );

                        left_min.x <= right_max.x
                            && right_min.x <= left_max.x
                            && left_min.y <= right_max.y
                            && right_min.y <= left_max.y
                    },
                )
            })
    }
}

fn finite_motion_bound(
    finite: &(collision::primitive::Primitive2<f64>, BodyPose2<f64>),
    radius: f64,
    start: DPose2,
    end: DPose2,
) -> (DVec2, DVec2) {
    if rotation_changed(start, end) {
        let radius = DVec2::splat(radius);

        return (
            start.translation.min(end.translation).sub(radius),
            start.translation.max(end.translation).add(radius),
        );
    }

    let (start_min, start_max) = finite_aabb(finite, start);
    let (end_min, end_max) = finite_aabb(finite, end);

    (start_min.min(end_min), start_max.max(end_max))
}

fn finite_aabb(
    finite: &(collision::primitive::Primitive2<f64>, BodyPose2<f64>),
    pose: DPose2,
) -> (DVec2, DVec2) {
    let right = finite_support_point(finite, pose, DVec2::X);
    let left = finite_support_point(finite, pose, DVec2::NEG_X);
    let top = finite_support_point(finite, pose, DVec2::Y);
    let bottom = finite_support_point(finite, pose, DVec2::NEG_Y);

    (DVec2::new(left.x, bottom.y), DVec2::new(right.x, top.y))
}

fn half_space_hits_finite(
    half_space: &HalfSpaceComponent,
    half_space_pose: DPose2,
    finite: &(collision::primitive::Primitive2<f64>, BodyPose2<f64>),
    finite_pose: DPose2,
) -> bool {
    let world_half_space = transform_half_space(half_space, half_space_pose);

    let support_point =
        finite_support_point(finite, finite_pose, world_half_space.outward_normal.neg());

    world_half_space.outward_normal.dot(support_point)
        <= world_half_space.offset.add(HALF_SPACE_EPSILON)
}

fn half_spaces_collide(
    left: &HalfSpaceComponent,
    left_pose: DPose2,
    right: &HalfSpaceComponent,
    right_pose: DPose2,
) -> bool {
    let left = transform_half_space(left, left_pose);
    let right = transform_half_space(right, right_pose);

    let normals_are_opposite =
        left.outward_normal.add(right.outward_normal).length() <= HALF_SPACE_EPSILON;

    if normals_are_opposite {
        left.offset.add(right.offset) >= HALF_SPACE_EPSILON.neg()
    } else {
        true
    }
}

fn transform_half_space(half_space: &HalfSpaceComponent, pose: DPose2) -> HalfSpaceComponent {
    let outward_normal = pose.rotation.mul(half_space.outward_normal);

    HalfSpaceComponent {
        outward_normal,
        offset: half_space.offset.add(outward_normal.dot(pose.translation)),
    }
}

fn finite_support_point(
    finite: &(collision::primitive::Primitive2<f64>, BodyPose2<f64>),
    pose: DPose2,
    direction: DVec2,
) -> DVec2 {
    let world_pose = glam_to_cgmath_pose(&pose).concat(&finite.1);

    let point = finite
        .0
        .support_point(&Vector2::new(direction.x, direction.y), &world_pose);

    DVec2::new(point.x, point.y)
}

impl From<CollisionObject> for RhusicsCoreCollisionObjectInner {
    fn from(value: CollisionObject) -> Self {
        let mut finite = Vec::new();
        let mut finite_motion_radii = Vec::new();
        let mut half_spaces = Vec::new();

        for simple in value {
            let converted = RhusicsCoreSimpleCollisionObject::from(simple);

            match converted {
                RhusicsCoreSimpleCollisionObject::Empty => {}

                RhusicsCoreSimpleCollisionObject::FullSpace => {
                    return Self::FullSpace;
                }

                other => {
                    for component in other.into_components() {
                        match component {
                            RhusicsCoreCollisionComponent::Finite(shape) => {
                                finite_motion_radii.push(shape.motion_radius);

                                finite.push((shape.primitive, shape.position));
                            }

                            RhusicsCoreCollisionComponent::HalfSpace(half_space) => {
                                half_spaces.push(half_space);
                            }
                        }
                    }
                }
            }
        }

        if finite.is_empty() && half_spaces.is_empty() {
            Self::Empty
        } else {
            Self::NonTrivial(NonTrivial {
                finite,
                finite_motion_radii,
                half_spaces,
            })
        }
    }
}
