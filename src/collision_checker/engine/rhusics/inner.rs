use crate::collision_checker::engine::rhusics::simple::{
    FiniteShape, FiniteShapeSupport, HalfSpaceComponent, RhusicsCoreCollisionComponent,
    RhusicsCoreSimpleCollisionObject,
};
use crate::collision_object::CollisionObject;
use cgmath::{Basis2, Point2, Rad, Rotation2};
use collision::algorithm::minkowski::GJK2;
use glamx::{DPose2, DVec2};
use rhusics_core::Pose;
use rhusics_core::collide2d::BodyPose2;
use std::fmt;

const HALF_SPACE_EPSILON: f64 = 1e-9;

#[derive(Debug, Clone)]
pub struct Unsupported(pub String);

#[allow(clippy::large_enum_variant)]
#[derive(Clone)]
pub enum RhusicsCoreCollisionObjectInner {
    Empty,
    FullSpace,
    NonTrivial(NonTrivial),
}

impl fmt::Debug for RhusicsCoreCollisionObjectInner {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(f, "Empty"),
            Self::FullSpace => write!(f, "FullSpace"),
            Self::NonTrivial(_) => write!(f, "NonTrivial([Components])"),
        }
    }
}

#[derive(Clone)]
pub struct NonTrivial {
    components: Vec<RhusicsCoreCollisionComponent>,
}

fn glam_to_cgmath_pose(pose: &DPose2) -> BodyPose2<f64> {
    BodyPose2::new(
        Point2::new(pose.translation.x, pose.translation.y),
        Basis2::from_angle(Rad(pose.rotation.angle())),
    )
}

impl RhusicsCoreCollisionObjectInner {
    pub fn collides(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<bool, Unsupported> {
        match (self, other) {
            (RhusicsCoreCollisionObjectInner::Empty, _)
            | (_, RhusicsCoreCollisionObjectInner::Empty) => Ok(false),
            (RhusicsCoreCollisionObjectInner::FullSpace, _)
            | (_, RhusicsCoreCollisionObjectInner::FullSpace) => Ok(true),
            (
                RhusicsCoreCollisionObjectInner::NonTrivial(slf),
                RhusicsCoreCollisionObjectInner::NonTrivial(other),
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
            (RhusicsCoreCollisionObjectInner::Empty, _)
            | (_, RhusicsCoreCollisionObjectInner::Empty) => Ok(false),
            (RhusicsCoreCollisionObjectInner::FullSpace, _)
            | (_, RhusicsCoreCollisionObjectInner::FullSpace) => Ok(true),
            (
                RhusicsCoreCollisionObjectInner::NonTrivial(slf),
                RhusicsCoreCollisionObjectInner::NonTrivial(other),
            ) => slf.collides_continuous(
                start_pos_self,
                end_pos_self,
                other,
                start_pos_other,
                end_pos_other,
            ),
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
        let gjk = GJK2::new();

        for component_self in &self.components {
            for component_other in &other.components {
                if components_collide(&gjk, component_self, pos_self, component_other, pos_other) {
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
        let gjk = GJK2::new();

        for component_self in &self.components {
            for component_other in &other.components {
                if components_collide_continuous(
                    &gjk,
                    component_self,
                    start_pos_self,
                    end_pos_self,
                    component_other,
                    start_pos_other,
                    end_pos_other,
                ) {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }
}

fn components_collide(
    gjk: &GJK2<f64>,
    left: &RhusicsCoreCollisionComponent,
    left_pose: DPose2,
    right: &RhusicsCoreCollisionComponent,
    right_pose: DPose2,
) -> bool {
    match (left, right) {
        (
            RhusicsCoreCollisionComponent::Finite(left),
            RhusicsCoreCollisionComponent::Finite(right),
        ) => finite_shapes_collide(gjk, left, left_pose, right, right_pose),
        (
            RhusicsCoreCollisionComponent::HalfSpace(left),
            RhusicsCoreCollisionComponent::HalfSpace(right),
        ) => half_spaces_collide(left, left_pose, right, right_pose),
        (
            RhusicsCoreCollisionComponent::HalfSpace(half_space),
            RhusicsCoreCollisionComponent::Finite(finite),
        ) => half_space_collides_finite(half_space, left_pose, finite, right_pose),
        (
            RhusicsCoreCollisionComponent::Finite(finite),
            RhusicsCoreCollisionComponent::HalfSpace(half_space),
        ) => half_space_collides_finite(half_space, right_pose, finite, left_pose),
    }
}

fn components_collide_continuous(
    gjk: &GJK2<f64>,
    left: &RhusicsCoreCollisionComponent,
    left_start_pose: DPose2,
    left_end_pose: DPose2,
    right: &RhusicsCoreCollisionComponent,
    right_start_pose: DPose2,
    right_end_pose: DPose2,
) -> bool {
    match (left, right) {
        (
            RhusicsCoreCollisionComponent::Finite(left),
            RhusicsCoreCollisionComponent::Finite(right),
        ) => finite_shapes_collide_continuous(
            gjk,
            left,
            left_start_pose,
            left_end_pose,
            right,
            right_start_pose,
            right_end_pose,
        ),
        _ => {
            components_collide(gjk, left, left_start_pose, right, right_start_pose)
                || components_collide(gjk, left, left_end_pose, right, right_end_pose)
        }
    }
}

fn finite_shapes_collide(
    gjk: &GJK2<f64>,
    left: &FiniteShape,
    left_pose: DPose2,
    right: &FiniteShape,
    right_pose: DPose2,
) -> bool {
    let left_global_pose = left_pose * left.position;
    let right_global_pose = right_pose * right.position;
    let left_cg_pose = glam_to_cgmath_pose(&left_global_pose);
    let right_cg_pose = glam_to_cgmath_pose(&right_global_pose);

    gjk.intersect(
        &left.primitive,
        &left_cg_pose,
        &right.primitive,
        &right_cg_pose,
    )
    .is_some()
}

fn finite_shapes_collide_continuous(
    gjk: &GJK2<f64>,
    left: &FiniteShape,
    left_start_pose: DPose2,
    left_end_pose: DPose2,
    right: &FiniteShape,
    right_start_pose: DPose2,
    right_end_pose: DPose2,
) -> bool {
    let left_start_global_pose = left_start_pose * left.position;
    let left_end_global_pose = left_end_pose * left.position;
    let right_start_global_pose = right_start_pose * right.position;
    let right_end_global_pose = right_end_pose * right.position;
    let left_cg_start_pose = glam_to_cgmath_pose(&left_start_global_pose);
    let left_cg_end_pose = glam_to_cgmath_pose(&left_end_global_pose);
    let right_cg_start_pose = glam_to_cgmath_pose(&right_start_global_pose);
    let right_cg_end_pose = glam_to_cgmath_pose(&right_end_global_pose);

    gjk.intersection_time_of_impact(
        &left.primitive,
        &left_cg_start_pose..&left_cg_end_pose,
        &right.primitive,
        &right_cg_start_pose..&right_cg_end_pose,
    )
    .is_some()
}

fn half_space_collides_finite(
    half_space: &HalfSpaceComponent,
    half_space_pose: DPose2,
    finite: &FiniteShape,
    finite_pose: DPose2,
) -> bool {
    let world_half_space = transform_half_space(half_space, half_space_pose);
    let support_point = finite_support_point(finite, finite_pose, -world_half_space.outward_normal);
    world_half_space.outward_normal.dot(support_point)
        <= world_half_space.offset + HALF_SPACE_EPSILON
}

fn half_spaces_collide(
    left: &HalfSpaceComponent,
    left_pose: DPose2,
    right: &HalfSpaceComponent,
    right_pose: DPose2,
) -> bool {
    let left = transform_half_space(left, left_pose);
    let right = transform_half_space(right, right_pose);

    if (left.outward_normal + right.outward_normal).length() <= HALF_SPACE_EPSILON {
        left.offset + right.offset >= -HALF_SPACE_EPSILON
    } else {
        true
    }
}

fn transform_half_space(half_space: &HalfSpaceComponent, pose: DPose2) -> HalfSpaceComponent {
    let outward_normal = pose.rotation * half_space.outward_normal;
    HalfSpaceComponent {
        outward_normal,
        offset: half_space.offset + outward_normal.dot(pose.translation),
    }
}

fn finite_support_point(finite: &FiniteShape, pose: DPose2, direction: DVec2) -> DVec2 {
    let global_pose = pose * finite.position;
    match &finite.support {
        FiniteShapeSupport::Circle { radius } => {
            global_pose.translation + direction.normalize_or_zero() * *radius
        }
        FiniteShapeSupport::Vertices(vertices) => vertices
            .iter()
            .map(|vertex| global_pose * *vertex)
            .max_by(|left, right| {
                direction
                    .dot(*left)
                    .partial_cmp(&direction.dot(*right))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .expect("Finite polygonal shapes should have vertices"),
    }
}

impl From<CollisionObject> for RhusicsCoreCollisionObjectInner {
    fn from(value: CollisionObject) -> Self {
        let mut components = Vec::new();

        for simple in value {
            let converted = RhusicsCoreSimpleCollisionObject::from(simple);
            match converted {
                RhusicsCoreSimpleCollisionObject::Empty => {}
                RhusicsCoreSimpleCollisionObject::FullSpace => {
                    return Self::FullSpace;
                }
                other => components.extend(other.into_components()),
            }
        }

        if components.is_empty() {
            Self::Empty
        } else {
            Self::NonTrivial(NonTrivial { components })
        }
    }
}
