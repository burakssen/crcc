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
const FINITE_COLLISION_EPSILON: f64 = 1e-12;
const HALF_SPACE_TOI_TIME_TOLERANCE: f64 = 1e-9;
const HALF_SPACE_TOI_INITIAL_SAMPLES: usize = 64;
const HALF_SPACE_TOI_MAX_DEPTH: usize = 4;

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
                if components_hit_continuous(
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
        ) => half_space_hits_finite(half_space, left_pose, finite, right_pose),
        (
            RhusicsCoreCollisionComponent::Finite(finite),
            RhusicsCoreCollisionComponent::HalfSpace(half_space),
        ) => half_space_hits_finite(half_space, right_pose, finite, left_pose),
    }
}

fn components_hit_continuous(
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
        ) => finite_hit_continuous(
            gjk,
            left,
            left_start_pose,
            left_end_pose,
            right,
            right_start_pose,
            right_end_pose,
        ),
        _ => RhusicsContinuousPair {
            gjk,
            left,
            left_start_pose,
            left_end_pose,
            right,
            right_start_pose,
            right_end_pose,
        }
        .collides(),
    }
}

struct RhusicsContinuousPair<'a> {
    gjk: &'a GJK2<f64>,
    left: &'a RhusicsCoreCollisionComponent,
    left_start_pose: DPose2,
    left_end_pose: DPose2,
    right: &'a RhusicsCoreCollisionComponent,
    right_start_pose: DPose2,
    right_end_pose: DPose2,
}

impl RhusicsContinuousPair<'_> {
    fn collides(&self) -> bool {
        if self.collides_at(0.0) || self.collides_at(1.0) {
            return true;
        }

        let mut previous_t = 0.0;
        for index in 1..=HALF_SPACE_TOI_INITIAL_SAMPLES {
            let t = index as f64 / HALF_SPACE_TOI_INITIAL_SAMPLES as f64;
            if self.collides_at(t) || self.interval_collides(previous_t, t, 0) {
                return true;
            }
            previous_t = t;
        }
        false
    }

    fn collides_at(&self, t: f64) -> bool {
        components_collide(
            self.gjk,
            self.left,
            lerp_pose(self.left_start_pose, self.left_end_pose, t),
            self.right,
            lerp_pose(self.right_start_pose, self.right_end_pose, t),
        )
    }

    fn interval_collides(&self, t0: f64, t1: f64, depth: usize) -> bool {
        let mid = (t0 + t1) / 2.0;
        if self.collides_at(mid) {
            return true;
        }

        if t1 - t0 <= HALF_SPACE_TOI_TIME_TOLERANCE || depth >= HALF_SPACE_TOI_MAX_DEPTH {
            return false;
        }

        self.interval_collides(t0, mid, depth + 1) || self.interval_collides(mid, t1, depth + 1)
    }
}

fn finite_shapes_collide(
    gjk: &GJK2<f64>,
    left: &FiniteShape,
    left_pose: DPose2,
    right: &FiniteShape,
    right_pose: DPose2,
) -> bool {
    if let (
        FiniteShapeSupport::Circle {
            radius: left_radius,
        },
        FiniteShapeSupport::Circle {
            radius: right_radius,
        },
    ) = (&left.support, &right.support)
    {
        let left_center = (left_pose * left.position).translation;
        let right_center = (right_pose * right.position).translation;
        return left_center.distance(right_center)
            <= left_radius + right_radius + FINITE_COLLISION_EPSILON;
    }

    if let (
        FiniteShapeSupport::Vertices(left_vertices),
        FiniteShapeSupport::Vertices(right_vertices),
    ) = (&left.support, &right.support)
    {
        return convex_polygons_collide(
            &transform_vertices(left_vertices, left_pose * left.position),
            &transform_vertices(right_vertices, right_pose * right.position),
        );
    }

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

fn transform_vertices(vertices: &[DVec2], pose: DPose2) -> Vec<DVec2> {
    vertices.iter().map(|vertex| pose * *vertex).collect()
}

fn convex_polygons_collide(left: &[DVec2], right: &[DVec2]) -> bool {
    !has_separating_axis(left, left, right) && !has_separating_axis(right, left, right)
}

fn has_separating_axis(axis_source: &[DVec2], left: &[DVec2], right: &[DVec2]) -> bool {
    axis_source
        .iter()
        .zip(axis_source.iter().cycle().skip(1))
        .take(axis_source.len())
        .any(|(start, end)| {
            let edge = *end - *start;
            let axis = DVec2::new(-edge.y, edge.x).normalize_or_zero();
            if axis == DVec2::ZERO {
                return false;
            }
            let (left_min, left_max) = project_vertices(left, axis);
            let (right_min, right_max) = project_vertices(right, axis);
            left_max < right_min || right_max < left_min
        })
}

fn project_vertices(vertices: &[DVec2], axis: DVec2) -> (f64, f64) {
    vertices.iter().map(|vertex| vertex.dot(axis)).fold(
        (f64::INFINITY, f64::NEG_INFINITY),
        |(min, max), projection| (min.min(projection), max.max(projection)),
    )
}

fn finite_hit_continuous(
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

fn half_space_hits_finite(
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

fn lerp_pose(start: DPose2, end: DPose2, t: f64) -> DPose2 {
    DPose2::from_parts(
        start.translation.lerp(end.translation, t),
        start.rotation.slerp(&end.rotation, t),
    )
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
