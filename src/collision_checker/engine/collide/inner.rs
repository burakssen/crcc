use crate::collision_checker::engine::collide::simple::{
    CollideCollisionComponent, CollideSimpleCollisionObject, CollideVec2, FiniteShape,
    FiniteShapeSupport, HalfSpaceComponent, vec2,
};
use crate::collision_object::CollisionObject;
use crate::collision_object::simple::rotation_changed;
use collide::{Collider, CollisionInfo, Transform, Transformable};
use collide_convex::Convex as CollideConvex;
use collide_sphere::Sphere as CollideSphere;
use glamx::{DPose2, DVec2};
use std::cmp::Ordering;
use std::fmt;
use std::ops::{Add, Div, Mul, Neg, Sub};

const HALF_SPACE_EPSILON: f64 = 1e-9;
const ROTATION_EPSILON: f64 = 1e-12;
const LOCAL_OFFSET_EPSILON_SQUARED: f64 = 1e-24;
const TOI_TIME_TOLERANCE: f64 = 1e-9;
const TOI_MAX_DEPTH: usize = 10;
const TOI_SAMPLE_TIMES: [f64; 8] = [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0];

#[derive(Clone)]
pub enum CollideCollisionObjectInner {
    Empty,
    FullSpace,
    NonTrivial(Box<NonTrivial>),
}

impl fmt::Debug for CollideCollisionObjectInner {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(formatter, "Empty"),
            Self::FullSpace => write!(formatter, "FullSpace"),
            Self::NonTrivial(_) => write!(formatter, "NonTrivial([Components])"),
        }
    }
}

#[derive(Clone)]
pub struct NonTrivial {
    components: Vec<CollideCollisionComponent>,
}

impl CollideCollisionObjectInner {
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
        if finite_components_collide(&self.components, pos_self, &other.components, pos_other) {
            return true;
        }

        for component_self in &self.components {
            for component_other in &other.components {
                if matches!(
                    (component_self, component_other),
                    (
                        CollideCollisionComponent::Finite(_),
                        CollideCollisionComponent::Finite(_)
                    )
                ) {
                    continue;
                }

                if components_collide(component_self, pos_self, component_other, pos_other) {
                    return true;
                }
            }
        }

        false
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
        for component_self in &self.components {
            for component_other in &other.components {
                if let Some(collides) = moving_circle_pair_collides(
                    component_self,
                    start_pos_self,
                    end_pos_self,
                    component_other,
                    start_pos_other,
                    end_pos_other,
                ) {
                    if collides {
                        return true;
                    }

                    continue;
                }

                if components_hit_continuous(
                    component_self,
                    start_pos_self,
                    end_pos_self,
                    component_other,
                    start_pos_other,
                    end_pos_other,
                ) {
                    return true;
                }
            }
        }

        false
    }
}

fn moving_circle_pair_collides(
    component_self: &CollideCollisionComponent,
    start_pos_self: DPose2,
    end_pos_self: DPose2,
    component_other: &CollideCollisionComponent,
    start_pos_other: DPose2,
    end_pos_other: DPose2,
) -> Option<bool> {
    let (
        CollideCollisionComponent::Finite(shape_self),
        CollideCollisionComponent::Finite(shape_other),
    ) = (component_self, component_other)
    else {
        return None;
    };

    let (
        FiniteShapeSupport::Circle {
            radius: radius_self,
        },
        FiniteShapeSupport::Circle {
            radius: radius_other,
        },
    ) = (&shape_self.support, &shape_other.support)
    else {
        return None;
    };

    let self_rotation_supported = start_pos_self
        .rotation
        .angle()
        .sub(end_pos_self.rotation.angle())
        .abs()
        <= ROTATION_EPSILON
        || shape_self.position.translation.length_squared() <= LOCAL_OFFSET_EPSILON_SQUARED;

    let other_rotation_supported = start_pos_other
        .rotation
        .angle()
        .sub(end_pos_other.rotation.angle())
        .abs()
        <= ROTATION_EPSILON
        || shape_other.position.translation.length_squared() <= LOCAL_OFFSET_EPSILON_SQUARED;

    if !self_rotation_supported || !other_rotation_supported {
        return None;
    }

    let start_self = start_pos_self.mul(shape_self.position.translation);
    let end_self = end_pos_self.mul(shape_self.position.translation);
    let start_other = start_pos_other.mul(shape_other.position.translation);
    let end_other = end_pos_other.mul(shape_other.position.translation);

    let relative_start = start_self.sub(start_other);
    let combined_radius = (*radius_self).add(*radius_other);
    let combined_radius_squared = combined_radius.mul(combined_radius);

    if relative_start.length_squared() <= combined_radius_squared {
        return Some(true);
    }

    let velocity_self = end_self.sub(start_self);
    let velocity_other = end_other.sub(start_other);
    let relative_velocity = velocity_self.sub(velocity_other);
    let velocity_squared = relative_velocity.length_squared();

    if velocity_squared <= 0.0 {
        return Some(false);
    }

    let closest_time = relative_start
        .dot(relative_velocity)
        .neg()
        .div(velocity_squared)
        .clamp(0.0, 1.0);
    let closest = relative_start.add(relative_velocity.mul(closest_time));

    Some(closest.length_squared() <= combined_radius_squared)
}

fn finite_components_collide(
    left_components: &[CollideCollisionComponent],
    left_pose: DPose2,
    right_components: &[CollideCollisionComponent],
    right_pose: DPose2,
) -> bool {
    left_components.iter().any(|left| {
        let CollideCollisionComponent::Finite(left) = left else {
            return false;
        };

        right_components.iter().any(|right| {
            let CollideCollisionComponent::Finite(right) = right else {
                return false;
            };

            finite_shapes_collide(left, left_pose, right, right_pose)
        })
    })
}

#[derive(Clone)]
struct ManagedFiniteShape {
    collider: CollideConvex<CollideVec2>,
    bounding_sphere: CollideSphere<CollideVec2>,
}

impl ManagedFiniteShape {
    fn new(finite: &FiniteShape, pose: DPose2) -> Self {
        let global_pose = pose.mul(finite.position);

        Self {
            collider: transformed_convex_at_pose(finite, global_pose),
            bounding_sphere: transformed_bounding_sphere_at_pose(finite, global_pose),
        }
    }
}

impl Collider for ManagedFiniteShape {
    type Vector = CollideVec2;

    fn check_collision(&self, other: &Self) -> bool {
        self.bounding_sphere.check_collision(&other.bounding_sphere)
    }

    fn collision_info(&self, other: &Self) -> Option<CollisionInfo<Self::Vector>> {
        self.collider.collision_info(&other.collider)
    }
}

fn components_collide(
    left: &CollideCollisionComponent,
    left_pose: DPose2,
    right: &CollideCollisionComponent,
    right_pose: DPose2,
) -> bool {
    match (left, right) {
        (CollideCollisionComponent::Finite(left), CollideCollisionComponent::Finite(right)) => {
            finite_shapes_collide(left, left_pose, right, right_pose)
        }
        (
            CollideCollisionComponent::HalfSpace(left),
            CollideCollisionComponent::HalfSpace(right),
        ) => half_spaces_collide(left, left_pose, right, right_pose),
        (
            CollideCollisionComponent::HalfSpace(half_space),
            CollideCollisionComponent::Finite(finite),
        ) => half_space_hits_finite(half_space, left_pose, finite, right_pose),
        (
            CollideCollisionComponent::Finite(finite),
            CollideCollisionComponent::HalfSpace(half_space),
        ) => half_space_hits_finite(half_space, right_pose, finite, left_pose),
    }
}

fn components_hit_continuous(
    left: &CollideCollisionComponent,
    left_start_pose: DPose2,
    left_end_pose: DPose2,
    right: &CollideCollisionComponent,
    right_start_pose: DPose2,
    right_end_pose: DPose2,
) -> bool {
    let pair = ContinuousPair {
        left,
        left_start_pose,
        left_end_pose,
        right,
        right_start_pose,
        right_end_pose,
    };

    if !pair.interval_may_collide(0.0, 1.0) {
        return false;
    }

    if pair.collides_at(0.0) || pair.collides_at(1.0) {
        return true;
    }

    if left_start_pose == left_end_pose && right_start_pose == right_end_pose {
        return false;
    }

    let mut previous_time = 0.0;

    for sample_time in TOI_SAMPLE_TIMES {
        if pair.collides_at(sample_time)
            || (pair.interval_may_collide(previous_time, sample_time)
                && pair.interval_collides(previous_time, sample_time, 0))
        {
            return true;
        }

        previous_time = sample_time;
    }

    false
}

struct ContinuousPair<'a> {
    left: &'a CollideCollisionComponent,
    left_start_pose: DPose2,
    left_end_pose: DPose2,
    right: &'a CollideCollisionComponent,
    right_start_pose: DPose2,
    right_end_pose: DPose2,
}

impl ContinuousPair<'_> {
    fn collides_at(&self, time: f64) -> bool {
        components_collide(
            self.left,
            lerp_pose(self.left_start_pose, self.left_end_pose, time),
            self.right,
            lerp_pose(self.right_start_pose, self.right_end_pose, time),
        )
    }

    fn interval_collides(&self, start_time: f64, end_time: f64, depth: usize) -> bool {
        let midpoint = f64::midpoint(start_time, end_time);

        if self.collides_at(midpoint) {
            return true;
        }

        if end_time.sub(start_time) <= TOI_TIME_TOLERANCE || depth >= TOI_MAX_DEPTH {
            // The broad phase still overlaps, so absence of collision is unproven.
            return true;
        }

        let Some(next_depth) = depth.checked_add(1) else {
            return true;
        };

        (self.interval_may_collide(start_time, midpoint)
            && self.interval_collides(start_time, midpoint, next_depth))
            || (self.interval_may_collide(midpoint, end_time)
                && self.interval_collides(midpoint, end_time, next_depth))
    }

    fn interval_may_collide(&self, start_time: f64, end_time: f64) -> bool {
        match (
            component_interval_aabb(
                self.left,
                self.left_start_pose,
                self.left_end_pose,
                start_time,
                end_time,
            ),
            component_interval_aabb(
                self.right,
                self.right_start_pose,
                self.right_end_pose,
                start_time,
                end_time,
            ),
        ) {
            (Some(left), Some(right)) => left.overlaps(right),
            // Infinite half-spaces cannot be rejected using finite AABBs.
            _ => true,
        }
    }
}

fn finite_shapes_collide(
    left: &FiniteShape,
    left_pose: DPose2,
    right: &FiniteShape,
    right_pose: DPose2,
) -> bool {
    let left = ManagedFiniteShape::new(left, left_pose);
    let right = ManagedFiniteShape::new(right, right_pose);

    left.check_collision(&right) && left.collision_info(&right).is_some()
}

fn transformed_convex_at_pose(finite: &FiniteShape, pose: DPose2) -> CollideConvex<CollideVec2> {
    finite.collider.transformed(&CollideTransform(pose))
}

fn transformed_bounding_sphere_at_pose(
    finite: &FiniteShape,
    pose: DPose2,
) -> CollideSphere<CollideVec2> {
    CollideSphere::new(
        transform_collide_point(finite.bounding_sphere.center, pose),
        finite.bounding_sphere.radius,
    )
}

#[derive(Clone, Copy, Debug)]
struct CollideTransform(DPose2);

impl Transform<CollideVec2> for CollideTransform {
    fn apply_point(&self, point: CollideVec2) -> CollideVec2 {
        transform_collide_point(point, self.0)
    }
}

fn transform_collide_point(point: CollideVec2, pose: DPose2) -> CollideVec2 {
    let [x, y]: [f64; 2] = point.into();
    vec2(pose.mul(DVec2::new(x, y)))
}

fn half_space_hits_finite(
    half_space: &HalfSpaceComponent,
    half_space_pose: DPose2,
    finite: &FiniteShape,
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

    // A tolerance here changes nonparallel, intersecting sets into parallel ones.
    let normals_are_opposite = left.outward_normal.perp_dot(right.outward_normal).abs() <= 0.0
        && left.outward_normal.dot(right.outward_normal) < 0.0;

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

fn finite_support_point(finite: &FiniteShape, pose: DPose2, direction: DVec2) -> DVec2 {
    let global_pose = pose.mul(finite.position);

    match &finite.support {
        FiniteShapeSupport::Circle { radius } => global_pose
            .translation
            .add(direction.normalize_or_zero().mul(*radius)),
        FiniteShapeSupport::Vertices(vertices) => vertices
            .iter()
            .map(|vertex| global_pose.mul(*vertex))
            .max_by(|left, right| {
                direction
                    .dot(*left)
                    .partial_cmp(&direction.dot(*right))
                    .unwrap_or(Ordering::Equal)
            })
            .unwrap_or(global_pose.translation),
    }
}

fn component_interval_aabb(
    component: &CollideCollisionComponent,
    start_pose: DPose2,
    end_pose: DPose2,
    start_time: f64,
    end_time: f64,
) -> Option<Aabb> {
    match component {
        CollideCollisionComponent::Finite(finite) => {
            let start = lerp_pose(start_pose, end_pose, start_time);
            let end = lerp_pose(start_pose, end_pose, end_time);

            if !rotation_changed(start, end) {
                return Some(finite_aabb_at(finite, start).union(finite_aabb_at(finite, end)));
            }

            let support_radius = match &finite.support {
                FiniteShapeSupport::Circle { radius } => *radius,
                FiniteShapeSupport::Vertices(vertices) => vertices
                    .iter()
                    .map(|vertex| vertex.length())
                    .fold(0.0, f64::max),
            };

            let radius = finite.position.translation.length().add(support_radius);
            let radius_vector = DVec2::splat(radius);

            Some(Aabb {
                min: start.translation.min(end.translation).sub(radius_vector),
                max: start.translation.max(end.translation).add(radius_vector),
            })
        }
        CollideCollisionComponent::HalfSpace(_) => None,
    }
}

fn finite_aabb_at(finite: &FiniteShape, pose: DPose2) -> Aabb {
    let global_pose = pose.mul(finite.position);

    match &finite.support {
        FiniteShapeSupport::Circle { radius } => {
            let radius_vector = DVec2::splat(*radius);

            Aabb {
                min: global_pose.translation.sub(radius_vector),
                max: global_pose.translation.add(radius_vector),
            }
        }
        FiniteShapeSupport::Vertices(vertices) => {
            let mut aabb = Aabb::empty();

            for vertex in vertices {
                aabb.include(global_pose.mul(*vertex));
            }

            aabb
        }
    }
}

#[derive(Clone, Copy)]
struct Aabb {
    min: DVec2,
    max: DVec2,
}

impl Aabb {
    const fn empty() -> Self {
        Self {
            min: DVec2::splat(f64::INFINITY),
            max: DVec2::splat(f64::NEG_INFINITY),
        }
    }

    fn include(&mut self, point: DVec2) {
        self.min = self.min.min(point);
        self.max = self.max.max(point);
    }

    fn union(self, other: Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    fn overlaps(self, other: Self) -> bool {
        self.min.x <= other.max.x
            && other.min.x <= self.max.x
            && self.min.y <= other.max.y
            && other.min.y <= self.max.y
    }
}

fn lerp_pose(start: DPose2, end: DPose2, time: f64) -> DPose2 {
    DPose2::from_parts(
        start.translation.lerp(end.translation, time),
        start.rotation.slerp(&end.rotation, time),
    )
}

impl From<CollisionObject> for CollideCollisionObjectInner {
    fn from(value: CollisionObject) -> Self {
        let mut components = Vec::new();

        for simple in value {
            match CollideSimpleCollisionObject::from(simple) {
                CollideSimpleCollisionObject::Empty => {}
                CollideSimpleCollisionObject::FullSpace => {
                    return Self::FullSpace;
                }
                other => components.extend(other.into_components()),
            }
        }

        if components.is_empty() {
            Self::Empty
        } else {
            Self::NonTrivial(Box::new(NonTrivial { components }))
        }
    }
}
