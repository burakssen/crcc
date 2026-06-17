use crate::collision_checker::engine::collide::simple::{
    CollideCollisionComponent, CollideSimpleCollisionObject, CollideVec2, FiniteShape,
    FiniteShapeSupport, HalfSpaceComponent, vec2,
};
use crate::collision_object::CollisionObject;
use collide::{Collider, Transform, Transformable};
use collide_convex::Convex as CollideConvex;
use collide_sphere::Sphere as CollideSphere;
use glamx::{DPose2, DVec2};
use std::fmt;

const HALF_SPACE_EPSILON: f64 = 1e-9;
const TOI_TIME_TOLERANCE: f64 = 1e-9;
const TOI_INITIAL_SAMPLES: usize = 8;
const TOI_MAX_DEPTH: usize = 10;

#[derive(Debug, Clone)]
pub struct Unsupported;

#[allow(clippy::large_enum_variant)]
#[derive(Clone)]
pub enum CollideCollisionObjectInner {
    Empty,
    FullSpace,
    NonTrivial(NonTrivial),
}

impl fmt::Debug for CollideCollisionObjectInner {
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
    components: Vec<CollideCollisionComponent>,
}

impl CollideCollisionObjectInner {
    pub fn collides(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<bool, Unsupported> {
        match (self, other) {
            (CollideCollisionObjectInner::Empty, _) | (_, CollideCollisionObjectInner::Empty) => {
                Ok(false)
            }
            (CollideCollisionObjectInner::FullSpace, _)
            | (_, CollideCollisionObjectInner::FullSpace) => Ok(true),
            (
                CollideCollisionObjectInner::NonTrivial(slf),
                CollideCollisionObjectInner::NonTrivial(other),
            ) => Ok(slf.collides(pos_self, other, pos_other)),
        }
    }

    pub fn collides_sweep(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> Result<bool, Unsupported> {
        match (self, other) {
            (CollideCollisionObjectInner::Empty, _) | (_, CollideCollisionObjectInner::Empty) => {
                Ok(false)
            }
            (CollideCollisionObjectInner::FullSpace, _)
            | (_, CollideCollisionObjectInner::FullSpace) => Ok(true),
            (
                CollideCollisionObjectInner::NonTrivial(slf),
                CollideCollisionObjectInner::NonTrivial(other),
            ) => Ok(slf.collides_sweep(
                start_pos_self,
                end_pos_self,
                other,
                start_pos_other,
                end_pos_other,
            )),
        }
    }
}

impl NonTrivial {
    pub fn collides(&self, pos_self: DPose2, other: &Self, pos_other: DPose2) -> bool {
        for component_self in &self.components {
            for component_other in &other.components {
                if components_collide(component_self, pos_self, component_other, pos_other) {
                    return true;
                }
            }
        }
        false
    }

    pub fn collides_sweep(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> bool {
        for component_self in &self.components {
            for component_other in &other.components {
                if components_hit_sweep(
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

fn components_hit_sweep(
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

    if pair.collides_at(0.0) || pair.collides_at(1.0) {
        return true;
    }
    if !pair.interval_may_collide(0.0, 1.0) {
        return false;
    }

    let mut previous_t = 0.0;
    for index in 1..=TOI_INITIAL_SAMPLES {
        let t = index as f64 / TOI_INITIAL_SAMPLES as f64;
        if pair.collides_at(t)
            || (pair.interval_may_collide(previous_t, t)
                && pair.interval_collides(previous_t, t, 0))
        {
            return true;
        }
        previous_t = t;
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
    fn collides_at(&self, t: f64) -> bool {
        components_collide(
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

        if t1 - t0 <= TOI_TIME_TOLERANCE || depth >= TOI_MAX_DEPTH {
            return false;
        }

        (self.interval_may_collide(t0, mid) && self.interval_collides(t0, mid, depth + 1))
            || (self.interval_may_collide(mid, t1) && self.interval_collides(mid, t1, depth + 1))
    }

    fn interval_may_collide(&self, t0: f64, t1: f64) -> bool {
        match (
            component_interval_aabb(self.left, self.left_start_pose, self.left_end_pose, t0, t1),
            component_interval_aabb(
                self.right,
                self.right_start_pose,
                self.right_end_pose,
                t0,
                t1,
            ),
        ) {
            (Some(left), Some(right)) => left.overlaps(right),
            // Infinite half-spaces cannot be rejected by finite AABBs.
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
    let left_pose = left_pose * left.position;
    let right_pose = right_pose * right.position;
    if !transformed_bounding_sphere(left, left_pose)
        .check_collision(&transformed_bounding_sphere(right, right_pose))
    {
        return false;
    }
    transformed_convex(left, left_pose)
        .collision_info(&transformed_convex(right, right_pose))
        .is_some()
}

fn transformed_convex(finite: &FiniteShape, pose: DPose2) -> CollideConvex<CollideVec2> {
    finite.collider.transformed(&CollideTransform(pose))
}

fn transformed_bounding_sphere(finite: &FiniteShape, pose: DPose2) -> CollideSphere<CollideVec2> {
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
    vec2(pose * DVec2::new(point[0], point[1]))
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

fn component_interval_aabb(
    component: &CollideCollisionComponent,
    start_pose: DPose2,
    end_pose: DPose2,
    t0: f64,
    t1: f64,
) -> Option<Aabb> {
    match component {
        CollideCollisionComponent::Finite(finite) => {
            let mut aabb: Option<Aabb> = None;
            // Endpoints plus interior points make rotating boxes/polygons rejectable while
            // preserving a deterministic bounded solver.
            for sample in [t0, (2.0 * t0 + t1) / 3.0, (t0 + 2.0 * t1) / 3.0, t1] {
                let pose = lerp_pose(start_pose, end_pose, sample);
                let sample_aabb = finite_aabb_at(finite, pose);
                aabb = Some(match aabb {
                    Some(existing) => existing.union(sample_aabb),
                    None => sample_aabb,
                });
            }
            aabb
        }
        CollideCollisionComponent::HalfSpace(_) => None,
    }
}

fn finite_aabb_at(finite: &FiniteShape, pose: DPose2) -> Aabb {
    let global_pose = pose * finite.position;
    match &finite.support {
        FiniteShapeSupport::Circle { radius } => Aabb {
            min: global_pose.translation - DVec2::splat(*radius),
            max: global_pose.translation + DVec2::splat(*radius),
        },
        FiniteShapeSupport::Vertices(vertices) => {
            let mut aabb = Aabb::empty();
            for vertex in vertices {
                aabb.include(global_pose * *vertex);
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
    fn empty() -> Self {
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

fn lerp_pose(start: DPose2, end: DPose2, t: f64) -> DPose2 {
    DPose2::from_parts(
        start.translation.lerp(end.translation, t),
        start.rotation.slerp(&end.rotation, t),
    )
}

impl From<CollisionObject> for CollideCollisionObjectInner {
    fn from(value: CollisionObject) -> Self {
        let mut components = Vec::new();

        for simple in value {
            let converted = CollideSimpleCollisionObject::from(simple);
            match converted {
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
            Self::NonTrivial(NonTrivial { components })
        }
    }
}
