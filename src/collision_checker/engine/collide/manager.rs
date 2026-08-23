use crate::collision_checker::engine::collide::simple::{
    CollideCollisionComponent, CollideVec2, FiniteShape,
};
use collide::{BoundedCollider, Collider, Transform, Transformable};
use collide_convex::Convex as CollideConvex;
use collide_sphere::Sphere as CollideSphere;
use collision_detection::CollisionManager;
use glamx::{DPose2, DVec2};
use std::ops::Mul;

type ManagedFiniteShape = BoundedCollider<CollideSphere<CollideVec2>, CollideConvex<CollideVec2>>;

pub(super) struct FiniteManager {
    manager: CollisionManager<ManagedFiniteShape, usize>,
}

impl FiniteManager {
    pub(super) fn from_components(components: &[CollideCollisionComponent]) -> Self {
        let finite_count = finite_component_count(components);
        let mut manager = CollisionManager::with_capacity(finite_count);

        for (component_index, component) in components.iter().enumerate() {
            if let CollideCollisionComponent::Finite(finite) = component {
                manager.insert_collider(
                    managed_finite_shape(finite, DPose2::IDENTITY),
                    component_index,
                );
            }
        }

        Self { manager }
    }

    fn check_collision(&self, finite: &FiniteShape, pose: DPose2) -> bool {
        // ponytail: cache the exact short-circuit manager before introducing a frozen BVH.
        self.manager
            .check_collision(&managed_finite_shape(finite, pose))
    }
}

pub(super) fn finite_components_collide(
    left_components: &[CollideCollisionComponent],
    left_pose: DPose2,
    left_manager: &FiniteManager,
    right_components: &[CollideCollisionComponent],
    right_pose: DPose2,
    right_manager: &FiniteManager,
) -> bool {
    let left_count = finite_component_count(left_components);
    let right_count = finite_component_count(right_components);

    if left_count == 0 || right_count == 0 {
        return false;
    }

    if left_count == 1 && right_count == 1 {
        let Some(left) = only_finite_component(left_components) else {
            return false;
        };
        let Some(right) = only_finite_component(right_components) else {
            return false;
        };
        return finite_shapes_collide(left, left_pose, right, right_pose);
    }

    // Probe the larger cached set with the smaller set to minimize conversions.
    if left_count >= right_count {
        let relative_pose = left_pose.inv_mul(&right_pose);
        right_components.iter().any(|component| {
            let CollideCollisionComponent::Finite(finite) = component else {
                return false;
            };
            left_manager.check_collision(finite, relative_pose)
        })
    } else {
        let relative_pose = right_pose.inv_mul(&left_pose);
        left_components.iter().any(|component| {
            let CollideCollisionComponent::Finite(finite) = component else {
                return false;
            };
            right_manager.check_collision(finite, relative_pose)
        })
    }
}

pub(super) fn finite_shapes_collide(
    left: &FiniteShape,
    left_pose: DPose2,
    right: &FiniteShape,
    right_pose: DPose2,
) -> bool {
    let left = managed_finite_shape(left, left_pose);
    let right = managed_finite_shape(right, right_pose);

    left.check_collision(&right)
}

fn finite_component_count(components: &[CollideCollisionComponent]) -> usize {
    components
        .iter()
        .filter(|component| matches!(component, CollideCollisionComponent::Finite(_)))
        .count()
}

fn only_finite_component(components: &[CollideCollisionComponent]) -> Option<&FiniteShape> {
    components.iter().find_map(|component| match component {
        CollideCollisionComponent::Finite(finite) => Some(finite),
        CollideCollisionComponent::HalfSpace(_) => None,
    })
}

fn managed_finite_shape(finite: &FiniteShape, pose: DPose2) -> ManagedFiniteShape {
    let global_pose = pose.mul(finite.position);

    ManagedFiniteShape {
        inner: transformed_convex_at_pose(finite, global_pose),
        bounding: transformed_bounding_sphere_at_pose(finite, global_pose),
    }
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
    let transformed = pose.mul(DVec2::new(x, y));
    CollideVec2::new([transformed.x, transformed.y])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision_checker::engine::collide::simple::FiniteShapeSupport;
    use collide::Bounded;
    use std::hint::black_box;
    use std::time::Instant;

    fn rectangle_shape(width: f64, height: f64, center_y: f64) -> FiniteShape {
        let half_width = width / 2.0;
        let half_height = height / 2.0;
        let vertices = vec![
            DVec2::new(-half_width, center_y - half_height),
            DVec2::new(half_width, center_y - half_height),
            DVec2::new(half_width, center_y + half_height),
            DVec2::new(-half_width, center_y + half_height),
        ];
        let collider = CollideConvex::new(
            0.0,
            vertices
                .iter()
                .copied()
                .map(|vertex| CollideVec2::new([vertex.x, vertex.y]))
                .collect(),
        );

        FiniteShape {
            bounding_sphere: collider.bounding_volume(),
            collider,
            position: DPose2::IDENTITY,
            support: FiniteShapeSupport::Vertices(vertices),
        }
    }

    fn finite(shape: FiniteShape) -> CollideCollisionComponent {
        CollideCollisionComponent::Finite(shape)
    }

    fn finite_components_collide_legacy(
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

    #[test]
    fn manager_requires_exact_collision_after_bounding_sphere_overlap() {
        let left = vec![
            finite(rectangle_shape(20.0, 0.2, 0.0)),
            finite(rectangle_shape(1.0, 1.0, 20.0)),
        ];
        let right = vec![
            finite(rectangle_shape(20.0, 0.2, 1.0)),
            finite(rectangle_shape(1.0, 1.0, 30.0)),
        ];

        let left_manager = FiniteManager::from_components(&left);
        let right_manager = FiniteManager::from_components(&right);

        assert!(!finite_components_collide(
            &left,
            DPose2::IDENTITY,
            &left_manager,
            &right,
            DPose2::IDENTITY,
            &right_manager,
        ));
    }

    #[test]
    fn cached_manager_matches_direct_collision_result() {
        let left = vec![
            finite(rectangle_shape(1.0, 1.0, 0.0)),
            finite(rectangle_shape(1.0, 1.0, 10.0)),
        ];
        let right = vec![
            finite(rectangle_shape(1.0, 1.0, 0.0)),
            finite(rectangle_shape(1.0, 1.0, 20.0)),
        ];
        let left_manager = FiniteManager::from_components(&left);
        let right_manager = FiniteManager::from_components(&right);
        let direct =
            finite_components_collide_legacy(&left, DPose2::IDENTITY, &right, DPose2::IDENTITY);
        let cached = finite_components_collide(
            &left,
            DPose2::IDENTITY,
            &left_manager,
            &right,
            DPose2::IDENTITY,
            &right_manager,
        );

        assert_eq!(direct, cached);
    }

    #[test]
    fn cached_manager_route_matches_direct_collision() {
        let count = 256;
        let left = (0..count)
            .map(|index| {
                let index = f64::from(u32::try_from(index).unwrap_or_default());
                finite(rectangle_shape(1.0, 1.0, index * 3.0))
            })
            .collect::<Vec<_>>();
        let right = (0..count)
            .map(|index| {
                let index = f64::from(u32::try_from(index).unwrap_or_default());
                finite(rectangle_shape(1.0, 1.0, index.mul_add(3.0, 1.5)))
            })
            .collect::<Vec<_>>();
        let left_manager = FiniteManager::from_components(&left);
        let right_manager = FiniteManager::from_components(&right);
        let direct =
            finite_components_collide_legacy(&left, DPose2::IDENTITY, &right, DPose2::IDENTITY);
        let routed = finite_components_collide(
            &left,
            DPose2::IDENTITY,
            &left_manager,
            &right,
            DPose2::IDENTITY,
            &right_manager,
        );

        assert_eq!(direct, routed);
    }

    #[test]
    fn manager_matches_direct_collision_for_multiple_components() {
        let left = vec![
            finite(rectangle_shape(1.0, 1.0, 0.0)),
            finite(rectangle_shape(1.0, 1.0, 10.0)),
        ];
        let right = vec![
            finite(rectangle_shape(1.0, 1.0, 20.0)),
            finite(rectangle_shape(1.0, 1.0, 0.0)),
        ];

        let legacy =
            finite_components_collide_legacy(&left, DPose2::IDENTITY, &right, DPose2::IDENTITY);
        let left_manager = FiniteManager::from_components(&left);
        let right_manager = FiniteManager::from_components(&right);
        let manager = finite_components_collide(
            &left,
            DPose2::IDENTITY,
            &left_manager,
            &right,
            DPose2::IDENTITY,
            &right_manager,
        );

        assert!(legacy);
        assert_eq!(legacy, manager);
    }

    #[test]
    fn cached_manager_applies_relative_pose_to_probe_components() {
        let left = vec![
            finite(rectangle_shape(1.0, 1.0, 0.0)),
            finite(rectangle_shape(1.0, 1.0, 20.0)),
        ];
        let right = vec![
            finite(rectangle_shape(1.0, 1.0, 0.0)),
            finite(rectangle_shape(1.0, 1.0, 30.0)),
        ];
        let left_pose = DPose2::translation(10.0, 5.0);
        let right_pose = DPose2::translation(10.5, 5.0);
        let left_manager = FiniteManager::from_components(&left);
        let right_manager = FiniteManager::from_components(&right);

        assert!(finite_components_collide(
            &left,
            left_pose,
            &left_manager,
            &right,
            right_pose,
            &right_manager,
        ));
        assert_eq!(
            finite_components_collide_legacy(&left, left_pose, &right, right_pose),
            finite_components_collide(
                &right,
                right_pose,
                &right_manager,
                &left,
                left_pose,
                &left_manager,
            )
        );
    }

    #[test]
    #[ignore = "manual finite collision strategy benchmark"]
    fn benchmark_finite_collision_strategies() {
        const ITERATIONS: usize = 1_000;
        const COUNTS: [usize; 6] = [1, 4, 16, 64, 256, 1024];

        println!("workload,count,strategy,ns_per_query,result");
        for workload in ["clear", "first_hit", "dense_hit"] {
            for count in COUNTS {
                let left = (0..count)
                    .map(|index| {
                        let index = f64::from(u32::try_from(index).unwrap_or_default());
                        finite(rectangle_shape(1.0, 1.0, index * 3.0))
                    })
                    .collect::<Vec<_>>();
                let right = (0..count)
                    .map(|index| {
                        let index = f64::from(u32::try_from(index).unwrap_or_default());
                        let offset = match workload {
                            "first_hit" if index == 0.0 => 0.0,
                            "clear" | "first_hit" => index.mul_add(3.0, 1.5),
                            "dense_hit" => index * 3.0,
                            _ => 0.0,
                        };
                        finite(rectangle_shape(1.0, 1.0, offset))
                    })
                    .collect::<Vec<_>>();
                let left_manager = FiniteManager::from_components(&left);
                let right_manager = FiniteManager::from_components(&right);
                let iterations = if count >= 1024 { 100 } else { ITERATIONS };

                let legacy = benchmark_strategy(iterations, || {
                    finite_components_collide_legacy(
                        black_box(&left),
                        DPose2::IDENTITY,
                        black_box(&right),
                        DPose2::IDENTITY,
                    )
                });
                let cached = benchmark_strategy(iterations, || {
                    finite_components_collide(
                        black_box(&left),
                        DPose2::IDENTITY,
                        &left_manager,
                        black_box(&right),
                        DPose2::IDENTITY,
                        &right_manager,
                    )
                });

                println!("{workload},{count},legacy,{},{}", legacy.0, legacy.1);
                println!(
                    "{workload},{count},manager_cached,{},{}",
                    cached.0, cached.1
                );
                assert_eq!(legacy.1, cached.1);
            }
        }
    }

    fn benchmark_strategy<F>(iterations: usize, mut strategy: F) -> (u128, bool)
    where
        F: FnMut() -> bool,
    {
        for _ in 0..100 {
            black_box(strategy());
        }

        let start = Instant::now();
        let mut result = false;
        for _ in 0..iterations {
            result |= black_box(strategy());
        }

        let elapsed = start.elapsed().as_nanos();
        let per_query = elapsed
            .checked_div(u128::try_from(iterations).unwrap_or_default())
            .unwrap_or_default();

        (per_query, result)
    }
}
