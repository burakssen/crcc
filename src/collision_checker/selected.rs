#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
use crate::collision_checker::CollisionChecker;
use crate::collision_checker::CollisionResult;
use crate::collision_checker::engine::CollisionEngine;
#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_object::CollisionObject;
use crate::dynamic_obstacle::DynamicObstacle;
#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
use crate::dynamic_obstacle::GenericDynamicObstacle;
use crate::time::TimeStep;
use glamx::DPose2;
use std::ops::RangeBounds;

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
const PARALLEL_QUERY_THRESHOLD: usize = 32;

pub(crate) enum SelectedCollisionCheckerInner {
    #[cfg(feature = "parry")]
    Parry(
        Box<
            crate::collision_checker::CollisionChecker<
                crate::collision_checker::engine::parry::ParryCollisionObject,
            >,
        >,
    ),
    #[cfg(feature = "rhusics")]
    Rhusics(
        Box<
            crate::collision_checker::CollisionChecker<
                crate::collision_checker::engine::rhusics::RhusicsCoreCollisionObject,
            >,
        >,
    ),
    #[cfg(feature = "collide")]
    Collide(
        Box<
            crate::collision_checker::CollisionChecker<
                crate::collision_checker::engine::collide::CollideCollisionObject,
            >,
        >,
    ),
}

/// An immutable collision scene using one runtime-selected backend.
pub struct SelectedCollisionChecker(SelectedCollisionCheckerInner);

impl SelectedCollisionChecker {
    pub(crate) fn new(inner: SelectedCollisionCheckerInner) -> Self {
        Self(inner)
    }

    /// Checks a static obstacle against the scene geometry across all active times.
    pub fn collides_static(&self, static_obstacle: &CollisionObject) -> CollisionResult {
        self.collides_static_range(static_obstacle, DPose2::IDENTITY, ..)
    }

    /// Checks a dynamic obstacle against the scene geometry across all active times.
    pub fn collides_dynamic(&self, dynamic_obstacle: &DynamicObstacle) -> CollisionResult {
        self.collides_dynamic_range(dynamic_obstacle, ..)
    }

    /// Checks a static obstacle against the scene geometry at a specific time step.
    pub fn collides_static_at(
        &self,
        static_obstacle: &CollisionObject,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_static_range(static_obstacle, DPose2::IDENTITY, time_step..=time_step)
    }

    /// Checks a dynamic obstacle against the scene geometry at a specific time step.
    pub fn collides_dynamic_at(
        &self,
        dynamic_obstacle: &DynamicObstacle,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_dynamic_range(dynamic_obstacle, time_step..=time_step)
    }

    /// Checks a positioned static obstacle against the scene geometry across all active times.
    pub fn collides_static_pos(
        &self,
        static_obstacle: &CollisionObject,
        position: DPose2,
    ) -> CollisionResult {
        self.collides_static_range(static_obstacle, position, ..)
    }

    /// Checks a positioned static obstacle against the scene geometry at a specific time step.
    pub fn collides_static_pos_at(
        &self,
        static_obstacle: &CollisionObject,
        position: DPose2,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_static_range(static_obstacle, position, time_step..=time_step)
    }

    /// Checks a positioned fixed shape against static and dynamic scene geometry.
    ///
    /// `time_range` limits dynamic-obstacle checks. Static geometry is always
    /// checked. The first dynamic collision time is returned.
    pub fn collides_static_range(
        &self,
        static_obstacle: &CollisionObject,
        position: DPose2,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
        let _ = (static_obstacle, position, &time_range);

        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(checker) => {
                collides_static(checker, static_obstacle, position, time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(checker) => {
                collides_static(checker, static_obstacle, position, time_range)
            }
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(checker) => {
                collides_static(checker, static_obstacle, position, time_range)
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => Err(crate::error::CrccError::Unsupported),
        }
    }

    /// Checks a moving obstacle against static and dynamic scene geometry.
    ///
    /// Continuous motion between adjacent active trajectory steps is included.
    pub fn collides_dynamic_range(
        &self,
        dynamic_obstacle: &DynamicObstacle,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
        let _ = (dynamic_obstacle, &time_range);

        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(checker) => {
                collides_dynamic(checker, dynamic_obstacle, time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(checker) => {
                collides_dynamic(checker, dynamic_obstacle, time_range)
            }
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(checker) => {
                collides_dynamic(checker, dynamic_obstacle, time_range)
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => Err(crate::error::CrccError::Unsupported),
        }
    }

    /// Returns the backend selected when this checker was built.
    pub fn engine(&self) -> CollisionEngine {
        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(_) => CollisionEngine::Parry,
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(_) => CollisionEngine::Rhusics,
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(_) => CollisionEngine::Collide,
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => CollisionEngine::default(),
        }
    }

    #[cfg(feature = "rayon")]
    /// Checks fixed-shape queries in a batch, preserving input order.
    ///
    /// Small batches run sequentially; larger batches use Rayon's active pool.
    pub fn collides_static_batch(
        &self,
        positioned_static_obstacles: &[(CollisionObject, DPose2)],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult> {
        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(checker) => {
                collides_static_batch(checker, positioned_static_obstacles, time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(checker) => {
                collides_static_batch(checker, positioned_static_obstacles, time_range)
            }
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(checker) => {
                collides_static_batch(checker, positioned_static_obstacles, time_range)
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => positioned_static_obstacles
                .iter()
                .map(|_| Err(crate::error::CrccError::Unsupported))
                .collect(),
        }
    }

    #[cfg(feature = "rayon")]
    /// Checks dynamic queries in a batch, preserving input order.
    ///
    /// Small batches run sequentially; larger batches use Rayon's active pool.
    pub fn collides_dynamic_batch(
        &self,
        dynamic_obstacles: &[DynamicObstacle],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult> {
        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(checker) => {
                collides_dynamic_batch(checker, dynamic_obstacles, time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(checker) => {
                collides_dynamic_batch(checker, dynamic_obstacles, time_range)
            }
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(checker) => {
                collides_dynamic_batch(checker, dynamic_obstacles, time_range)
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => dynamic_obstacles
                .iter()
                .map(|_| Err(crate::error::CrccError::Unsupported))
                .collect(),
        }
    }

    #[cfg(feature = "rayon")]
    /// Checks multiple positioned static obstacles in parallel using Rayon.
    pub fn par_static(
        &self,
        positioned_static_obstacles: &[(CollisionObject, DPose2)],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult> {
        self.collides_static_batch(positioned_static_obstacles, time_range)
    }

    #[cfg(feature = "rayon")]
    /// Checks multiple dynamic obstacles in parallel using Rayon.
    pub fn par_dynamic(
        &self,
        dynamic_obstacles: &[DynamicObstacle],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult> {
        self.collides_dynamic_batch(dynamic_obstacles, time_range)
    }
}

#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
fn collides_static<E: EngineCollisionObject>(
    checker: &CollisionChecker<E>,
    static_obstacle: &CollisionObject,
    position: DPose2,
    time_range: impl RangeBounds<TimeStep>,
) -> CollisionResult {
    let static_obstacle = E::from(static_obstacle.clone());
    checker.collides_static_range(&static_obstacle, position, time_range)
}

#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
fn collides_dynamic<E: EngineCollisionObject>(
    checker: &CollisionChecker<E>,
    dynamic_obstacle: &DynamicObstacle,
    time_range: impl RangeBounds<TimeStep>,
) -> CollisionResult {
    let dynamic_obstacle: GenericDynamicObstacle<E> = dynamic_obstacle.clone().convert_repr();
    checker.collides_dynamic_range(&dynamic_obstacle, time_range)
}

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
fn collides_static_batch<E: EngineCollisionObject + Send + Sync>(
    checker: &CollisionChecker<E>,
    positioned_static_obstacles: &[(CollisionObject, DPose2)],
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
) -> Vec<CollisionResult> {
    use crate::collision_checker::parallel::ParallelCollisionChecker;
    use rayon::prelude::*;

    let converted = positioned_static_obstacles
        .iter()
        .map(|(obstacle, position)| (E::from(obstacle.clone()), *position))
        .collect::<Vec<_>>();

    if converted.len() < PARALLEL_QUERY_THRESHOLD {
        return converted
            .iter()
            .map(|(obstacle, position)| {
                checker.collides_static_range(obstacle, *position, time_range.clone())
            })
            .collect();
    }

    checker.collides_static_batch(
        converted
            .par_iter()
            .map(|(obstacle, position)| (obstacle, *position)),
        time_range,
    )
}

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
fn collides_dynamic_batch<E: EngineCollisionObject + Send + Sync>(
    checker: &CollisionChecker<E>,
    dynamic_obstacles: &[DynamicObstacle],
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
) -> Vec<CollisionResult> {
    use crate::collision_checker::parallel::ParallelCollisionChecker;
    use rayon::prelude::*;

    let converted = dynamic_obstacles
        .iter()
        .cloned()
        .map(DynamicObstacle::convert_repr)
        .collect::<Vec<GenericDynamicObstacle<E>>>();

    if converted.len() < PARALLEL_QUERY_THRESHOLD {
        return converted
            .iter()
            .map(|obstacle| checker.collides_dynamic_range(obstacle, time_range.clone()))
            .collect();
    }

    checker.collides_dynamic_batch(converted.par_iter(), time_range)
}

#[cfg(all(
    test,
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
mod tests {
    use super::*;
    use crate::collision_checker::CollisionCheckerBuilder;
    use crate::collision_object::simple::SimpleCollisionObject;

    fn engines() -> Vec<CollisionEngine> {
        vec![
            #[cfg(feature = "parry")]
            CollisionEngine::Parry,
            #[cfg(feature = "rhusics")]
            CollisionEngine::Rhusics,
            #[cfg(feature = "collide")]
            CollisionEngine::Collide,
        ]
    }

    fn assert_send_sync<T: Send + Sync>() {}

    #[test]
    fn backend_objects_are_send_and_sync() {
        #[cfg(feature = "parry")]
        assert_send_sync::<crate::collision_checker::engine::parry::ParryCollisionObject>();
        #[cfg(feature = "rhusics")]
        assert_send_sync::<crate::collision_checker::engine::rhusics::RhusicsCoreCollisionObject>();
        #[cfg(feature = "collide")]
        assert_send_sync::<crate::collision_checker::engine::collide::CollideCollisionObject>();
    }

    #[test]
    fn parallel_static_matches_sequential_around_threshold() {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(2)
            .build()
            .unwrap();
        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();

            for count in [
                PARALLEL_QUERY_THRESHOLD - 1,
                PARALLEL_QUERY_THRESHOLD,
                PARALLEL_QUERY_THRESHOLD + 1,
            ] {
                let queries = (0..count)
                    .map(|index| {
                        let x = if index % 2 == 0 { 0.5 } else { 10.0 };
                        (
                            CollisionObject::circle((0.0, 0.0), 0.25).unwrap(),
                            DPose2::translation(x, 0.0),
                        )
                    })
                    .collect::<Vec<_>>();
                let sequential = queries
                    .iter()
                    .map(|(query, position)| checker.collides_static_range(query, *position, ..))
                    .collect::<Vec<_>>();

                assert_eq!(
                    pool.install(|| checker.collides_static_batch(&queries, ..)),
                    sequential,
                    "{engine:?}, {count}"
                );
            }
        }
    }

    #[test]
    fn parallel_dynamic_matches_sequential_around_threshold() {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(2)
            .build()
            .unwrap();
        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();

            for count in [
                PARALLEL_QUERY_THRESHOLD - 1,
                PARALLEL_QUERY_THRESHOLD,
                PARALLEL_QUERY_THRESHOLD + 1,
            ] {
                let queries = (0..count)
                    .map(|index| {
                        let x = if index % 2 == 0 { 4.0 } else { 10.0 };
                        DynamicObstacle::new(
                            CollisionObject::circle((0.0, 0.0), 0.25).unwrap(),
                            vec![DPose2::translation(x, 0.0), DPose2::translation(0.5, 0.0)],
                            TimeStep(5),
                        )
                    })
                    .collect::<Vec<_>>();
                let sequential = queries
                    .iter()
                    .map(|query| checker.collides_dynamic_range(query, TimeStep(5)..=TimeStep(6)))
                    .collect::<Vec<_>>();

                assert_eq!(
                    pool.install(
                        || checker.collides_dynamic_batch(&queries, TimeStep(5)..=TimeStep(6))
                    ),
                    sequential,
                    "{engine:?}, {count}"
                );
            }
        }
    }
}
