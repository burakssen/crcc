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

pub enum SelectedCollisionChecker {
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

impl SelectedCollisionChecker {
    pub fn collides_static(&self, static_obstacle: &CollisionObject) -> CollisionResult {
        self.collides_static_range(static_obstacle, DPose2::IDENTITY, ..)
    }

    pub fn collides_dynamic(&self, dynamic_obstacle: &DynamicObstacle) -> CollisionResult {
        self.collides_dynamic_range(dynamic_obstacle, ..)
    }

    pub fn collides_static_at(
        &self,
        static_obstacle: &CollisionObject,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_static_range(static_obstacle, DPose2::IDENTITY, time_step..=time_step)
    }

    pub fn collides_dynamic_at(
        &self,
        dynamic_obstacle: &DynamicObstacle,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_dynamic_range(dynamic_obstacle, time_step..=time_step)
    }

    pub fn collides_static_pos(
        &self,
        static_obstacle: &CollisionObject,
        position: DPose2,
    ) -> CollisionResult {
        self.collides_static_range(static_obstacle, position, ..)
    }

    pub fn collides_static_pos_at(
        &self,
        static_obstacle: &CollisionObject,
        position: DPose2,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_static_range(static_obstacle, position, time_step..=time_step)
    }

    pub fn collides_static_range(
        &self,
        static_obstacle: &CollisionObject,
        position: DPose2,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
        let _ = (static_obstacle, position, &time_range);

        match self {
            #[cfg(feature = "parry")]
            SelectedCollisionChecker::Parry(checker) => {
                collides_static_range(checker, static_obstacle, position, time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionChecker::Rhusics(checker) => {
                collides_static_range(checker, static_obstacle, position, time_range)
            }
            #[cfg(feature = "collide")]
            SelectedCollisionChecker::Collide(checker) => {
                collides_static_range(checker, static_obstacle, position, time_range)
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => Err(crate::error::CrccError::Unsupported),
        }
    }

    pub fn collides_dynamic_range(
        &self,
        dynamic_obstacle: &DynamicObstacle,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
        let _ = (dynamic_obstacle, &time_range);

        match self {
            #[cfg(feature = "parry")]
            SelectedCollisionChecker::Parry(checker) => {
                collides_dynamic_range(checker, dynamic_obstacle, time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionChecker::Rhusics(checker) => {
                collides_dynamic_range(checker, dynamic_obstacle, time_range)
            }
            #[cfg(feature = "collide")]
            SelectedCollisionChecker::Collide(checker) => {
                collides_dynamic_range(checker, dynamic_obstacle, time_range)
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => Err(crate::error::CrccError::Unsupported),
        }
    }

    pub fn engine(&self) -> CollisionEngine {
        match self {
            #[cfg(feature = "parry")]
            SelectedCollisionChecker::Parry(_) => CollisionEngine::Parry,
            #[cfg(feature = "rhusics")]
            SelectedCollisionChecker::Rhusics(_) => CollisionEngine::Rhusics,
            #[cfg(feature = "collide")]
            SelectedCollisionChecker::Collide(_) => CollisionEngine::Collide,
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => CollisionEngine::default(),
        }
    }

    #[cfg(feature = "rayon")]
    pub fn par_collides_static(
        &self,
        positioned_static_obstacles: &[(CollisionObject, DPose2)],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult> {
        match self {
            #[cfg(feature = "parry")]
            SelectedCollisionChecker::Parry(checker) => {
                par_collides_static(checker, positioned_static_obstacles, time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionChecker::Rhusics(checker) => {
                par_collides_static(checker, positioned_static_obstacles, time_range)
            }
            #[cfg(feature = "collide")]
            SelectedCollisionChecker::Collide(checker) => {
                par_collides_static(checker, positioned_static_obstacles, time_range)
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => positioned_static_obstacles
                .iter()
                .map(|_| Err(crate::error::CrccError::Unsupported))
                .collect(),
        }
    }

    #[cfg(feature = "rayon")]
    pub fn par_collides_dynamic(
        &self,
        dynamic_obstacles: &[DynamicObstacle],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult> {
        match self {
            #[cfg(feature = "parry")]
            SelectedCollisionChecker::Parry(checker) => {
                par_collides_dynamic(checker, dynamic_obstacles, time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionChecker::Rhusics(checker) => {
                par_collides_dynamic(checker, dynamic_obstacles, time_range)
            }
            #[cfg(feature = "collide")]
            SelectedCollisionChecker::Collide(checker) => {
                par_collides_dynamic(checker, dynamic_obstacles, time_range)
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => dynamic_obstacles
                .iter()
                .map(|_| Err(crate::error::CrccError::Unsupported))
                .collect(),
        }
    }
}

#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
fn collides_static_range<E: EngineCollisionObject>(
    checker: &CollisionChecker<E>,
    static_obstacle: &CollisionObject,
    position: DPose2,
    time_range: impl RangeBounds<TimeStep>,
) -> CollisionResult {
    let static_obstacle = E::from(static_obstacle.clone());
    checker.collides_static_range(&static_obstacle, position, time_range)
}

#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
fn collides_dynamic_range<E: EngineCollisionObject>(
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
fn par_collides_static<E: EngineCollisionObject + Send + Sync>(
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
    checker.par_collides_static(
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
fn par_collides_dynamic<E: EngineCollisionObject + Send + Sync>(
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
    checker.par_collides_dynamic(converted.par_iter(), time_range)
}
