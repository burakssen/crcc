use crate::collision_checker::engine::CollisionEngine;
use crate::collision_checker::{CollisionChecker, CollisionResult};
use crate::collision_object::CollisionObject;
use crate::dynamic_obstacle::DynamicObstacle;
use crate::time::TimeStep;
use glamx::DPose2;
use std::ops::RangeBounds;

pub enum SelectedCollisionChecker {
    #[cfg(feature = "parry")]
    Parry(CollisionChecker<crate::collision_checker::engine::parry::ParryCollisionObject>),
    #[cfg(feature = "rhusics")]
    Rhusics(
        CollisionChecker<crate::collision_checker::engine::rhusics::RhusicsCoreCollisionObject>,
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
        match self {
            #[cfg(feature = "parry")]
            SelectedCollisionChecker::Parry(checker) => {
                let static_obstacle = static_obstacle.clone().into();
                checker.collides_static_range(&static_obstacle, position, time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionChecker::Rhusics(checker) => {
                let static_obstacle = static_obstacle.clone().into();
                checker.collides_static_range(&static_obstacle, position, time_range)
            }
        }
    }

    pub fn collides_dynamic_range(
        &self,
        dynamic_obstacle: &DynamicObstacle,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        match self {
            #[cfg(feature = "parry")]
            SelectedCollisionChecker::Parry(checker) => {
                let dynamic_obstacle = dynamic_obstacle.clone().convert_repr();
                checker.collides_dynamic_range(&dynamic_obstacle, time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionChecker::Rhusics(checker) => {
                let dynamic_obstacle = dynamic_obstacle.clone().convert_repr();
                checker.collides_dynamic_range(&dynamic_obstacle, time_range)
            }
        }
    }

    pub fn engine(&self) -> CollisionEngine {
        match self {
            #[cfg(feature = "parry")]
            SelectedCollisionChecker::Parry(_) => CollisionEngine::Parry,
            #[cfg(feature = "rhusics")]
            SelectedCollisionChecker::Rhusics(_) => CollisionEngine::Rhusics,
        }
    }

    #[cfg(feature = "rayon")]
    pub fn par_collides_static(
        &self,
        positioned_static_obstacles: &[(CollisionObject, DPose2)],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult> {
        use crate::collision_checker::parallel::ParallelCollisionChecker;
        use rayon::prelude::*;

        match self {
            #[cfg(feature = "parry")]
            SelectedCollisionChecker::Parry(checker) => {
                let converted = positioned_static_obstacles
                    .iter()
                    .map(|(obs, pos)| (obs.clone().into(), *pos))
                    .collect::<Vec<_>>();
                checker.par_collides_static(
                    converted.par_iter().map(|(obs, pos)| (obs, *pos)),
                    time_range,
                )
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionChecker::Rhusics(checker) => {
                let converted = positioned_static_obstacles
                    .iter()
                    .map(|(obs, pos)| (obs.clone().into(), *pos))
                    .collect::<Vec<_>>();
                checker.par_collides_static(
                    converted.par_iter().map(|(obs, pos)| (obs, *pos)),
                    time_range,
                )
            }
        }
    }

    #[cfg(feature = "rayon")]
    pub fn par_collides_dynamic(
        &self,
        dynamic_obstacles: &[DynamicObstacle],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult> {
        use crate::collision_checker::parallel::ParallelCollisionChecker;
        use rayon::prelude::*;

        match self {
            #[cfg(feature = "parry")]
            SelectedCollisionChecker::Parry(checker) => {
                let converted = dynamic_obstacles
                    .iter()
                    .cloned()
                    .map(DynamicObstacle::convert_repr)
                    .collect::<Vec<_>>();
                checker.par_collides_dynamic(converted.par_iter(), time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionChecker::Rhusics(checker) => {
                let converted = dynamic_obstacles
                    .iter()
                    .cloned()
                    .map(DynamicObstacle::convert_repr)
                    .collect::<Vec<_>>();
                checker.par_collides_dynamic(converted.par_iter(), time_range)
            }
        }
    }
}
