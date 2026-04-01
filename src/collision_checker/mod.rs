use crate::collision_checker::ccd_collider::{CCDCollider, CCDColliderAt};
use crate::collision_checker::engine::EngineCollisionObject;
use crate::dynamic_obstacle::GenericDynamicObstacle;
use crate::time::{TimeStep, TimeStepSet};
use glamx::DPose2;
use std::cell::LazyCell;
use std::ops::RangeBounds;

use crate::error::CrccError;

mod builder;
mod ccd_collider;
pub mod engine;
#[cfg(feature = "rayon")]
pub mod parallel;
mod selected;

pub use builder::CollisionCheckerBuilder;
pub use selected::SelectedCollisionChecker;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionStatus {
    NoCollision,
    CollidesStatic,
    CollidesDynamic(TimeStep),
}

impl CollisionStatus {
    pub fn collides(&self) -> bool {
        match self {
            CollisionStatus::NoCollision => false,
            CollisionStatus::CollidesStatic | CollisionStatus::CollidesDynamic(_) => true,
        }
    }
}

pub type CollisionResult = Result<CollisionStatus, CrccError>;

pub struct CollisionChecker<E: EngineCollisionObject> {
    static_obstacle: E,
    dynamic_obstacles: Vec<GenericDynamicObstacle<E>>,
    active_times: TimeStepSet,
}

impl<E: EngineCollisionObject> CollisionChecker<E> {
    pub fn collides_static(&self, static_obstacle: &E) -> CollisionResult {
        self.collides_static_range(static_obstacle, DPose2::IDENTITY, ..)
    }

    pub fn collides_dynamic(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E>,
    ) -> CollisionResult {
        self.collides_dynamic_range(dynamic_obstacle, ..)
    }

    pub fn collides_static_at(&self, static_obstacle: &E, time_step: TimeStep) -> CollisionResult {
        self.collides_static_range(static_obstacle, DPose2::IDENTITY, time_step..=time_step)
    }

    pub fn collides_dynamic_at(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E>,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_dynamic_range(dynamic_obstacle, time_step..=time_step)
    }

    pub fn collides_static_pos(&self, static_obstacle: &E, position: DPose2) -> CollisionResult {
        self.collides_static_range(static_obstacle, position, ..)
    }

    pub fn collides_static_pos_at(
        &self,
        static_obstacle: &E,
        position: DPose2,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_static_range(static_obstacle, position, time_step..=time_step)
    }

    pub fn collides_static_range(
        &self,
        static_obstacle: &E,
        position: DPose2,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        if self.check_collision_static_static(static_obstacle, position)? {
            return Ok(CollisionStatus::CollidesStatic);
        }

        let ccd_collider = LazyCell::new(|| CCDCollider {
            shape: static_obstacle,
            position,
            next_position: position,
            convex_hull: static_obstacle,
            convex_hull_position: position,
        });

        let mut active_times = TimeStepSet::from(time_range);
        active_times.intersect(&self.active_times);
        for time_step in active_times.iter() {
            if active_times.contains(time_step.succ()) {
                if self.check_collision_dynamic_ccd(&ccd_collider, time_step)? {
                    return Ok(CollisionStatus::CollidesDynamic(time_step));
                }
            } else if self.check_collision_dynamic_static(static_obstacle, position, time_step)? {
                return Ok(CollisionStatus::CollidesDynamic(time_step));
            }
        }
        Ok(CollisionStatus::NoCollision)
    }

    pub fn collides_dynamic_range(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E>,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        let shape = dynamic_obstacle.shape();
        let mut active_times = TimeStepSet::from(time_range);
        active_times.intersect(&dynamic_obstacle.active_times());
        active_times.intersect(&self.active_times);
        for time_step in active_times.iter() {
            if active_times.contains(time_step.succ()) {
                let ccd_collider = dynamic_obstacle
                    .ccd_collider_at(time_step)
                    .expect("Should exist since the time step and its successor are active");
                if self.check_collision_static_static(ccd_collider.convex_hull, DPose2::IDENTITY)?
                    || self.check_collision_dynamic_ccd(&ccd_collider, time_step)?
                {
                    return Ok(CollisionStatus::CollidesDynamic(time_step));
                }
            } else {
                let position = dynamic_obstacle
                    .position_at(time_step)
                    .expect("Should exist since the time step is active");
                if self.check_collision_static_static(shape, position)?
                    || self.check_collision_dynamic_static(shape, position, time_step)?
                {
                    return Ok(CollisionStatus::CollidesDynamic(time_step));
                }
            }
        }
        Ok(CollisionStatus::NoCollision)
    }

    fn check_collision_static_static(
        &self,
        static_obstacle: &E,
        position: DPose2,
    ) -> Result<bool, CrccError> {
        E::collides_at(
            &self.static_obstacle,
            DPose2::IDENTITY,
            static_obstacle,
            position,
        )
    }

    fn check_collision_dynamic_static(
        &self,
        static_obstacle: &E,
        position: DPose2,
        time_step: TimeStep,
    ) -> Result<bool, CrccError> {
        for obs in &self.dynamic_obstacles {
            let Some((obs_shape, obs_pos)) =
                obs.position_at(time_step).map(|pos| (obs.shape(), pos))
            else {
                continue;
            };
            if E::collides_at(obs_shape, obs_pos, static_obstacle, position)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn check_collision_dynamic_ccd(
        &self,
        ccd_collider: &CCDCollider<E>,
        time_step: TimeStep,
    ) -> Result<bool, CrccError> {
        for obs in &self.dynamic_obstacles {
            let Some(obs_ccd_collider) = obs.ccd_collider_at(time_step) else {
                continue;
            };
            if E::collides_at(
                // Broad-phase check with convex hull
                obs_ccd_collider.convex_hull,
                obs_ccd_collider.convex_hull_position,
                ccd_collider.convex_hull,
                ccd_collider.convex_hull_position,
            )? && E::collides_continuous(
                // Narrow-phase check with CCD
                obs_ccd_collider.shape,
                obs_ccd_collider.position,
                obs_ccd_collider.next_position,
                ccd_collider.shape,
                ccd_collider.position,
                ccd_collider.next_position,
            )? {
                return Ok(true);
            }
        }
        Ok(false)
    }
}
