pub use crate::collision_checker::builder::CollisionCheckerBuilder;
use crate::collision_checker::engine::CollisionEngine;
pub use crate::collision_checker::engine::parry::ParryCollisionObject;
use crate::collision_checker::engine::parry::ParryEngine;
use crate::dynamic_obstacle::{CCDCollider, GenericDynamicObstacle};
use crate::time::{TimeStep, TimeStepSet};
use glamx::DPose2;
use std::cell::LazyCell;
use std::ops::{Bound, RangeBounds};

mod builder;
mod engine;

#[derive(Debug, PartialEq, Eq)]
pub enum DynamicCollisionResult {
    NoCollision,
    FirstCollisionAt(TimeStep),
}

#[derive(Debug)]
pub enum CollisionCheckerError {
    Unsupported,
}

pub struct CollisionChecker<E: CollisionEngine = ParryEngine> {
    static_obstacle: E::EngineCollisionObject,
    dynamic_obstacles: Vec<GenericDynamicObstacle<E::EngineCollisionObject>>,
    active_times: TimeStepSet,
}

impl<E: CollisionEngine> CollisionChecker<E> {
    pub fn collides_static(
        &self,
        static_obstacle: &E::EngineCollisionObject,
    ) -> Result<DynamicCollisionResult, CollisionCheckerError> {
        self.collides_static_range(static_obstacle, &DPose2::identity(), ..)
    }

    pub fn collides_dynamic(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E::EngineCollisionObject>,
    ) -> Result<DynamicCollisionResult, CollisionCheckerError> {
        self.collides_dynamic_range(dynamic_obstacle, ..)
    }

    pub fn collides_static_at(
        &self,
        static_obstacle: &E::EngineCollisionObject,
        time_step: TimeStep,
    ) -> Result<bool, CollisionCheckerError> {
        self.collides_static_range(static_obstacle, &DPose2::identity(), time_step..=time_step)
            .map(|res| matches!(res, DynamicCollisionResult::FirstCollisionAt(_)))
    }

    pub fn collides_dynamic_at(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E::EngineCollisionObject>,
        time_step: TimeStep,
    ) -> Result<bool, CollisionCheckerError> {
        self.collides_dynamic_range(dynamic_obstacle, time_step..=time_step)
            .map(|res| matches!(res, DynamicCollisionResult::FirstCollisionAt(_)))
    }

    pub fn collides_static_pos(
        &self,
        static_obstacle: &E::EngineCollisionObject,
        position: &DPose2,
    ) -> Result<DynamicCollisionResult, CollisionCheckerError> {
        self.collides_static_range(static_obstacle, position, ..)
    }

    pub fn collides_static_pos_at(
        &self,
        static_obstacle: &E::EngineCollisionObject,
        position: &DPose2,
        time_step: TimeStep,
    ) -> Result<bool, CollisionCheckerError> {
        self.collides_static_range(static_obstacle, position, time_step..=time_step)
            .map(|res| matches!(res, DynamicCollisionResult::FirstCollisionAt(_)))
    }

    pub fn collides_static_range(
        &self,
        static_obstacle: &E::EngineCollisionObject,
        position: &DPose2,
        time_range: impl RangeBounds<TimeStep>,
    ) -> Result<DynamicCollisionResult, CollisionCheckerError> {
        if self.check_collision_static_static(static_obstacle, position)? {
            return Ok(DynamicCollisionResult::FirstCollisionAt(
                match time_range.start_bound() {
                    Bound::Included(t) => *t,
                    Bound::Excluded(t) => t.succ(),
                    Bound::Unbounded => TimeStep::MIN,
                },
            ));
        }

        let ccd_collider = LazyCell::new(|| CCDCollider {
            shape: static_obstacle,
            position,
            next_position: position,
            convex_hull: static_obstacle,
        });

        let mut active_times = TimeStepSet::from(time_range);
        active_times.intersect(&self.active_times);
        for time_step in active_times.iter() {
            if active_times.contains(time_step.succ()) {
                if self.check_collision_dynamic_ccd(&ccd_collider, time_step)? {
                    return Ok(DynamicCollisionResult::FirstCollisionAt(time_step));
                }
            } else if self.check_collision_dynamic_static(static_obstacle, position, time_step)? {
                return Ok(DynamicCollisionResult::FirstCollisionAt(time_step));
            }
        }
        Ok(DynamicCollisionResult::NoCollision)
    }

    pub fn collides_dynamic_range(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E::EngineCollisionObject>,
        time_range: impl RangeBounds<TimeStep>,
    ) -> Result<DynamicCollisionResult, CollisionCheckerError> {
        let shape = dynamic_obstacle.shape();
        let mut active_times = TimeStepSet::from(time_range);
        active_times.intersect(&dynamic_obstacle.active_times().into());
        active_times.intersect(&self.active_times);
        for time_step in active_times.iter() {
            if active_times.contains(time_step.succ()) {
                let ccd_collider = dynamic_obstacle
                    .ccd_collider_at(time_step)
                    .expect("Should exist since the time step and its successor are active");
                if self
                    .check_collision_static_static(ccd_collider.convex_hull, &DPose2::identity())?
                    || self.check_collision_dynamic_ccd(&ccd_collider, time_step)?
                {
                    return Ok(DynamicCollisionResult::FirstCollisionAt(time_step));
                }
            } else {
                let position = dynamic_obstacle
                    .position_at(time_step)
                    .expect("Should exist since the time step is active");
                if self.check_collision_static_static(shape, position)?
                    || self.check_collision_dynamic_static(shape, position, time_step)?
                {
                    return Ok(DynamicCollisionResult::FirstCollisionAt(time_step));
                }
            }
        }
        Ok(DynamicCollisionResult::NoCollision)
    }

    fn check_collision_static_static(
        &self,
        static_obstacle: &E::EngineCollisionObject,
        position: &DPose2,
    ) -> Result<bool, CollisionCheckerError> {
        E::collides(
            &self.static_obstacle,
            &DPose2::identity(),
            static_obstacle,
            position,
        )
    }

    fn check_collision_dynamic_static(
        &self,
        static_obstacle: &E::EngineCollisionObject,
        position: &DPose2,
        time_step: TimeStep,
    ) -> Result<bool, CollisionCheckerError> {
        for obs in &self.dynamic_obstacles {
            let Some((obs_shape, obs_pos)) =
                obs.position_at(time_step).map(|pos| (obs.shape(), pos))
            else {
                continue;
            };
            if E::collides(obs_shape, obs_pos, static_obstacle, position)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn check_collision_dynamic_ccd(
        &self,
        ccd_collider: &CCDCollider<E::EngineCollisionObject>,
        time_step: TimeStep,
    ) -> Result<bool, CollisionCheckerError> {
        let identity = DPose2::identity();
        for obs in &self.dynamic_obstacles {
            let Some(obs_ccd_collider) = obs.ccd_collider_at(time_step) else {
                continue;
            };
            if E::collides(
                // Broad-phase check with convex hull
                obs_ccd_collider.convex_hull,
                &identity,
                ccd_collider.convex_hull,
                &identity,
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
