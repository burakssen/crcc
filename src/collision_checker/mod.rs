use crate::collision_checker::DynamicCollisionResult::{FirstCollisionAt, NoCollision};
use crate::collision_checker::engine::CollisionEngine;
use crate::collision_checker::engine::parry::ParryEngine;
use crate::dynamic_obstacle::GenericDynamicObstacle;
use crate::time::{TimeStep, TimeStepSet};
pub use builder::CollisionCheckerBuilder;
use nalgebra::Isometry2;
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
    pub fn collides_at_range(
        &self,
        obj: &E::EngineCollisionObject,
        time_range: impl RangeBounds<TimeStep>,
        position: &Isometry2<f64>,
    ) -> Result<DynamicCollisionResult, CollisionCheckerError> {
        if self.collides_with_static_at(obj, position)? {
            return Ok(FirstCollisionAt(match time_range.start_bound() {
                Bound::Included(t) => *t,
                Bound::Excluded(t) => t.succ(),
                Bound::Unbounded => TimeStep::MIN,
            }));
        }

        let mut active_times = TimeStepSet::from(time_range);
        active_times.intersect(&self.active_times);
        for time_step in active_times.iter() {
            let dyn_obs_collides = if active_times.contains(time_step.succ()) {
                Self::dynamic_obstacle_collides_ccd
            } else {
                Self::dynamic_obstacle_collides
            };
            for obs in &self.dynamic_obstacles {
                if dyn_obs_collides(obs, obj, time_step, position)? {
                    return Ok(FirstCollisionAt(time_step));
                }
            }
        }
        Ok(NoCollision)
    }

    pub fn collides(
        &self,
        obj: &E::EngineCollisionObject,
        time_step: TimeStep,
    ) -> Result<bool, CollisionCheckerError> {
        self.collides_at(obj, time_step, &Isometry2::identity())
    }

    pub fn collides_at(
        &self,
        obj: &E::EngineCollisionObject,
        time_step: TimeStep,
        position: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError> {
        self.collides_at_range(obj, time_step..=time_step, position)
            .map(|res| matches!(res, FirstCollisionAt(_)))
    }

    /// Check collision with the given object at the given position and time step.
    ///
    /// This method takes ownership of the object, because it should only be used
    /// if we need to check collision only once.
    /// Otherwise, it is better to convert the object once and reuse it with `collides_at`.
    pub fn obj_collides_at(
        &self,
        obj: impl Into<E::EngineCollisionObject>,
        time_step: TimeStep,
        position: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError> {
        self.collides_at(&obj.into(), time_step, position)
    }

    pub fn collides_with_static(
        &self,
        obj: &E::EngineCollisionObject,
    ) -> Result<bool, CollisionCheckerError> {
        self.collides_with_static_at(obj, &Isometry2::identity())
    }

    pub fn collides_with_static_at(
        &self,
        obj: &E::EngineCollisionObject,
        position: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError> {
        E::collides_at(&self.static_obstacle, &Isometry2::identity(), obj, position)
    }

    /// Check collision of the static obstacles with any convertible object at the given position.
    ///
    /// This method takes ownership of the object, because it should only be used
    /// if we need to check collision only once.
    /// Otherwise, it is better to convert the object once and reuse it with `collides_with_static_at`.
    pub fn obj_collides_with_static_at(
        &self,
        obj: impl Into<E::EngineCollisionObject>,
        position: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError> {
        self.collides_with_static_at(&obj.into(), position)
    }

    fn dynamic_obstacle_collides(
        dynamic_obstacle: &GenericDynamicObstacle<E::EngineCollisionObject>,
        obj: &E::EngineCollisionObject,
        time_step: TimeStep,
        position: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError> {
        match dynamic_obstacle.position_at(time_step) {
            Some(obs_pos) => E::collides_at(dynamic_obstacle.shape(), obs_pos, obj, position),
            None => Ok(false),
        }
    }

    fn dynamic_obstacle_collides_ccd(
        dynamic_obstacle: &GenericDynamicObstacle<E::EngineCollisionObject>,
        obj: &E::EngineCollisionObject,
        time_step: TimeStep,
        position: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError> {
        match dynamic_obstacle.convex_hull_after(time_step) {
            Some(conv_hull) => E::collides_at(conv_hull, &Isometry2::identity(), obj, position),
            None => Ok(false),
        }
    }
}
