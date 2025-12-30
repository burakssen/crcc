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
    pub fn collides_dynamic_obstacle(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E::EngineCollisionObject>,
    ) -> Result<DynamicCollisionResult, CollisionCheckerError> {
        let mut active_times = TimeStepSet::from(dynamic_obstacle.active_times());
        active_times.intersect(&self.active_times);
        for time_step in active_times.iter() {
            let use_ccd = active_times.contains(time_step.succ());
            if self.collides_dynamic_dynamic(dynamic_obstacle, time_step, use_ccd)? {
                return Ok(FirstCollisionAt(time_step));
            }
        }
        Ok(NoCollision)
    }

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
            let use_ccd = active_times.contains(time_step.succ());
            if self.collides_with_dynamic(obj, time_step, position, use_ccd)? {
                return Ok(FirstCollisionAt(time_step));
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
        E::collides(&self.static_obstacle, &Isometry2::identity(), obj, position)
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

    // TODO: Refactor the next two/three methods

    fn collides_with_dynamic(
        &self,
        obj: &E::EngineCollisionObject,
        time_step: TimeStep,
        position: &Isometry2<f64>,
        use_ccd: bool,
    ) -> Result<bool, CollisionCheckerError> {
        for obs in &self.dynamic_obstacles {
            let Some((obs_shape, obs_pos)) = Self::get_dyn_obs_collider(obs, time_step, use_ccd)
            else {
                continue;
            };
            if E::collides(obs_shape, &obs_pos, obj, position)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn collides_dynamic_dynamic(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E::EngineCollisionObject>,
        time_step: TimeStep,
        use_ccd: bool,
    ) -> Result<bool, CollisionCheckerError> {
        let Some((shape_broad, pos_broad)) =
            Self::get_dyn_obs_collider(dynamic_obstacle, time_step, use_ccd)
        else {
            return Ok(false);
        };
        for obs in &self.dynamic_obstacles {
            let Some((obs_shape_broad, obs_pos_broad)) =
                Self::get_dyn_obs_collider(obs, time_step, use_ccd)
            else {
                continue;
            };
            if E::collides(obs_shape_broad, &obs_pos_broad, shape_broad, &pos_broad)? {
                if use_ccd {
                    let (shape_narrow, pos_narrow) =
                        Self::get_dyn_obs_collider(dynamic_obstacle, time_step, false)
                            .expect("Should exist since CCD collider exists.");
                    let next_pos = dynamic_obstacle
                        .position_at(time_step.succ())
                        .expect("There should be a next position since CCD collider exists.");
                    let (obs_shape_narrow, obs_pos_narrow) =
                        Self::get_dyn_obs_collider(obs, time_step, false)
                            .expect("Should exist since CCD collider exists.");
                    let next_obs_pos = obs
                        .position_at(time_step.succ())
                        .expect("There should be a next position since CCD collider exists.");
                    if E::collides_continuous(
                        obs_shape_narrow,
                        &obs_pos_narrow,
                        next_obs_pos,
                        shape_narrow,
                        &pos_narrow,
                        next_pos,
                    )? {
                        return Ok(true);
                    }
                } else {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    fn get_dyn_obs_collider(
        dynamic_obstacle: &GenericDynamicObstacle<E::EngineCollisionObject>,
        time_step: TimeStep,
        use_ccd: bool,
    ) -> Option<(&E::EngineCollisionObject, Isometry2<f64>)> {
        if use_ccd {
            dynamic_obstacle
                .convex_hull_after(time_step)
                .map(|conv_hull| (conv_hull, Isometry2::identity()))
        } else {
            dynamic_obstacle
                .position_at(time_step)
                .map(|pos| (dynamic_obstacle.shape(), *pos))
        }
    }
}
