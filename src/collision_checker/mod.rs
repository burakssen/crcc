use crate::collision_checker::engine::CollisionEngine;
use crate::collision_checker::engine::parry::ParryEngine;
use crate::dynamic_obstacle::GenericDynamicObstacle;
use crate::time::TimeStep;
pub use builder::CollisionCheckerBuilder;
use nalgebra::Isometry2;

mod builder;
mod engine;

#[derive(Debug)]
pub enum CollisionCheckerError {
    Unsupported,
}

pub struct CollisionChecker<E: CollisionEngine = ParryEngine> {
    static_obstacle: E::EngineCollisionObject,
    dynamic_obstacles: Vec<GenericDynamicObstacle<E::EngineCollisionObject>>,
}

impl<E: CollisionEngine> CollisionChecker<E> {
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
        if self.collides_with_static_at(obj, position)? {
            return Ok(true);
        }
        for obs in &self.dynamic_obstacles {
            if Self::dynamic_obstacle_collides(obs, obj, time_step, position)? {
                return Ok(true);
            }
        }
        Ok(false)
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
}
