use crate::collision_checker::engine::CollisionEngine;
use crate::collision_checker::engine::parry::ParryEngine;
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
}

impl<E: CollisionEngine> CollisionChecker<E> {
    pub fn collides(&self, obj: &E::EngineCollisionObject) -> Result<bool, CollisionCheckerError> {
        self.collides_at(obj, &Isometry2::identity())
    }

    pub fn collides_at(
        &self,
        obj: &E::EngineCollisionObject,
        position: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError> {
        E::collides_at(&self.static_obstacle, &Isometry2::identity(), obj, position)
    }

    /// Check collision with any convertible object at the given position.
    ///
    /// This method takes ownership of the object, because it should only be used
    /// if we need to check collision only once.
    /// Otherwise, it is better to convert the object once and reuse it with `collides_at`.
    pub fn obj_collides_at(
        &self,
        obj: impl Into<E::EngineCollisionObject>,
        position: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError> {
        self.collides_at(&obj.into(), position)
    }
}
