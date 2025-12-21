use crate::collision_checker::CollisionCheckerError;
use crate::collision_object::CollisionObject;
use nalgebra::Isometry2;

pub mod parry;

pub trait CollisionEngine {
    type EngineCollisionObject: From<CollisionObject>;

    fn from_collision_objects(collision_objects: Vec<CollisionObject>) -> Self;

    fn collides_at(
        &self,
        obj: &Self::EngineCollisionObject,
        position: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError>;
}
