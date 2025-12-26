use crate::collision_checker::CollisionCheckerError;
use crate::collision_object::StaticCollisionObject;
use nalgebra::Isometry2;

pub mod parry;

pub trait CollisionEngine {
    type EngineCollisionObject: From<StaticCollisionObject>;

    fn collides_at(
        obj_1: &Self::EngineCollisionObject,
        pos_1: &Isometry2<f64>,
        obj_2: &Self::EngineCollisionObject,
        pos_2: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError>;
}
