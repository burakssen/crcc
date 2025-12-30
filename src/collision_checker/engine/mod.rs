use crate::collision_checker::CollisionCheckerError;
use crate::collision_object::CollisionObject;
use nalgebra::Isometry2;

pub mod parry;

pub trait CollisionEngine {
    type EngineCollisionObject: From<CollisionObject>;

    fn collides(
        obj_1: &Self::EngineCollisionObject,
        pos_1: &Isometry2<f64>,
        obj_2: &Self::EngineCollisionObject,
        pos_2: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError>;

    fn collides_continuous(
        obj_1: &Self::EngineCollisionObject,
        start_pos_1: &Isometry2<f64>,
        end_pos_1: &Isometry2<f64>,
        obj_2: &Self::EngineCollisionObject,
        start_pos_2: &Isometry2<f64>,
        end_pos_2: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError>;
}
