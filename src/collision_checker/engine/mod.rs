use crate::collision_checker::CollisionCheckerError;
use crate::collision_object::CollisionObject;
use glamx::DPose2;

pub mod parry;

pub trait CollisionEngine {
    type EngineCollisionObject: From<CollisionObject>;

    fn collides(
        obj_1: &Self::EngineCollisionObject,
        pos_1: DPose2,
        obj_2: &Self::EngineCollisionObject,
        pos_2: DPose2,
    ) -> Result<bool, CollisionCheckerError>;

    fn collides_continuous(
        obj_1: &Self::EngineCollisionObject,
        start_pos_1: DPose2,
        end_pos_1: DPose2,
        obj_2: &Self::EngineCollisionObject,
        start_pos_2: DPose2,
        end_pos_2: DPose2,
    ) -> Result<bool, CollisionCheckerError>;
}
