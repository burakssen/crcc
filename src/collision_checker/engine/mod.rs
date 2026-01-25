use crate::collision_checker::CollisionCheckerError;
use crate::collision_object::CollisionObject;
use glamx::DPose2;

#[cfg(feature = "default-engine")]
pub(crate) mod default;
#[cfg(feature = "parry")]
pub mod parry;

pub trait EngineCollisionObject: From<CollisionObject> {
    fn collides(&self, other: &Self) -> Result<bool, CollisionCheckerError> {
        self.collides_at(DPose2::IDENTITY, other, DPose2::IDENTITY)
    }

    fn collides_at(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<bool, CollisionCheckerError>;

    fn collides_continuous(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> Result<bool, CollisionCheckerError>;
}
