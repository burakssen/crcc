use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_checker::engine::parry::inner::ParryCollisionObjectInner;
use crate::collision_object::CollisionObject;
use crate::error::{CrccError, CrccResult};
use glamx::DPose2;
use parry2d_f64::query::Unsupported;

mod inner;
mod simple;

#[derive(Debug, Clone)]
pub struct ParryCollisionObject(ParryCollisionObjectInner);

impl EngineCollisionObject for ParryCollisionObject {
    fn collides_at(&self, pos_self: DPose2, other: &Self, pos_other: DPose2) -> CrccResult<bool> {
        self.as_ref()
            .collides(pos_self, other.as_ref(), pos_other)
            .map_err(CrccError::from)
    }

    fn collides_continuous(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> CrccResult<bool> {
        self.as_ref()
            .collides_continuous(
                start_pos_self,
                end_pos_self,
                other.as_ref(),
                start_pos_other,
                end_pos_other,
            )
            .map_err(CrccError::from)
    }

    fn distance_at(&self, pos_self: DPose2, other: &Self, pos_other: DPose2) -> CrccResult<f64> {
        self.as_ref()
            .distance(pos_self, other.as_ref(), pos_other)
            .map_err(CrccError::from)
    }
}

impl From<CollisionObject> for ParryCollisionObject {
    fn from(value: CollisionObject) -> Self {
        Self(value.into())
    }
}

impl AsRef<ParryCollisionObjectInner> for ParryCollisionObject {
    fn as_ref(&self) -> &ParryCollisionObjectInner {
        &self.0
    }
}

impl From<Unsupported> for CrccError {
    fn from(_: Unsupported) -> Self {
        Self::Unsupported
    }
}
