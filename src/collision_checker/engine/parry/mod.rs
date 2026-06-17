use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_checker::engine::parry::inner::ParryCollisionObjectInner;
use crate::collision_object::CollisionObject;
use crate::error::CrccError;
use glamx::DPose2;
use parry2d_f64::query::Unsupported;

mod inner;
mod simple;

#[derive(Debug, Clone)]
pub struct ParryCollisionObject(ParryCollisionObjectInner);

impl EngineCollisionObject for ParryCollisionObject {
    fn collides_at(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<bool, CrccError> {
        Ok(self
            .as_ref()
            .collides(pos_self, other.as_ref(), pos_other)?)
    }

    fn collides_sweep(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> Result<bool, CrccError> {
        Ok(self.as_ref().collides_sweep(
            start_pos_self,
            end_pos_self,
            other.as_ref(),
            start_pos_other,
            end_pos_other,
        )?)
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
    fn from(error: Unsupported) -> Self {
        match error {
            // Deliberate match with one arm to future-proof against new error variants in parry
            Unsupported => CrccError::Unsupported,
        }
    }
}
