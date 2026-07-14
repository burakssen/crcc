use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_checker::engine::rhusics::inner::{
    RhusicsCoreCollisionObjectInner, Unsupported,
};
use crate::collision_object::CollisionObject;
use crate::error::CrccError;
use glamx::DPose2;
//use parry2d_f64::query::Unsupported;

mod inner;
mod simple;

#[derive(Debug, Clone)]
pub struct RhusicsCoreCollisionObject {
    inner: RhusicsCoreCollisionObjectInner,
}

impl EngineCollisionObject for RhusicsCoreCollisionObject {
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

    fn collides_continuous(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> Result<bool, CrccError> {
        Ok(self.as_ref().collides_continuous(
            start_pos_self,
            end_pos_self,
            other.as_ref(),
            start_pos_other,
            end_pos_other,
        )?)
    }
}

impl From<CollisionObject> for RhusicsCoreCollisionObject {
    fn from(value: CollisionObject) -> Self {
        Self {
            inner: value.into(),
        }
    }
}

impl AsRef<RhusicsCoreCollisionObjectInner> for RhusicsCoreCollisionObject {
    fn as_ref(&self) -> &RhusicsCoreCollisionObjectInner {
        &self.inner
    }
}

impl From<crate::collision_checker::engine::rhusics::inner::Unsupported> for CrccError {
    fn from(err: crate::collision_checker::engine::rhusics::inner::Unsupported) -> Self {
        // Map this to whichever variant in CollisionCheckerError handles unsupported features.
        // For example, if you have an `Unsupported` variant:
        match err {
            Unsupported(_) => CrccError::Unsupported,
        }
    }
}
