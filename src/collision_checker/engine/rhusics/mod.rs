use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_checker::engine::rhusics::inner::RhusicsCoreCollisionObjectInner;
use crate::collision_object::CollisionObject;
use crate::error::CrccResult;
use glamx::DPose2;

mod inner;
mod simple;

#[derive(Debug, Clone)]
/// Rhusics finite/analytic-half-space geometry for typed queries. Typed distance
/// retains the trait's unsupported default; runtime distance uses a shared fallback.
pub struct RhusicsCoreCollisionObject {
    inner: RhusicsCoreCollisionObjectInner,
}

impl EngineCollisionObject for RhusicsCoreCollisionObject {
    fn collides_at(&self, pos_self: DPose2, other: &Self, pos_other: DPose2) -> CrccResult<bool> {
        Ok(self.as_ref().collides(pos_self, other.as_ref(), pos_other))
    }

    fn collides_continuous(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> CrccResult<bool> {
        Ok(self.as_ref().collides_continuous(
            start_pos_self,
            end_pos_self,
            other.as_ref(),
            start_pos_other,
            end_pos_other,
        ))
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
