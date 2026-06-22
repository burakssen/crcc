use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_checker::engine::collide::inner::{CollideCollisionObjectInner, Unsupported};
use crate::collision_object::CollisionObject;
use crate::error::CrccError;
use glamx::DPose2;

mod inner;
mod simple;

#[derive(Debug, Clone)]
pub struct CollideCollisionObject {
    inner: CollideCollisionObjectInner,
    original: CollisionObject,
}

impl EngineCollisionObject for CollideCollisionObject {
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

    fn distance_at(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<f64, CrccError> {
        self.original.distance(pos_self, &other.original, pos_other)
    }
}

impl From<CollisionObject> for CollideCollisionObject {
    fn from(value: CollisionObject) -> Self {
        Self {
            inner: value.clone().into(),
            original: value,
        }
    }
}

impl AsRef<CollideCollisionObjectInner> for CollideCollisionObject {
    fn as_ref(&self) -> &CollideCollisionObjectInner {
        &self.inner
    }
}

impl From<Unsupported> for CrccError {
    fn from(_error: Unsupported) -> Self {
        CrccError::Unsupported
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn collide_backend_stays_self_contained() {
        let sources = [
            include_str!("mod.rs"),
            include_str!("inner.rs"),
            include_str!("simple.rs"),
        ];
        let forbidden = [
            concat!("cg", "math"),
            concat!("par", "ry"),
            concat!("rhu", "sics"),
        ];
        for source in sources {
            for pattern in forbidden {
                assert!(!source.contains(pattern));
            }
        }
    }
}
