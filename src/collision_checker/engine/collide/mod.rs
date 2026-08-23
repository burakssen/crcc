use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_checker::engine::collide::inner::CollideCollisionObjectInner;
use crate::collision_object::CollisionObject;
use crate::error::CrccResult;
use glamx::DPose2;

mod inner;
mod manager;
mod simple;

#[derive(Debug, Clone)]
pub struct CollideCollisionObject {
    inner: CollideCollisionObjectInner,
}

impl EngineCollisionObject for CollideCollisionObject {
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

impl From<CollisionObject> for CollideCollisionObject {
    fn from(value: CollisionObject) -> Self {
        Self {
            inner: value.into(),
        }
    }
}

impl AsRef<CollideCollisionObjectInner> for CollideCollisionObject {
    fn as_ref(&self) -> &CollideCollisionObjectInner {
        &self.inner
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn collide_backend_stays_self_contained() {
        let sources = [
            include_str!("mod.rs"),
            include_str!("inner.rs"),
            include_str!("manager.rs"),
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
