use crate::collision_checker::CollisionCheckerError;
use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_object::CollisionObject;
use cfg_if::cfg_if;
use delegate::delegate;
use glamx::DPose2;

cfg_if!(
    // Add new default engines here above the one enabled in the default features of this crate.
    // This ensures that the engine a crate user selected is picked even if they do not deactivate
    // the default features of this crate explicitly.
    if #[cfg(feature = "parry-default-engine")] {
        use crate::collision_checker::engine::parry::ParryCollisionObject;
        type DefaultCollisionObjectInner = ParryCollisionObject;
    } else {
        compile_error!("do not enable the `default-engine` feature manually. Instead choose a specific engine as the default by enabling the corresponding feature, e.g., `parry-default-engine`");
    }
);

#[derive(Clone, Debug)]
pub struct DefaultEngineCollisionObject(DefaultCollisionObjectInner);

impl EngineCollisionObject for DefaultEngineCollisionObject {
    delegate! {
        to self.0 {
            fn collides_at(
                &self,
                pos_self: DPose2,
                #[as_ref] other: &Self,
                pos_other: DPose2,
            ) -> Result<bool, CollisionCheckerError>;

            fn collides_continuous(
                &self,
                start_pos_self: DPose2,
                end_pos_self: DPose2,
                #[as_ref] other: &Self,
                start_pos_other: DPose2,
                end_pos_other: DPose2,
            ) -> Result<bool, CollisionCheckerError>;
        }
    }
}

impl From<CollisionObject> for DefaultEngineCollisionObject {
    fn from(value: CollisionObject) -> Self {
        Self(value.into())
    }
}

impl AsRef<DefaultCollisionObjectInner> for DefaultEngineCollisionObject {
    fn as_ref(&self) -> &DefaultCollisionObjectInner {
        &self.0
    }
}
