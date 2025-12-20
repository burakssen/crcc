use crate::collision_object::CollisionObject;

pub mod parry;

pub trait Solver {
    type CollisionObjectRepr: From<CollisionObject>;
    type CollidesError;

    fn from_collision_objects(collision_objects: Vec<CollisionObject>) -> Self;

    fn collides(&self, obj: &Self::CollisionObjectRepr) -> Result<bool, Self::CollidesError>;
}
