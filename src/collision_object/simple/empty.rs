use crate::collision_object::simple::{SimpleCollisionObject, SimpleCollisionObjectOps};
use glamx::DPose2;

#[derive(Debug, Clone, Copy)]
pub struct Empty {}

impl SimpleCollisionObjectOps for Empty {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        vec![SimpleCollisionObject::empty(); positions.len().saturating_sub(1)]
    }
}
