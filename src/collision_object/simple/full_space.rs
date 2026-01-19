use crate::collision_object::simple::{SimpleCollisionObject, SimpleCollisionObjectOps};
use glamx::DPose2;

#[derive(Debug, Clone, Copy)]
pub struct FullSpace;

impl SimpleCollisionObjectOps for FullSpace {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        vec![SimpleCollisionObject::full_space(); positions.len().saturating_sub(1)]
    }
}
