use crate::collision_object::simple::{SimpleCollisionObject, SweptArea};
use glamx::DPose2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Geometry representing the empty set.
pub struct Empty;

impl SweptArea for Empty {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        vec![SimpleCollisionObject::empty(); positions.len().saturating_sub(1)]
    }
}
