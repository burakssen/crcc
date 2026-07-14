use crate::collision_object::simple::{SimpleCollisionObject, SweptArea};
use glamx::DPose2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Geometry representing the entire plane.
pub struct FullSpace;

impl SweptArea for FullSpace {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        vec![SimpleCollisionObject::full_space(); positions.len().saturating_sub(1)]
    }
}
