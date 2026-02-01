use crate::collision_object::simple::{
    SimpleCollisionObject, SimpleCollisionObjectOps, swept_areas,
};
use geo::Polygon;
use glamx::DPose2;
use std::ops::Deref;

#[derive(Debug, Clone, PartialEq)]
pub struct PolygonWithHoles(pub(super) Polygon);

impl PolygonWithHoles {
    pub fn new(polygon: Polygon) -> PolygonWithHoles {
        PolygonWithHoles(polygon)
    }
}

impl Deref for PolygonWithHoles {
    type Target = Polygon;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl SimpleCollisionObjectOps for PolygonWithHoles {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        swept_areas(&self.0, positions)
    }
}
