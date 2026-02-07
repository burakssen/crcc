use crate::collision_object::simple::{SimpleCollisionObject, SweptArea, swept_areas};
use geo::Polygon;
use glamx::DPose2;
use std::ops::Deref;

#[derive(Debug, Clone, PartialEq)]
pub struct NonConvexPolygon(pub(super) Polygon);

impl NonConvexPolygon {
    pub fn new(polygon: Polygon) -> NonConvexPolygon {
        if !polygon.interiors().is_empty() {
            panic!("NonConvexPolygon must not have holes.")
        }
        NonConvexPolygon(polygon)
    }

    pub fn polygon(&self) -> &Polygon {
        &self.0
    }
}

impl Deref for NonConvexPolygon {
    type Target = Polygon;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl SweptArea for NonConvexPolygon {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        swept_areas(&self.0, positions)
    }
}
