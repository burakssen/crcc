use crate::collision_object::simple::{SimpleCollisionObject, SweptArea, swept_areas};
use crate::error::{CrccError, CrccResult};
use geo::{IsConvex, Polygon};
use glamx::DPose2;
use std::ops::Deref;

#[derive(Debug, Clone, PartialEq)]
pub struct ConvexPolygon(pub(super) Polygon);

impl ConvexPolygon {
    pub fn new(polygon: Polygon) -> CrccResult<ConvexPolygon> {
        if !polygon.interiors().is_empty() {
            return Err(CrccError::HasHoles);
        }
        if !polygon.exterior().is_convex() {
            return Err(CrccError::NotConvex);
        }
        Ok(ConvexPolygon(polygon))
    }

    pub fn polygon(&self) -> &Polygon {
        &self.0
    }
}

impl Deref for ConvexPolygon {
    type Target = Polygon;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl SweptArea for ConvexPolygon {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        swept_areas(&self.0, positions)
    }
}
