use crate::collision_object::simple::{
    SimpleCollisionObject, SimpleCollisionObjectOps, swept_areas,
};
use geo::{IsConvex, Polygon};
use nalgebra::Isometry2;
use std::ops::Deref;

#[derive(Debug, Clone)]
pub struct ConvexPolygon(pub(super) Polygon);

impl ConvexPolygon {
    pub fn new(polygon: Polygon) -> ConvexPolygon {
        if !polygon.exterior().is_convex() || !polygon.interiors().is_empty() {
            panic!("ConvexPolygon must be convex and may not have holes.")
        }
        ConvexPolygon(polygon)
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

impl SimpleCollisionObjectOps for ConvexPolygon {
    fn swept_areas(&self, positions: &[Isometry2<f64>]) -> Vec<SimpleCollisionObject> {
        swept_areas(&self.0, positions)
    }
}
