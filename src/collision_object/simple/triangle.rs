use crate::collision_object::simple::{
    SimpleCollisionObject, SimpleCollisionObjectOps, swept_areas,
};
use geo::{HasDimensions, Triangle as GeoTriangle};
use nalgebra::Isometry2;
use std::ops::Deref;

#[derive(Debug, Clone)]
pub struct Triangle(pub(super) GeoTriangle);

impl Triangle {
    pub fn new(triangle: GeoTriangle) -> Triangle {
        if triangle.is_empty() {
            panic!("Triangle must not be empty.");
        }
        Triangle(triangle)
    }
}

impl Deref for Triangle {
    type Target = GeoTriangle;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl SimpleCollisionObjectOps for Triangle {
    fn swept_areas(&self, positions: &[Isometry2<f64>]) -> Vec<SimpleCollisionObject> {
        swept_areas(&self.0, positions)
    }
}
