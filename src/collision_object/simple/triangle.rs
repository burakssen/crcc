use crate::collision_object::simple::{SimpleCollisionObject, SweptArea, swept_areas};
use crate::error::{CrccError, CrccResult};
use geo::{HasDimensions, Triangle as GeoTriangle};
use glamx::DPose2;
use std::ops::Deref;

#[derive(Debug, Clone, PartialEq)]
pub struct Triangle(pub(super) GeoTriangle);

impl Triangle {
    pub fn new(triangle: GeoTriangle) -> CrccResult<Triangle> {
        if triangle.is_empty() {
            return Err(CrccError::EmptyShape);
        }
        Ok(Triangle(triangle))
    }
}

impl Deref for Triangle {
    type Target = GeoTriangle;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl SweptArea for Triangle {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        swept_areas(&self.0, positions)
    }
}
