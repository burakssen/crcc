use geo::{HasDimensions, Triangle as GeoTriangle};
use std::ops::Deref;

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
