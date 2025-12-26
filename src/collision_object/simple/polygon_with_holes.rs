use geo::Polygon;
use std::ops::Deref;

#[derive(Debug, Clone)]
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
