use geo::Polygon;
use std::ops::Deref;

#[derive(Debug, Clone)]
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
