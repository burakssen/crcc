use crate::collision_object::simple::{
    SimpleCollisionObject, SimpleCollisionObjectOps, swept_areas,
};
use geo::{HasDimensions, Polygon, Rect, Rotate};
use glamx::DPose2;

#[derive(Debug, Clone, PartialEq)]
pub struct Rectangle {
    pub(super) rect: Rect,
    pub(super) orientation: f64,
}

impl Rectangle {
    pub fn new(rect: Rect, orientation: f64) -> Rectangle {
        if rect.is_empty() {
            panic!("Rectangle must not be empty.");
        }
        Rectangle { rect, orientation }
    }

    pub fn rect(&self) -> &Rect {
        &self.rect
    }

    pub fn center(&self) -> (f64, f64) {
        self.rect.center().into()
    }

    pub fn width(&self) -> f64 {
        self.rect.width()
    }

    pub fn height(&self) -> f64 {
        self.rect.height()
    }

    pub fn orientation(&self) -> f64 {
        self.orientation
    }
}

impl SimpleCollisionObjectOps for Rectangle {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        let mut poly = Polygon::from(self.rect);
        poly.rotate_around_center_mut(self.orientation.to_degrees());
        swept_areas(&poly, positions)
    }
}
