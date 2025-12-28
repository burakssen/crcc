use crate::collision_object::simple::{
    SimpleCollisionObject, SimpleCollisionObjectOps, swept_areas,
};
use geo::{HasDimensions, Rect};
use nalgebra::Isometry2;
use std::ops::Deref;

#[derive(Debug, Clone)]
pub struct Rectangle(pub(super) Rect);

impl Rectangle {
    pub fn new(rect: Rect) -> Rectangle {
        if rect.is_empty() {
            panic!("Rectangle must not be empty.");
        }
        Rectangle(rect)
    }
}

impl Deref for Rectangle {
    type Target = Rect;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl SimpleCollisionObjectOps for Rectangle {
    fn swept_areas(&self, positions: &[Isometry2<f64>]) -> Vec<SimpleCollisionObject> {
        swept_areas(&self.0, positions)
    }
}
