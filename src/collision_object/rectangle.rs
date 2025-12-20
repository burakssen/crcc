use geo::{HasDimensions, Rect};
use std::ops::Deref;

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
