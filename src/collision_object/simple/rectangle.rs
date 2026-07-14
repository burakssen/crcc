use crate::collision_object::simple::{SimpleCollisionObject, SweptArea, swept_areas};
use crate::error::{CrccError, CrccResult};
use geo::{HasDimensions, Polygon, Rect, Rotate};
use glamx::DPose2;

#[derive(Debug, Clone, PartialEq)]
/// A rectangle with a local orientation in radians.
pub struct Rectangle {
    pub(super) rect: Rect,
    pub(super) orientation: f64,
}

impl Rectangle {
    /// Creates a finite, non-empty oriented rectangle.
    pub fn new(rect: Rect, orientation: f64) -> CrccResult<Rectangle> {
        if !rect.min().x.is_finite()
            || !rect.min().y.is_finite()
            || !rect.max().x.is_finite()
            || !rect.max().y.is_finite()
            || !orientation.is_finite()
        {
            return Err(CrccError::InvalidGeometry(
                "rectangle coordinates and orientation must be finite",
            ));
        }
        if rect.is_empty() {
            return Err(CrccError::EmptyShape);
        }
        Ok(Rectangle { rect, orientation })
    }

    /// Returns the underlying axis-aligned local rectangle.
    pub fn rect(&self) -> &Rect {
        &self.rect
    }

    /// Returns the local-space center.
    pub fn center(&self) -> (f64, f64) {
        self.rect.center().into()
    }

    /// Returns the local x extent.
    pub fn width(&self) -> f64 {
        self.rect.width()
    }

    /// Returns the local y extent.
    pub fn height(&self) -> f64 {
        self.rect.height()
    }

    /// Returns the local orientation in radians.
    pub fn orientation(&self) -> f64 {
        self.orientation
    }
}

impl SweptArea for Rectangle {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        let mut poly = Polygon::from(self.rect);
        poly.rotate_around_center_mut(self.orientation.to_degrees());
        swept_areas(&poly, positions)
    }
}
