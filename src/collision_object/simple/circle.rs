use crate::collision_object::simple::{SimpleCollisionObject, SweptArea, rotation_changed};
use crate::error::{CrccError, CrccResult};
use geo::Rect;
use glamx::{DPose2, DVec2};
use itertools::Itertools;

#[derive(Debug, Clone, PartialEq)]
/// Circle geometry accepted by [`crate::CollisionObject::from`].
pub struct Circle {
    pub(super) center: (f64, f64),
    pub(super) radius: f64,
}

impl Circle {
    /// Creates a circle with a finite center and strictly positive radius.
    pub fn new(center: (f64, f64), radius: f64) -> CrccResult<Circle> {
        if !center.0.is_finite() || !center.1.is_finite() || !radius.is_finite() || radius <= 0.0 {
            return Err(CrccError::InvalidRadius(radius));
        }
        Ok(Circle { center, radius })
    }

    /// Returns the local-space center.
    pub fn center(&self) -> (f64, f64) {
        self.center
    }

    /// Returns the radius.
    pub fn radius(&self) -> f64 {
        self.radius
    }
}

impl SweptArea for Circle {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        let mut swept_areas = Vec::with_capacity(positions.len().saturating_sub(1));
        for (start_pos, end_pos) in positions.iter().tuple_windows() {
            if rotation_changed(*start_pos, *end_pos) && self.center != (0.0, 0.0) {
                let bound_radius = DVec2::from(self.center).length() + self.radius;
                swept_areas.push(
                    SimpleCollisionObject::rectangle(
                        Rect::new(
                            (
                                start_pos.translation.x.min(end_pos.translation.x) - bound_radius,
                                start_pos.translation.y.min(end_pos.translation.y) - bound_radius,
                            ),
                            (
                                start_pos.translation.x.max(end_pos.translation.x) + bound_radius,
                                start_pos.translation.y.max(end_pos.translation.y) + bound_radius,
                            ),
                        ),
                        0.0,
                    )
                    .expect("a finite circle has a valid rotational swept bound"),
                );
                continue;
            }
            let center = DVec2::from(self.center);
            let start = start_pos * center;
            let end = end_pos * center;
            swept_areas.push(
                SimpleCollisionObject::rectangle(
                    Rect::new(
                        (
                            start.x.min(end.x) - self.radius,
                            start.y.min(end.y) - self.radius,
                        ),
                        (
                            start.x.max(end.x) + self.radius,
                            start.y.max(end.y) + self.radius,
                        ),
                    ),
                    0.0,
                )
                .expect("a finite circle has a valid swept bounding rectangle"),
            );
        }
        swept_areas
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_nonfinite_geometry() {
        assert!(Circle::new((f64::NAN, 0.0), 1.0).is_err());
        assert!(Circle::new((0.0, 0.0), f64::NAN).is_err());
        assert!(Circle::new((0.0, f64::INFINITY), 1.0).is_err());
    }
}
