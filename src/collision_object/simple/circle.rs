use crate::collision_object::simple::{SimpleCollisionObject, SweptArea};
use crate::error::{CrccError, CrccResult};
use geo::{Buffer, LineString, coord};
use glamx::{DPose2, DVec2};
use itertools::Itertools;

#[derive(Debug, Clone, PartialEq)]
pub struct Circle {
    pub(super) center: (f64, f64),
    pub(super) radius: f64,
}

impl Circle {
    pub fn new(center: (f64, f64), radius: f64) -> CrccResult<Circle> {
        if radius <= 0.0 {
            return Err(CrccError::InvalidRadius(radius));
        }
        Ok(Circle { center, radius })
    }

    pub fn center(&self) -> (f64, f64) {
        self.center
    }

    pub fn radius(&self) -> f64 {
        self.radius
    }
}

impl SweptArea for Circle {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        let mut swept_areas = Vec::with_capacity(positions.len().saturating_sub(1));
        for (start_pos, end_pos) in positions.iter().tuple_windows() {
            let center = DVec2::from(self.center);
            let start = start_pos * center;
            let end = end_pos * center;
            let line = LineString::new(vec![
                coord! { x: start.x, y: start.y},
                coord! { x: end.x, y: end.y},
            ]);
            let mut buffered = line.buffer(self.radius);
            assert_eq!(
                buffered.0.len(),
                1,
                "Buffering should produce exactly one polygon."
            );
            swept_areas.push(
                SimpleCollisionObject::polygon(buffered.0.pop().unwrap())
                    .expect("Buffered circle swept area should be a valid polygon"),
            );
        }
        swept_areas
    }
}
