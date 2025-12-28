use crate::collision_object::simple::{SimpleCollisionObject, SimpleCollisionObjectOps};
use geo::{Buffer, LineString, coord};
use itertools::Itertools;
use nalgebra::{Isometry2, Point2};

#[derive(Debug, Clone)]
pub struct Circle {
    pub(super) center: (f64, f64),
    pub(super) radius: f64,
}

impl Circle {
    pub fn new(center: (f64, f64), radius: f64) -> Circle {
        if radius <= 0.0 {
            panic!("Circle radius must be positive.");
        }
        Circle { center, radius }
    }

    pub fn center(&self) -> (f64, f64) {
        self.center
    }

    pub fn radius(&self) -> f64 {
        self.radius
    }
}

impl SimpleCollisionObjectOps for Circle {
    fn swept_areas(&self, positions: &[Isometry2<f64>]) -> Vec<SimpleCollisionObject> {
        let mut swept_areas = Vec::with_capacity(positions.len().saturating_sub(1));
        for (start_pos, end_pos) in positions.iter().tuple_windows() {
            let center = Point2::new(self.center.0, self.center.1);
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
            swept_areas.push(SimpleCollisionObject::polygon(buffered.0.pop().unwrap()));
        }
        swept_areas
    }
}
