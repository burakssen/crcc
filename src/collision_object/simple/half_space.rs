use crate::collision_object::simple::{SimpleCollisionObject, SimpleCollisionObjectOps};
use glamx::{DPose2, DVec2};
use itertools::Itertools;

#[derive(Debug, Clone)]
pub struct HalfSpace {
    pub outward_normal: DVec2,
    pub offset: f64,
}

impl HalfSpace {
    pub fn from_points(p1: impl Into<DVec2>, p2: impl Into<DVec2>) -> Self {
        // Create a half space defined by the line through p1 and p2,
        // with the outward normal pointing to the left of the line
        let p1 = p1.into();
        let p2 = p2.into();
        let dir = p2 - p1;
        let unit_normal = DVec2::new(-dir.y, dir.x).normalize();
        let offset = unit_normal.dot(p1);
        Self {
            outward_normal: unit_normal,
            offset,
        }
    }

    pub fn from_coeffs(a: f64, b: f64, c: f64) -> Self {
        // Represents the half space ax + by <= c
        let normal = DVec2::new(a, b);
        let offset = c / normal.length();
        Self {
            outward_normal: normal.normalize(),
            offset,
        }
    }

    pub fn almost_equal(&self, other: &HalfSpace) -> bool {
        self.almost_equal_with_tol(other, 1e-9)
    }

    pub fn almost_equal_with_tol(&self, other: &HalfSpace, tol: f64) -> bool {
        (self.outward_normal - other.outward_normal).length().abs() < tol
            && (self.offset - other.offset).abs() < tol
    }
}

impl SimpleCollisionObjectOps for HalfSpace {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        // This could still be optimized if we allow to return a regular CollisionObject,
        // which would allow us to return a union of half spaces.
        let mut swept_areas = Vec::with_capacity(positions.len().saturating_sub(1));
        for (start_pos, end_pos) in positions.iter().tuple_windows() {
            if (start_pos.rotation.angle() - end_pos.rotation.angle()).abs() > 1e-9 {
                swept_areas.push(SimpleCollisionObject::full_space());
            } else {
                let outward_normal = start_pos.rotation * self.outward_normal;
                let start_offset = outward_normal.dot(start_pos.translation);
                let end_offset = outward_normal.dot(end_pos.translation);
                let offset = start_offset.min(end_offset) + self.offset;
                swept_areas.push(SimpleCollisionObject::half_space(outward_normal, offset));
            }
        }
        swept_areas
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn test_constructors() {
        let hs = HalfSpace {
            outward_normal: DVec2::new(4.0, 3.0).normalize(),
            offset: 5.0,
        };
        let hs_from_points = HalfSpace::from_points((1.0, 7.0), (4.0, 3.0));
        let hs_from_coeffs = HalfSpace::from_coeffs(4.0, 3.0, 25.0);
        assert!(hs.almost_equal(&hs_from_points));
        assert!(hs.almost_equal(&hs_from_coeffs));
    }

    #[test]
    fn test_swept_area() {
        let hs = HalfSpace::from_coeffs(1.0, 0.0, 5.0); // x <= 5.0
        let swept_area = hs.swept_area(
            DPose2::new(DVec2::new(0.0, 1.0), FRAC_PI_2),
            DPose2::new(DVec2::new(0.0, -1.0), FRAC_PI_2),
        );
        let expected = HalfSpace::from_coeffs(0.0, 1.0, 4.0); // y <= 4.0
        match swept_area {
            SimpleCollisionObject::HalfSpace(swept_hs) => {
                assert!(swept_hs.almost_equal(&expected));
            }
            _ => panic!("Expected HalfSpace"),
        }
    }
}
