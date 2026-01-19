use crate::collision_object::simple::{SimpleCollisionObject, SimpleCollisionObjectOps};
use glamx::{DPose2, DVec2};

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
    fn swept_areas(&self, _positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        // Always full-space if rotation changes, otherwise the one with shorter offset (?)
        todo!("Requires support for full-space collision objects.")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
