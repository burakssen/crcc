use crate::collision_object::simple::{SimpleCollisionObject, SweptArea};
use crate::error::{CrccError, CrccResult};
use glamx::{DPose2, DVec2};
use itertools::Itertools;

#[derive(Debug, Clone, PartialEq)]
/// The half-space `outward_normal · point <= offset`.
pub struct HalfSpace {
    /// The normalized outward normal.
    pub outward_normal: DVec2,
    /// The signed boundary offset along the normal.
    pub offset: f64,
}

impl HalfSpace {
    /// Creates a half-space from a directed boundary line.
    pub fn from_points(p1: impl Into<DVec2>, p2: impl Into<DVec2>) -> CrccResult<Self> {
        // Create a half space defined by the line through p1 and p2,
        // with the outward normal pointing to the left of the line
        let p1 = p1.into();
        let p2 = p2.into();
        let dir = p2 - p1;
        if !p1.is_finite() || !p2.is_finite() || dir.length_squared() == 0.0 {
            return Err(CrccError::InvalidGeometry(
                "half-space points must be finite and distinct",
            ));
        }
        let unit_normal = DVec2::new(-dir.y, dir.x).normalize();
        let offset = unit_normal.dot(p1);
        Ok(Self {
            outward_normal: unit_normal,
            offset,
        })
    }

    /// Creates the half-space `a*x + b*y <= c`.
    pub fn from_coeffs(a: f64, b: f64, c: f64) -> CrccResult<Self> {
        // Represents the half space ax + by <= c
        let normal = DVec2::new(a, b);
        if !normal.is_finite() || !c.is_finite() || normal.length_squared() == 0.0 {
            return Err(CrccError::InvalidGeometry(
                "half-space coefficients must be finite with a nonzero normal",
            ));
        }
        let offset = c / normal.length();
        Ok(Self {
            outward_normal: normal.normalize(),
            offset,
        })
    }

    /// Compares two normalized half-spaces with the default tolerance.
    pub fn almost_equal(&self, other: &HalfSpace) -> bool {
        self.almost_equal_with_tol(other, 1e-9)
    }

    /// Compares two normalized half-spaces with `tol`.
    pub fn almost_equal_with_tol(&self, other: &HalfSpace, tol: f64) -> bool {
        (self.outward_normal - other.outward_normal).length().abs() < tol
            && (self.offset - other.offset).abs() < tol
    }
}

impl SweptArea for HalfSpace {
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
                let offset = start_offset.max(end_offset) + self.offset;
                swept_areas.push(
                    SimpleCollisionObject::half_space(outward_normal, offset)
                        .expect("a transformed valid half-space remains valid"),
                );
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
    fn constructors_create_expected_half_spaces() {
        let hs = HalfSpace {
            outward_normal: DVec2::new(4.0, 3.0).normalize(),
            offset: 5.0,
        };
        let hs_from_points = HalfSpace::from_points((1.0, 7.0), (4.0, 3.0)).unwrap();
        let hs_from_coeffs = HalfSpace::from_coeffs(4.0, 3.0, 25.0).unwrap();
        assert!(hs.almost_equal(&hs_from_points));
        assert!(hs.almost_equal(&hs_from_coeffs));
    }

    #[test]
    fn constructors_reject_degenerate_half_spaces() {
        assert!(HalfSpace::from_points((1.0, 1.0), (1.0, 1.0)).is_err());
        assert!(HalfSpace::from_coeffs(0.0, 0.0, 1.0).is_err());
        assert!(HalfSpace::from_coeffs(f64::NAN, 1.0, 1.0).is_err());
    }

    #[test]
    fn swept_area_covers_translated_half_space() {
        let hs = HalfSpace::from_coeffs(1.0, 0.0, 5.0).unwrap(); // x <= 5.0
        let swept_area = hs.swept_area(
            DPose2::new(DVec2::new(0.0, 1.0), FRAC_PI_2),
            DPose2::new(DVec2::new(0.0, -1.0), FRAC_PI_2),
        );
        let expected = HalfSpace::from_coeffs(0.0, 1.0, 6.0).unwrap(); // y <= 6.0
        match swept_area {
            SimpleCollisionObject::HalfSpace(swept_hs) => {
                assert!(
                    swept_hs.almost_equal(&expected),
                    "Expected {:?}, got {:?}",
                    expected,
                    swept_hs
                );
            }
            _ => panic!("Expected HalfSpace"),
        }
    }
}
