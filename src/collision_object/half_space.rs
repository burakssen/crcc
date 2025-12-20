use nalgebra::{Unit, Vector2};

#[derive(Debug, Clone)]
pub struct HalfSpace {
    pub outward_normal: Unit<Vector2<f64>>,
    pub offset: f64,
}

impl HalfSpace {
    pub fn from_points(p1: (f64, f64), p2: (f64, f64)) -> Self {
        // Create a half space defined by the line through p1 and p2,
        // with the outward normal pointing to the left of the line
        let p1 = Vector2::new(p1.0, p1.1);
        let p2 = Vector2::new(p2.0, p2.1);
        let dir = p2 - p1;
        let unit_normal = Unit::new_normalize(Vector2::new(-dir.y, dir.x));
        let offset = unit_normal.dot(&p1);
        Self {
            outward_normal: unit_normal,
            offset,
        }
    }

    pub fn from_coeffs(a: f64, b: f64, c: f64) -> Self {
        // Represents the half space ax + by <= c
        let normal = Vector2::new(a, b);
        let unit_normal = Unit::new_normalize(normal);
        let offset = c / normal.norm();
        Self {
            outward_normal: unit_normal,
            offset,
        }
    }

    pub fn almost_equal(&self, other: &HalfSpace) -> bool {
        self.almost_equal_with_tol(other, 1e-9)
    }

    pub fn almost_equal_with_tol(&self, other: &HalfSpace, tol: f64) -> bool {
        (*self.outward_normal - *other.outward_normal).norm().abs() < tol
            && (self.offset - other.offset).abs() < tol
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constructors() {
        let hs = HalfSpace {
            outward_normal: Unit::new_normalize(Vector2::new(4.0, 3.0)),
            offset: 5.0,
        };
        let hs_from_points = HalfSpace::from_points((1.0, 7.0), (4.0, 3.0));
        let hs_from_coeffs = HalfSpace::from_coeffs(4.0, 3.0, 25.0);
        assert!(hs.almost_equal(&hs_from_points));
        assert!(hs.almost_equal(&hs_from_coeffs));
    }
}
