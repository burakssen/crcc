use nalgebra::{Isometry2, Vector2};
use pyo3::prelude::*;

#[pyclass]
pub struct Isometry {
    pub(crate) inner: Isometry2<f64>,
}

#[pymethods]
impl Isometry {
    #[new]
    pub fn translation_rotation(shift: (f64, f64), angle: f64) -> Self {
        Isometry {
            inner: Isometry2::new(Vector2::new(shift.0, shift.1), angle),
        }
    }

    #[staticmethod]
    pub fn identity() -> Self {
        Isometry {
            inner: Isometry2::identity(),
        }
    }

    #[staticmethod]
    pub fn translation(shift: (f64, f64)) -> Self {
        Isometry {
            inner: Isometry2::translation(shift.0, shift.1),
        }
    }

    #[staticmethod]
    pub fn rotation(angle: f64) -> Self {
        Isometry {
            inner: Isometry2::rotation(angle),
        }
    }

    pub fn and_then(&self, other: &Self) -> Self {
        Isometry {
            inner: self.inner * other.inner,
        }
    }
}

impl Default for Isometry {
    fn default() -> Self {
        Self::identity()
    }
}

#[pymodule]
pub(super) mod isometry {
    #[pymodule_export]
    use super::Isometry;
}
