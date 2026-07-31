use glamx::{DPose2, DVec2};
use pyo3::prelude::*;
use std::ops::Mul;

/// A rigid 2D transform with translation in metres and rotation in radians.
///
/// `Pose((x, y), angle)` creates a transform. Use `identity()`,
/// `from_translation()`, or `from_rotation()` for common cases. Multiplication
/// and `compose()` apply the right-hand pose first.
#[pyclass]
#[derive(Clone, Copy)]
pub struct Pose(pub(crate) DPose2);

#[pymethods]
impl Pose {
    #[new]
    /// Creates a pose from an `(x, y)` translation and a counter-clockwise angle.
    pub fn new(translation: (f64, f64), angle: f64) -> Self {
        Self(DPose2::new(DVec2::new(translation.0, translation.1), angle))
    }

    #[staticmethod]
    /// Returns the identity transform.
    pub const fn identity() -> Self {
        Self(DPose2::IDENTITY)
    }

    #[staticmethod]
    /// Returns a pure translation.
    pub fn from_translation(translation: (f64, f64)) -> Self {
        Self(DPose2::translation(translation.0, translation.1))
    }

    #[staticmethod]
    /// Returns a pure counter-clockwise rotation in radians.
    pub fn from_rotation(angle: f64) -> Self {
        Self(DPose2::rotation(angle))
    }

    #[getter]
    /// The `(x, y)` translation.
    pub fn translation(&self) -> (f64, f64) {
        self.0.translation.into()
    }

    #[getter]
    /// The counter-clockwise rotation in radians.
    pub fn rotation(&self) -> f64 {
        self.0.rotation.angle()
    }

    /// Composes this transform with `other`.
    #[must_use]
    pub fn compose(&self, other: &Self) -> Self {
        Self(self.0.mul(other.0))
    }

    #[must_use]
    pub fn __mul__(&self, other: &Self) -> Self {
        self.compose(other)
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self::identity()
    }
}

#[pymodule]
pub(super) mod pose {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::Pose;

    /// Hack: workaround for <https://github.com/PyO3/pyo3/issues/759>.
    #[pymodule_init]
    fn init(module: &Bound<'_, PyModule>) -> PyResult<()> {
        Python::attach(|python| {
            python
                .import("sys")?
                .getattr("modules")?
                .set_item("crcc._core.pose", module)
        })
    }
}
