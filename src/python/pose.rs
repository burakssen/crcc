use glamx::{DPose2, DVec2};
use pyo3::prelude::*;

#[pyclass]
#[derive(Clone, Copy)]
pub struct Pose(pub(crate) DPose2);

#[pymethods]
impl Pose {
    #[new]
    pub fn new(translation: (f64, f64), angle: f64) -> Self {
        Pose(DPose2::new(DVec2::new(translation.0, translation.1), angle))
    }

    #[staticmethod]
    pub fn identity() -> Self {
        Pose(DPose2::IDENTITY)
    }

    #[staticmethod]
    pub fn from_translation(translation: (f64, f64)) -> Self {
        Pose(DPose2::translation(translation.0, translation.1))
    }

    #[staticmethod]
    pub fn from_rotation(angle: f64) -> Self {
        Pose(DPose2::rotation(angle))
    }

    #[getter]
    pub fn translation(&self) -> (f64, f64) {
        self.0.translation.into()
    }

    #[getter]
    pub fn rotation(&self) -> f64 {
        self.0.rotation.angle()
    }

    pub fn compose(&self, other: &Self) -> Self {
        Pose(self.0 * other.0)
    }

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

    /// Hack: workaround for https://github.com/PyO3/pyo3/issues/759
    #[pymodule_init]
    fn init(m: &Bound<'_, PyModule>) -> PyResult<()> {
        Python::attach(|py| {
            py.import("sys")?
                .getattr("modules")?
                .set_item("crcc._core.pose", m)
        })
    }
}
