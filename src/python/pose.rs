use glamx::{DPose2, DVec2};
use pyo3::prelude::*;

#[pyclass]
#[derive(Clone, Copy)]
pub struct Pose(pub(crate) DPose2);

#[pymethods]
impl Pose {
    #[new]
    pub fn translation_rotation(translation: (f64, f64), angle: f64) -> Self {
        Pose(DPose2::new(DVec2::new(translation.0, translation.1), angle))
    }

    #[staticmethod]
    pub fn identity() -> Self {
        Pose(DPose2::IDENTITY)
    }

    #[staticmethod]
    pub fn translation(translation: (f64, f64)) -> Self {
        Pose(DPose2::translation(translation.0, translation.1))
    }

    #[staticmethod]
    pub fn rotation(angle: f64) -> Self {
        Pose(DPose2::rotation(angle))
    }

    pub fn and_then(&self, other: &Self) -> Self {
        Pose(self.0 * other.0)
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
                .set_item("commonroad_collision_checker._core.pose", m)
        })
    }
}
