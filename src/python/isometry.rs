use nalgebra::{Isometry2, Vector2};
use pyo3::prelude::*;

#[pyclass]
#[derive(Clone, Copy)]
pub struct Isometry(pub(crate) Isometry2<f64>);

#[pymethods]
impl Isometry {
    #[new]
    pub fn translation_rotation(shift: (f64, f64), angle: f64) -> Self {
        Isometry(Isometry2::new(Vector2::new(shift.0, shift.1), angle))
    }

    #[staticmethod]
    pub fn identity() -> Self {
        Isometry(Isometry2::identity())
    }

    #[staticmethod]
    pub fn translation(shift: (f64, f64)) -> Self {
        Isometry(Isometry2::translation(shift.0, shift.1))
    }

    #[staticmethod]
    pub fn rotation(angle: f64) -> Self {
        Isometry(Isometry2::rotation(angle))
    }

    pub fn and_then(&self, other: &Self) -> Self {
        Isometry(self.0 * other.0)
    }
}

impl Default for Isometry {
    fn default() -> Self {
        Self::identity()
    }
}

#[pymodule]
pub(super) mod isometry {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::Isometry;

    /// Hack: workaround for https://github.com/PyO3/pyo3/issues/759
    #[pymodule_init]
    fn init(m: &Bound<'_, PyModule>) -> PyResult<()> {
        Python::attach(|py| {
            py.import("sys")?
                .getattr("modules")?
                .set_item("commonroad_collision_checker._core.isometry", m)
        })
    }
}
