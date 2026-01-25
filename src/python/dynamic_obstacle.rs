use crate::dynamic_obstacle::DynamicObstacle as RustDynamicObstacle;
use crate::python::collision_object::CollisionObject;
use crate::python::pose::Pose;
use crate::time::TimeStepInner;
use pyo3::prelude::*;
use std::sync::Arc;

#[pyclass]
pub struct DynamicObstacle(Arc<RustDynamicObstacle>);

impl AsRef<RustDynamicObstacle> for DynamicObstacle {
    fn as_ref(&self) -> &RustDynamicObstacle {
        &self.0
    }
}

#[pymethods]
impl DynamicObstacle {
    #[new]
    pub fn new(shape: &CollisionObject, positions: Vec<Pose>, time_offset: TimeStepInner) -> Self {
        let dyn_obs = RustDynamicObstacle::new(
            shape.as_ref().clone(),
            positions.into_iter().map(|pos| pos.0).collect(),
            time_offset.into(),
        );
        Self(Arc::new(dyn_obs))
    }
}

#[pymodule]
pub(super) mod dynamic_obstacle {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::DynamicObstacle;

    /// Hack: workaround for https://github.com/PyO3/pyo3/issues/759
    #[pymodule_init]
    fn init(m: &Bound<'_, PyModule>) -> PyResult<()> {
        Python::attach(|py| {
            py.import("sys")?
                .getattr("modules")?
                .set_item("commonroad_collision_checker._core.dynamic_obstacle", m)
        })
    }
}
