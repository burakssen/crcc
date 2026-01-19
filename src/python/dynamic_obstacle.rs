use crate::collision_checker::ParryCollisionObject;
use crate::dynamic_obstacle::{DynamicObstacle as RustDynamicObstacle, GenericDynamicObstacle};
use crate::python::collision_object::CollisionObject;
use crate::python::pose::Pose;
use crate::time::TimeStepInner;
use pyo3::prelude::*;
use std::sync::{Arc, OnceLock};

#[pyclass]
pub struct DynamicObstacle {
    pub(crate) plain_dyn_obs: Arc<RustDynamicObstacle>,
    parry_dyn_obs: Arc<OnceLock<GenericDynamicObstacle<ParryCollisionObject>>>,
}

#[pymethods]
impl DynamicObstacle {
    #[new]
    pub fn new(shape: &CollisionObject, positions: Vec<Pose>, time_offset: TimeStepInner) -> Self {
        let plain_dyn_obs = RustDynamicObstacle::new(
            shape.plain_collision_object.as_ref().clone(),
            positions.into_iter().map(|iso| iso.0).collect(),
            time_offset.into(),
        );
        Self {
            plain_dyn_obs: Arc::new(plain_dyn_obs),
            parry_dyn_obs: Arc::new(OnceLock::new()),
        }
    }
}

impl DynamicObstacle {
    pub(crate) fn get_parry(&self) -> &GenericDynamicObstacle<ParryCollisionObject> {
        self.parry_dyn_obs
            .get_or_init(|| self.plain_dyn_obs.as_ref().clone().convert_repr())
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
