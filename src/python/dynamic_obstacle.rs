use crate::collision_object::DynamicObstacle as RustDynamicObstacle;
use crate::python::collision_object::CollisionObject;
use crate::python::pose::Pose;
use crate::time::TimeStepInner;
use glamx::DPose2;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use std::sync::Arc;

/// A discrete obstacle trajectory used by `CollisionChecker`.
///
/// `DynamicObstacle(shape, positions, time_offset)` keeps one shape and assigns
/// successive poses to successive integer time steps. Adjacent poses are joined
/// by conservative continuous collision checks.
#[pyclass]
#[derive(Clone)]
pub struct DynamicObstacle(Arc<RustDynamicObstacle>);

impl AsRef<RustDynamicObstacle> for DynamicObstacle {
    fn as_ref(&self) -> &RustDynamicObstacle {
        &self.0
    }
}

#[pymethods]
impl DynamicObstacle {
    #[new]
    /// Creates a fixed-shape trajectory.
    pub fn new(
        shape: &CollisionObject,
        positions: Vec<Pose>,
        time_offset: TimeStepInner,
    ) -> PyResult<Self> {
        let dynamic_obstacle = RustDynamicObstacle::new(
            shape.as_ref().clone(),
            positions.into_iter().map(|position| position.0).collect(),
            time_offset.into(),
        )?;

        Ok(Self(Arc::new(dynamic_obstacle)))
    }

    #[staticmethod]
    #[pyo3(signature = (obstacles, time_offset = 0, positions = None))]
    /// Creates a trajectory whose shape may vary at each time step.
    ///
    /// `positions` defaults to identity poses.
    ///
    /// # Errors
    ///
    /// Returns a Python `ValueError` when the numbers of shapes and poses
    /// differ.
    pub fn from_time_variant(
        obstacles: Vec<CollisionObject>,
        time_offset: TimeStepInner,
        positions: Option<Vec<Pose>>,
    ) -> PyResult<Self> {
        let positions = positions.map_or_else(
            || vec![DPose2::IDENTITY; obstacles.len()],
            |positions| positions.into_iter().map(|position| position.0).collect(),
        );

        if obstacles.len() != positions.len() {
            return Err(PyValueError::new_err(
                "obstacles and positions must have the same length",
            ));
        }

        let dynamic_obstacle = RustDynamicObstacle::time_variant(
            obstacles
                .into_iter()
                .map(|obstacle| obstacle.as_ref().clone())
                .collect(),
            positions,
            time_offset.into(),
        )?;

        Ok(Self(Arc::new(dynamic_obstacle)))
    }
}

#[pymodule]
pub(super) mod dynamic_obstacle {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::DynamicObstacle;

    /// Hack: workaround for <https://github.com/PyO3/pyo3/issues/759>.
    #[pymodule_init]
    fn init(module: &Bound<'_, PyModule>) -> PyResult<()> {
        Python::attach(|python| {
            python
                .import("sys")?
                .getattr("modules")?
                .set_item("crcc._core.dynamic_obstacle", module)
        })
    }
}
