use geo::{LineString, Polygon};
use itertools::Itertools;
use parry2d_f64::query::Unsupported;
use pyo3::{exceptions::PyValueError, prelude::*};

use crate::{
    collision_checker::{
        CollisionChecker as RustCollisionChecker,
        CollisionCheckerBuilder as RustCollisionCheckerBuilder,
    },
    python::{collision_object::Shape, isometry::Isometry},
};

#[pyclass]
pub struct CollisionChecker(RustCollisionChecker);

#[pymethods]
impl CollisionChecker {
    pub fn collides_static(&self, shape: &Shape, position: &Isometry) -> PyResult<bool> {
        self.0
            .collides_static(shape.0.as_ref(), &position.0)
            .map_err(|err| match err {
                Unsupported => PyValueError::new_err("Unsupported shape combination"),
            })
    }
}

#[pyclass]
pub struct CollisionCheckerBuilder(RustCollisionCheckerBuilder);

#[pymethods]
impl CollisionCheckerBuilder {
    #[new]
    pub fn new() -> Self {
        CollisionCheckerBuilder(RustCollisionCheckerBuilder::new())
    }

    pub fn with_static_obstacle(&mut self, obstacle: &Shape, position: &Isometry) {
        self.0.with_static_obstacle(obstacle.0.clone(), position.0);
    }

    pub fn with_road_boundary_obstacle(&mut self, lanelets: Vec<Vec<(f64, f64)>>) {
        let lanelets = lanelets
            .into_iter()
            .map(|boundary| {
                Polygon::new(
                    LineString::new(boundary.into_iter().map_into().collect()),
                    vec![],
                )
            })
            .collect_vec();
        self.0.with_road_boundary_obstacle(&lanelets);
    }

    pub fn build(&self) -> CollisionChecker {
        CollisionChecker(self.0.clone().build())
    }
}

impl Default for CollisionCheckerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[pymodule]
pub(super) mod collision_checker {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::{CollisionChecker, CollisionCheckerBuilder};

    /// Hack: workaround for https://github.com/PyO3/pyo3/issues/759
    #[pymodule_init]
    fn init(m: &Bound<'_, PyModule>) -> PyResult<()> {
        Python::attach(|py| {
            py.import("sys")?
                .getattr("modules")?
                .set_item("commonroad_collision_checker._core.collision_checker", m)
        })
    }
}
