use crate::collision_checker::CollisionCheckerError;
use crate::collision_checker::{
    CollisionChecker as RustCollisionChecker,
    CollisionCheckerBuilder as RustCollisionCheckerBuilder, CollisionStatus as RustCollisionStatus,
};
use crate::collision_object::CollisionObject as RustCollisionObject;
use crate::python::collision_object::CollisionObject;
use crate::python::dynamic_obstacle::DynamicObstacle;
use crate::python::pose::Pose;
use crate::time::{TimeStep, TimeStepInner};
use geo::Polygon;
use glamx::DPose2;
use itertools::Itertools;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use replace_with::replace_with;
use std::fmt::Display;
use std::ops::RangeInclusive;

#[pyclass]
#[derive(Debug, PartialEq, Eq, Copy, Clone, Hash)]
pub enum CollisionStatus {
    NoCollision(),
    CollidesStatic(),
    CollidesDynamic(TimeStepInner),
}

#[pymethods]
impl CollisionStatus {
    pub fn collides(&self) -> bool {
        match self {
            CollisionStatus::NoCollision() => false,
            CollisionStatus::CollidesStatic() | CollisionStatus::CollidesDynamic(_) => true,
        }
    }

    pub fn __str__(&self) -> String {
        format!("{}", self)
    }
}

impl Display for CollisionStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CollisionStatus::NoCollision() => write!(f, "NoCollision"),
            CollisionStatus::CollidesStatic() => write!(f, "CollidesStatic"),
            CollisionStatus::CollidesDynamic(t) => write!(f, "CollidesDynamic({})", t),
        }
    }
}

impl From<RustCollisionStatus> for CollisionStatus {
    fn from(value: RustCollisionStatus) -> Self {
        match value {
            RustCollisionStatus::NoCollision => CollisionStatus::NoCollision(),
            RustCollisionStatus::CollidesStatic => CollisionStatus::CollidesStatic(),
            RustCollisionStatus::CollidesDynamic(t) => CollisionStatus::CollidesDynamic(t.0),
        }
    }
}

#[pyclass]
pub struct CollisionChecker(RustCollisionChecker<RustCollisionObject>);

impl AsRef<RustCollisionChecker<RustCollisionObject>> for CollisionChecker {
    fn as_ref(&self) -> &RustCollisionChecker<RustCollisionObject> {
        &self.0
    }
}

#[pymethods]
impl CollisionChecker {
    #[pyo3(signature = (static_obstacle, position=None, min_time=None, max_time=None))]
    pub fn collides_static(
        &self,
        static_obstacle: &CollisionObject,
        position: Option<&Pose>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<CollisionStatus> {
        let position = match position {
            Some(position) => position.0,
            None => DPose2::IDENTITY,
        };
        let res = self.0.collides_static_range(
            static_obstacle.as_ref(),
            position,
            min_max_to_range(min_time, max_time),
        )?;
        Ok(res.into())
    }

    #[pyo3(signature = (dynamic_obstacle, min_time=None, max_time=None))]
    pub fn collides_dynamic(
        &self,
        dynamic_obstacle: &DynamicObstacle,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<CollisionStatus> {
        let res = self.0.collides_dynamic_range(
            dynamic_obstacle.as_ref(),
            min_max_to_range(min_time, max_time),
        )?;
        Ok(res.into())
    }
}

fn min_max_to_range(
    min_time: Option<TimeStepInner>,
    max_time: Option<TimeStepInner>,
) -> RangeInclusive<TimeStep> {
    match (min_time, max_time) {
        (Some(min_t), Some(max_t)) => TimeStep::from(min_t)..=TimeStep::from(max_t),
        (Some(t), None) | (None, Some(t)) => TimeStep::from(t)..=TimeStep::from(t),
        (None, None) => TimeStep::MIN..=TimeStep::MAX,
    }
}

impl From<CollisionCheckerError> for PyErr {
    fn from(value: CollisionCheckerError) -> Self {
        match value {
            CollisionCheckerError::Unsupported => {
                PyValueError::new_err("Unsupported shape combination")
            }
        }
    }
}

#[pyclass]
pub struct CollisionCheckerBuilder(pub(crate) RustCollisionCheckerBuilder);

#[pymethods]
impl CollisionCheckerBuilder {
    #[new]
    pub fn new() -> Self {
        CollisionCheckerBuilder(RustCollisionCheckerBuilder::new())
    }

    pub fn with_static_obstacle(&mut self, collision_object: &CollisionObject) {
        replace_with(&mut self.0, Default::default, |builder| {
            builder.with_static_obstacle(collision_object.as_ref().clone())
        });
    }

    pub fn with_dynamic_obstacle(&mut self, dynamic_obstacle: &DynamicObstacle) {
        replace_with(&mut self.0, Default::default, |builder| {
            builder.with_dynamic_obstacle(dynamic_obstacle.as_ref().clone())
        });
    }

    pub fn with_road_boundary_obstacle(&mut self, lanelets: Vec<Vec<(f64, f64)>>) {
        let lanelets = lanelets
            .into_iter()
            .map(|exterior| Polygon::new(exterior.into(), vec![]))
            .collect_vec();
        replace_with(&mut self.0, Default::default, |builder| {
            builder.with_road_boundary_obstacle(&lanelets)
        });
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
