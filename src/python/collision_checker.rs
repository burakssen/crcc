use crate::collision_checker::SelectedCollisionChecker;
pub use crate::collision_checker::engine::CollisionEngine;
use crate::collision_checker::{
    CollisionCheckerBuilder as RustCollisionCheckerBuilder, CollisionStatus as RustCollisionStatus,
};
use crate::python::collision_object::CollisionObject;
use crate::python::dynamic_obstacle::DynamicObstacle;
use crate::python::pose::Pose;
use crate::time::{TimeStep, TimeStepInner};
use geo::Polygon;
use glamx::DPose2;
use itertools::Itertools;
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
    #[getter]
    pub fn collides(&self) -> bool {
        match self {
            CollisionStatus::NoCollision() => false,
            CollisionStatus::CollidesStatic() | CollisionStatus::CollidesDynamic(_) => true,
        }
    }

    #[getter]
    pub fn time_step(&self) -> Option<TimeStepInner> {
        match self {
            CollisionStatus::CollidesDynamic(t) => Some(*t),
            CollisionStatus::NoCollision() | CollisionStatus::CollidesStatic() => None,
        }
    }

    pub fn __str__(&self) -> String {
        format!("{}", self)
    }

    pub fn __repr__(&self) -> String {
        self.__str__()
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
pub struct CollisionChecker(SelectedCollisionChecker);

impl AsRef<SelectedCollisionChecker> for CollisionChecker {
    fn as_ref(&self) -> &SelectedCollisionChecker {
        &self.0
    }
}

#[pymethods]
impl CollisionChecker {
    #[pyo3(signature = (query_shape, position=None, min_time=None, max_time=None))]
    pub fn collides_static(
        &self,
        query_shape: &CollisionObject,
        position: Option<&Pose>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<CollisionStatus> {
        let position = match position {
            Some(position) => position.0,
            None => DPose2::IDENTITY,
        };
        let res = self.0.collides_static_range(
            query_shape.as_ref(),
            position,
            min_max_to_range(min_time, max_time),
        )?;
        Ok(res.into())
    }

    #[pyo3(signature = (positioned_query_shapes, min_time=None, max_time=None))]
    pub fn par_collides_static(
        &self,
        positioned_query_shapes: Vec<(CollisionObject, Pose)>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<Vec<CollisionStatus>> {
        let positioned_query_shapes = positioned_query_shapes
            .into_iter()
            .map(|(obstacle, position)| (obstacle.as_ref().clone(), position.0))
            .collect::<Vec<_>>();
        let res = self
            .0
            .par_collides_static(
                &positioned_query_shapes,
                min_max_to_range(min_time, max_time),
            )
            .into_iter()
            .map(|result| result.map(CollisionStatus::from))
            .collect::<Result<_, _>>()?;
        Ok(res)
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

    #[pyo3(signature = (dynamic_obstacles, min_time=None, max_time=None))]
    pub fn par_collides_dynamic(
        &self,
        dynamic_obstacles: Vec<DynamicObstacle>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<Vec<CollisionStatus>> {
        let dynamic_obstacles = dynamic_obstacles
            .into_iter()
            .map(|obstacle| obstacle.as_ref().clone())
            .collect::<Vec<_>>();
        let res = self
            .0
            .par_collides_dynamic(&dynamic_obstacles, min_max_to_range(min_time, max_time))
            .into_iter()
            .map(|result| result.map(CollisionStatus::from))
            .collect::<Result<_, _>>()?;
        Ok(res)
    }
}

fn min_max_to_range(
    min_time: Option<TimeStepInner>,
    max_time: Option<TimeStepInner>,
) -> RangeInclusive<TimeStep> {
    match (min_time, max_time) {
        (Some(min_t), Some(max_t)) => TimeStep::from(min_t)..=TimeStep::from(max_t),
        (Some(min_t), None) => TimeStep::from(min_t)..=TimeStep::MAX,
        (None, Some(max_t)) => TimeStep::MIN..=TimeStep::from(max_t),
        (None, None) => TimeStep::MIN..=TimeStep::MAX,
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

    pub fn with_static_obstacle<'a>(
        mut slf: PyRefMut<'a, Self>,
        collision_object: &CollisionObject,
    ) -> PyRefMut<'a, Self> {
        replace_with(&mut slf.0, Default::default, |builder| {
            builder.with_static_obstacle(collision_object.as_ref().clone())
        });
        slf
    }

    pub fn with_dynamic_obstacle<'a>(
        mut slf: PyRefMut<'a, Self>,
        dynamic_obstacle: &DynamicObstacle,
    ) -> PyRefMut<'a, Self> {
        replace_with(&mut slf.0, Default::default, |builder| {
            builder.with_dynamic_obstacle(dynamic_obstacle.as_ref().clone())
        });
        slf
    }

    pub fn with_road_boundary_obstacle(
        mut slf: PyRefMut<Self>,
        lanelets: Vec<Vec<(f64, f64)>>,
    ) -> PyRefMut<Self> {
        let lanelets = lanelets
            .into_iter()
            .map(|exterior| Polygon::new(exterior.into(), vec![]))
            .collect_vec();
        replace_with(&mut slf.0, Default::default, |builder| {
            builder.with_road_boundary_obstacle(&lanelets)
        });
        slf
    }

    #[pyo3(signature = (engine = None))]
    pub fn build(&self, engine: Option<CollisionEngine>) -> PyResult<CollisionChecker> {
        Ok(CollisionChecker(
            self.0
                .clone()
                .build_with_engine(engine.unwrap_or_default())?,
        ))
    }
}

impl Default for CollisionCheckerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[pyfunction]
pub fn create_road_boundary_obstacle(lanelets: Vec<Vec<(f64, f64)>>) -> PyResult<CollisionObject> {
    let lanelets = lanelets
        .into_iter()
        .map(|exterior| geo::Polygon::new(exterior.into(), vec![]))
        .collect::<Vec<_>>();
    Ok(CollisionObject::from(crate::collision_checker::create_road_boundary_obstacle(&lanelets)))
}

#[pymodule]
pub(super) mod collision_checker {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::{
        CollisionChecker, CollisionCheckerBuilder, CollisionEngine, CollisionStatus,
        create_road_boundary_obstacle,
    };

    /// Hack: workaround for https://github.com/PyO3/pyo3/issues/759
    #[pymodule_init]
    fn init(m: &Bound<'_, PyModule>) -> PyResult<()> {
        Python::attach(|py| {
            py.import("sys")?
                .getattr("modules")?
                .set_item("crcc._core.collision_checker", m)
        })
    }
}
