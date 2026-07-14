use crate::collision_checker::SelectedCollisionChecker as RustCollisionChecker;
pub use crate::collision_checker::engine::CollisionEngine;
use crate::collision_checker::{
    CollisionCheckerBuilder as RustCollisionCheckerBuilder, CollisionStatus as RustCollisionStatus,
};
use crate::python::collision_object::CollisionObject;
use crate::python::dynamic_obstacle::DynamicObstacle;
use crate::python::pose::Pose;
use crate::time::{TimeStep, TimeStepInner};
use glamx::DPose2;
use pyo3::prelude::*;
use replace_with::replace_with;
use std::fmt::Display;
use std::ops::RangeInclusive;

/// The outcome of a checker query.
///
/// `collides` is true for static and dynamic collisions. `time_step` is set only
/// for a dynamic collision and contains the first colliding step.
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

/// An immutable scene containing merged static geometry and dynamic trajectories.
///
/// Construct instances with CollisionCheckerBuilder. Time bounds are inclusive;
/// omitted bounds leave that side unbounded.
#[pyclass]
pub struct CollisionChecker(RustCollisionChecker);

impl AsRef<RustCollisionChecker> for CollisionChecker {
    fn as_ref(&self) -> &RustCollisionChecker {
        &self.0
    }
}

#[pymethods]
impl CollisionChecker {
    #[getter]
    /// The runtime collision backend used by this checker.
    pub fn engine(&self) -> CollisionEngine {
        self.0.engine()
    }

    #[pyo3(signature = (query_shape, position=None, min_time=None, max_time=None))]
    /// Checks a fixed shape against the scene.
    ///
    /// Raises ValueError when min_time exceeds max_time or an operation is
    /// unsupported.
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
            min_max_to_range(min_time, max_time)?,
        )?;
        Ok(res.into())
    }

    #[pyo3(signature = (positioned_query_shapes, min_time=None, max_time=None))]
    /// Checks positioned fixed shapes and returns statuses in input order.
    pub fn collides_static_batch(
        &self,
        py: Python<'_>,
        positioned_query_shapes: Vec<(CollisionObject, Pose)>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<Vec<CollisionStatus>> {
        let positioned_query_shapes = positioned_query_shapes
            .into_iter()
            .map(|(obstacle, position)| (obstacle.as_ref().clone(), position.0))
            .collect::<Vec<_>>();
        let time_range = min_max_to_range(min_time, max_time)?;
        let res = py.detach(|| {
            self.0
                .collides_static_batch(&positioned_query_shapes, time_range)
                .into_iter()
                .map(|result| result.map(CollisionStatus::from))
                .collect::<Result<Vec<_>, _>>()
        })?;
        Ok(res)
    }

    #[pyo3(signature = (positioned_query_shapes, min_time=None, max_time=None))]
    pub fn par_static(
        &self,
        py: Python<'_>,
        positioned_query_shapes: Vec<(CollisionObject, Pose)>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<Vec<CollisionStatus>> {
        self.collides_static_batch(py, positioned_query_shapes, min_time, max_time)
    }

    #[pyo3(signature = (positioned_query_shapes, threads, min_time=None, max_time=None))]
    pub fn _collides_static_batch_threads(
        &self,
        py: Python<'_>,
        positioned_query_shapes: Vec<(CollisionObject, Pose)>,
        threads: usize,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<Vec<CollisionStatus>> {
        let positioned_query_shapes = positioned_query_shapes
            .into_iter()
            .map(|(obstacle, position)| (obstacle.as_ref().clone(), position.0))
            .collect::<Vec<_>>();
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads.max(1))
            .build()
            .map_err(|error| pyo3::exceptions::PyRuntimeError::new_err(error.to_string()))?;
        let time_range = min_max_to_range(min_time, max_time)?;
        Ok(py.detach(|| {
            pool.install(|| {
                self.0
                    .collides_static_batch(&positioned_query_shapes, time_range)
                    .into_iter()
                    .map(|result| result.map(CollisionStatus::from))
                    .collect::<Result<Vec<_>, _>>()
            })
        })?)
    }

    #[pyo3(signature = (positioned_query_shapes, threads, min_time=None, max_time=None))]
    pub fn par_static_threads(
        &self,
        py: Python<'_>,
        positioned_query_shapes: Vec<(CollisionObject, Pose)>,
        threads: usize,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<Vec<CollisionStatus>> {
        self._collides_static_batch_threads(
            py,
            positioned_query_shapes,
            threads,
            min_time,
            max_time,
        )
    }

    #[pyo3(signature = (dynamic_obstacle, min_time=None, max_time=None))]
    /// Checks a moving obstacle against the scene over an inclusive time window.
    pub fn collides_dynamic(
        &self,
        dynamic_obstacle: &DynamicObstacle,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<CollisionStatus> {
        let res = self.0.collides_dynamic_range(
            dynamic_obstacle.as_ref(),
            min_max_to_range(min_time, max_time)?,
        )?;
        Ok(res.into())
    }

    #[pyo3(signature = (dynamic_obstacles, min_time=None, max_time=None))]
    /// Checks moving obstacles and returns statuses in input order.
    pub fn collides_dynamic_batch(
        &self,
        py: Python<'_>,
        dynamic_obstacles: Vec<DynamicObstacle>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<Vec<CollisionStatus>> {
        let dynamic_obstacles = dynamic_obstacles
            .into_iter()
            .map(|obstacle| obstacle.as_ref().clone())
            .collect::<Vec<_>>();
        let time_range = min_max_to_range(min_time, max_time)?;
        let res = py.detach(|| {
            self.0
                .collides_dynamic_batch(&dynamic_obstacles, time_range)
                .into_iter()
                .map(|result| result.map(CollisionStatus::from))
                .collect::<Result<Vec<_>, _>>()
        })?;
        Ok(res)
    }

    #[pyo3(signature = (dynamic_obstacles, min_time=None, max_time=None))]
    pub fn par_dynamic(
        &self,
        py: Python<'_>,
        dynamic_obstacles: Vec<DynamicObstacle>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<Vec<CollisionStatus>> {
        self.collides_dynamic_batch(py, dynamic_obstacles, min_time, max_time)
    }
}

fn min_max_to_range(
    min_time: Option<TimeStepInner>,
    max_time: Option<TimeStepInner>,
) -> PyResult<RangeInclusive<TimeStep>> {
    if min_time.zip(max_time).is_some_and(|(min, max)| min > max) {
        return Err(pyo3::exceptions::PyValueError::new_err(
            "min_time must not exceed max_time",
        ));
    }
    Ok(match (min_time, max_time) {
        (Some(min_t), Some(max_t)) => TimeStep::from(min_t)..=TimeStep::from(max_t),
        (Some(min_t), None) => TimeStep::from(min_t)..=TimeStep::MAX,
        (None, Some(max_t)) => TimeStep::MIN..=TimeStep::from(max_t),
        (None, None) => TimeStep::MIN..=TimeStep::MAX,
    })
}

/// A fluent builder for an immutable CollisionChecker.
///
/// The constructor defaults to Parry in Python. Builder methods mutate and
/// return the same builder so calls may be chained. `build()` raises ValueError
/// if the selected engine is unavailable.
#[pyclass]
pub struct CollisionCheckerBuilder {
    pub(crate) builder: RustCollisionCheckerBuilder,
    engine: CollisionEngine,
}

#[pymethods]
impl CollisionCheckerBuilder {
    #[new]
    #[pyo3(signature = (engine = None))]
    /// Creates an empty builder, optionally selecting an engine.
    pub fn new(engine: Option<CollisionEngine>) -> Self {
        Self {
            builder: RustCollisionCheckerBuilder::new(),
            engine: engine.unwrap_or_default(),
        }
    }

    /// Selects the backend used by the checker.
    pub fn with_engine<'a>(
        mut slf: PyRefMut<'a, Self>,
        engine: CollisionEngine,
    ) -> PyRefMut<'a, Self> {
        slf.engine = engine;
        slf
    }

    /// Adds geometry to the merged static scene.
    pub fn with_static_obstacle<'a>(
        mut slf: PyRefMut<'a, Self>,
        collision_object: &CollisionObject,
    ) -> PyRefMut<'a, Self> {
        replace_with(&mut slf.builder, Default::default, |builder| {
            builder.with_static_obstacle(collision_object.as_ref().clone())
        });
        slf
    }

    /// Adds a dynamic obstacle trajectory.
    pub fn with_dynamic_obstacle<'a>(
        mut slf: PyRefMut<'a, Self>,
        dynamic_obstacle: &DynamicObstacle,
    ) -> PyRefMut<'a, Self> {
        replace_with(&mut slf.builder, Default::default, |builder| {
            builder.with_dynamic_obstacle(dynamic_obstacle.as_ref().clone())
        });
        slf
    }

    /// Builds an immutable checker.
    pub fn with_road_boundary<'a>(
        mut slf: PyRefMut<'a, Self>,
        lanelets: Vec<Vec<(f64, f64)>>,
    ) -> PyRefMut<'a, Self> {
        let lanelets = lanelets
            .into_iter()
            .map(|exterior| geo::Polygon::new(exterior.into(), vec![]))
            .collect::<Vec<_>>();
        replace_with(&mut slf.builder, Default::default, |builder| {
            builder.with_road_boundary(&lanelets)
        });
        slf
    }

    #[pyo3(signature = (engine = None))]
    pub fn build(&self, engine: Option<CollisionEngine>) -> PyResult<CollisionChecker> {
        Ok(CollisionChecker(
            self.builder
                .clone()
                .build_with_engine(engine.unwrap_or(self.engine))?,
        ))
    }
}

impl Default for CollisionCheckerBuilder {
    fn default() -> Self {
        Self::new(None)
    }
}

#[pyfunction]
pub fn road_boundary(lanelets: Vec<Vec<(f64, f64)>>) -> PyResult<CollisionObject> {
    let lanelets = lanelets
        .into_iter()
        .map(|exterior| geo::Polygon::new(exterior.into(), vec![]))
        .collect::<Vec<_>>();
    Ok(CollisionObject::from(
        crate::collision_checker::road_boundary(&lanelets),
    ))
}

#[pymodule]
pub(super) mod collision_checker {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::{
        CollisionChecker, CollisionCheckerBuilder, CollisionEngine, CollisionStatus, road_boundary,
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
