pub use crate::collision_checker::engine::CollisionEngine;
use crate::collision_checker::{
    CollisionCheckerBuilder as RustCollisionCheckerBuilder, CollisionStatus as RustCollisionStatus,
    DynamicBatchQuery as RustDynamicBatchQuery, PreparedDynamicQuery as RustPreparedDynamicQuery,
    PreparedStaticQuery as RustPreparedStaticQuery,
    SelectedCollisionChecker as RustCollisionChecker, StaticBatchQuery as RustStaticBatchQuery,
};
use crate::collision_object::{
    CollisionObject as RustCollisionObject, DynamicObstacle as RustDynamicObstacle,
};
use crate::python::collision_object::CollisionObject;
use crate::python::dynamic_obstacle::DynamicObstacle;
use crate::python::pose::Pose;
use crate::time::{TimeStep, TimeStepInner};
use glamx::DPose2;
use pyo3::exceptions::{PyDeprecationWarning, PyTypeError, PyValueError};
use pyo3::prelude::*;
use replace_with::replace_with;
use std::ffi::CStr;
use std::fmt::Display;
use std::ops::RangeInclusive;
use std::sync::Arc;

/// Emits a `DeprecationWarning` pointing at the calling Python frame.
pub(super) fn warn_deprecated(py: Python<'_>, message: &CStr) -> PyResult<()> {
    PyErr::warn(py, &py.get_type::<PyDeprecationWarning>(), message, 2)
}

const ENGINE_PROPERTY_MESSAGE: &CStr = c"engine is deprecated; use backend instead";

/// The outcome of a checker query.
///
/// `collides` is true for static and dynamic collisions. `time_step` is set only
/// for a dynamic collision and contains the first colliding step.
#[pyclass]
#[derive(Debug, PartialEq, Eq, Clone, Hash)]
pub enum CollisionStatus {
    NoCollision(),
    CollidesStatic(),
    CollidesDynamic(TimeStepInner),
}

#[pymethods]
impl CollisionStatus {
    #[getter]
    pub const fn collides(&self) -> bool {
        match self {
            Self::NoCollision() => false,
            Self::CollidesStatic() | Self::CollidesDynamic(_) => true,
        }
    }

    #[getter]
    pub const fn time_step(&self) -> Option<TimeStepInner> {
        match self {
            Self::CollidesDynamic(time_step) => Some(*time_step),
            Self::NoCollision() | Self::CollidesStatic() => None,
        }
    }

    pub fn __str__(&self) -> String {
        format!("{self}")
    }

    pub fn __repr__(&self) -> String {
        self.__str__()
    }
}

impl Display for CollisionStatus {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoCollision() => write!(formatter, "NoCollision"),
            Self::CollidesStatic() => write!(formatter, "CollidesStatic"),
            Self::CollidesDynamic(time_step) => {
                write!(formatter, "CollidesDynamic({time_step})")
            }
        }
    }
}

impl From<RustCollisionStatus> for CollisionStatus {
    fn from(value: RustCollisionStatus) -> Self {
        match value {
            RustCollisionStatus::NoCollision => Self::NoCollision(),
            RustCollisionStatus::CollidesStatic => Self::CollidesStatic(),
            RustCollisionStatus::CollidesDynamic(time_step) => Self::CollidesDynamic(time_step.0),
        }
    }
}

/// Fixed geometry converted for repeated queries against one backend.
#[pyclass]
pub struct PreparedStaticQuery(Arc<RustPreparedStaticQuery>);

#[pymethods]
impl PreparedStaticQuery {
    #[getter]
    pub fn backend(&self) -> CollisionEngine {
        self.0.engine()
    }

    #[getter]
    /// Deprecated alias for [`Self::backend`].
    pub fn engine(&self, py: Python<'_>) -> PyResult<CollisionEngine> {
        warn_deprecated(py, ENGINE_PROPERTY_MESSAGE)?;
        Ok(self.backend())
    }
}

/// A dynamic trajectory converted for repeated queries against one backend.
#[pyclass]
pub struct PreparedDynamicQuery(Arc<RustPreparedDynamicQuery>);

#[pymethods]
impl PreparedDynamicQuery {
    #[getter]
    pub fn backend(&self) -> CollisionEngine {
        self.0.engine()
    }

    #[getter]
    /// Deprecated alias for [`Self::backend`].
    pub fn engine(&self, py: Python<'_>) -> PyResult<CollisionEngine> {
        warn_deprecated(py, ENGINE_PROPERTY_MESSAGE)?;
        Ok(self.backend())
    }
}

/// One classified entry of a static batch.
enum StaticSource {
    Raw(RustCollisionObject),
    Prepared(Arc<RustPreparedStaticQuery>),
}

/// One classified entry of a dynamic batch.
enum DynamicSource {
    Raw(RustDynamicObstacle),
    Prepared(Arc<RustPreparedDynamicQuery>),
}

/// How a parsed batch maps onto the optimized execution paths.
enum StaticBatchPlan {
    Raw(Vec<(RustCollisionObject, DPose2)>),
    Prepared(Arc<RustPreparedStaticQuery>, Vec<DPose2>),
    Heterogeneous(Vec<(StaticSource, DPose2)>),
}

/// How a parsed dynamic batch maps onto the optimized execution paths.
enum DynamicBatchPlan {
    Raw(Vec<RustDynamicObstacle>),
    Prepared(Vec<RustPreparedDynamicQuery>),
    Heterogeneous(Vec<DynamicSource>),
}

/// An immutable scene containing merged static geometry and dynamic trajectories.
///
/// Construct instances with `CollisionCheckerBuilder`. Time bounds are
/// inclusive; omitted bounds leave that side unbounded.
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
    /// The collision backend used by this checker.
    pub const fn backend(&self) -> CollisionEngine {
        self.0.engine()
    }

    #[getter]
    /// Deprecated alias for [`Self::backend`].
    pub fn engine(&self, py: Python<'_>) -> PyResult<CollisionEngine> {
        warn_deprecated(py, ENGINE_PROPERTY_MESSAGE)?;
        Ok(self.backend())
    }

    /// Converts fixed geometry once for repeated queries.
    pub fn prepare_static(&self, query_shape: &CollisionObject) -> PyResult<PreparedStaticQuery> {
        Ok(PreparedStaticQuery(Arc::new(
            self.0.prepare_static(query_shape.as_ref())?,
        )))
    }

    /// Converts a dynamic trajectory once for repeated queries.
    pub fn prepare_dynamic(
        &self,
        dynamic_obstacle: &DynamicObstacle,
    ) -> PyResult<PreparedDynamicQuery> {
        Ok(PreparedDynamicQuery(Arc::new(
            self.0.prepare_dynamic(dynamic_obstacle.as_ref())?,
        )))
    }

    #[pyo3(signature = (query, position = None, min_time = None, max_time = None))]
    /// Checks a fixed shape or prepared geometry against the scene.
    ///
    /// Raises `ValueError` when `min_time` exceeds `max_time` or an operation is
    /// unsupported, and `TypeError` when `query` has neither accepted type.
    #[allow(clippy::option_if_let_else)]
    pub fn collides_static(
        &self,
        query: &Bound<'_, PyAny>,
        position: Option<&Pose>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<CollisionStatus> {
        let position = position.map_or(DPose2::IDENTITY, |position| position.0);
        let time_range = min_max_to_range(min_time, max_time)?;

        if let Ok(raw) = query.extract::<PyRef<'_, CollisionObject>>() {
            return Ok(self
                .0
                .collides_static_range(raw.as_ref(), position, time_range)?
                .into());
        }
        if let Ok(prepared) = query.extract::<PyRef<'_, PreparedStaticQuery>>() {
            return Ok(self
                .0
                .collides_static_prepared_range(&prepared.0, position, time_range)?
                .into());
        }
        Err(PyTypeError::new_err(
            "query must be a CollisionObject or PreparedStaticQuery",
        ))
    }

    #[pyo3(signature = (queries, min_time = None, max_time = None, parallel = false))]
    /// Checks positioned fixed shapes and returns statuses in input order.
    ///
    /// Entries may mix raw objects with prepared geometry. Set `parallel=True`
    /// to execute the batch on Rayon's active pool.
    ///
    /// # Errors
    ///
    /// Returns a Python exception when the time range is invalid, an entry has
    /// an unsupported type or backend, or a collision query fails.
    #[allow(clippy::option_if_let_else)]
    pub fn collides_static_batch(
        &self,
        py: Python<'_>,
        queries: Vec<(Bound<'_, PyAny>, Pose)>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
        parallel: bool,
    ) -> PyResult<Vec<CollisionStatus>> {
        let time_range = min_max_to_range(min_time, max_time)?;
        let sources = queries
            .into_iter()
            .map(|(query, position)| {
                if let Ok(raw) = query.extract::<PyRef<'_, CollisionObject>>() {
                    Ok((StaticSource::Raw(raw.as_ref().clone()), position.0))
                } else if let Ok(prepared) = query.extract::<PyRef<'_, PreparedStaticQuery>>() {
                    Ok((StaticSource::Prepared(Arc::clone(&prepared.0)), position.0))
                } else {
                    Err(PyTypeError::new_err(
                        "each query must be a CollisionObject or PreparedStaticQuery",
                    ))
                }
            })
            .collect::<PyResult<Vec<_>>>()?;

        run_static_batch(&self.0, py, sources, time_range, parallel)
    }

    #[pyo3(signature = (queries, min_time=None, max_time=None, parallel = false))]
    /// Checks moving obstacles or prepared trajectories over an inclusive time
    /// window and returns statuses in input order.
    ///
    /// Entries may mix raw obstacles with prepared trajectories. Set
    /// `parallel=True` to execute the batch on Rayon's active pool.
    ///
    /// # Errors
    ///
    /// Returns a Python exception when the time range is invalid, an entry has
    /// an unsupported type or backend, or a collision query fails.
    #[allow(clippy::option_if_let_else)]
    pub fn collides_dynamic_batch(
        &self,
        py: Python<'_>,
        queries: Vec<Bound<'_, PyAny>>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
        parallel: bool,
    ) -> PyResult<Vec<CollisionStatus>> {
        let time_range = min_max_to_range(min_time, max_time)?;
        let sources = queries
            .into_iter()
            .map(|query| {
                if let Ok(raw) = query.extract::<PyRef<'_, DynamicObstacle>>() {
                    Ok(DynamicSource::Raw(raw.as_ref().clone()))
                } else if let Ok(prepared) = query.extract::<PyRef<'_, PreparedDynamicQuery>>() {
                    Ok(DynamicSource::Prepared(Arc::clone(&prepared.0)))
                } else {
                    Err(PyTypeError::new_err(
                        "each query must be a DynamicObstacle or PreparedDynamicQuery",
                    ))
                }
            })
            .collect::<PyResult<Vec<_>>>()?;

        run_dynamic_batch(&self.0, py, sources, time_range, parallel)
    }

    #[pyo3(signature = (query, position = None, min_time = None, max_time = None))]
    /// Deprecated: pass the prepared query to [`Self::collides_static`].
    pub fn collides_static_prepared(
        &self,
        py: Python<'_>,
        query: &PreparedStaticQuery,
        position: Option<&Pose>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<CollisionStatus> {
        warn_deprecated(
            py,
            c"collides_static_prepared() is deprecated; pass the prepared query to collides_static()",
        )?;
        let position = position.map_or(DPose2::IDENTITY, |position| position.0);
        Ok(self
            .0
            .collides_static_prepared_range(
                &query.0,
                position,
                min_max_to_range(min_time, max_time)?,
            )?
            .into())
    }

    #[pyo3(signature = (query, positions, min_time = None, max_time = None, parallel = false))]
    /// Deprecated: use `collides_static_batch([(query, pose), ...])`.
    #[allow(clippy::needless_pass_by_value)]
    pub fn collides_static_prepared_batch(
        &self,
        py: Python<'_>,
        query: &PreparedStaticQuery,
        positions: Vec<Pose>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
        parallel: bool,
    ) -> PyResult<Vec<CollisionStatus>> {
        warn_deprecated(
            py,
            c"collides_static_prepared_batch() is deprecated; use collides_static_batch([(query, pose), ...])",
        )?;
        let time_range = min_max_to_range(min_time, max_time)?;
        let sources = positions
            .into_iter()
            .map(|position| (StaticSource::Prepared(Arc::clone(&query.0)), position.0))
            .collect::<Vec<_>>();
        run_static_batch(&self.0, py, sources, time_range, parallel)
    }

    #[pyo3(signature = (query, min_time=None, max_time=None))]
    /// Deprecated: pass the prepared trajectory to [`Self::collides_dynamic`].
    pub fn collides_dynamic_prepared(
        &self,
        py: Python<'_>,
        query: &PreparedDynamicQuery,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<CollisionStatus> {
        warn_deprecated(
            py,
            c"collides_dynamic_prepared() is deprecated; pass the prepared trajectory to collides_dynamic()",
        )?;
        Ok(self
            .0
            .collides_dynamic_prepared_range(&query.0, min_max_to_range(min_time, max_time)?)?
            .into())
    }

    #[pyo3(signature = (queries, min_time = None, max_time = None, parallel = false))]
    /// Deprecated: use `collides_dynamic_batch([...])`.
    #[allow(clippy::needless_pass_by_value)]
    pub fn collides_dynamic_prepared_batch(
        &self,
        py: Python<'_>,
        queries: Vec<Py<PreparedDynamicQuery>>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
        parallel: bool,
    ) -> PyResult<Vec<CollisionStatus>> {
        warn_deprecated(
            py,
            c"collides_dynamic_prepared_batch() is deprecated; use collides_dynamic_batch([...])",
        )?;
        let time_range = min_max_to_range(min_time, max_time)?;
        let sources = queries
            .iter()
            .map(|query| DynamicSource::Prepared(Arc::clone(&query.bind(py).borrow().0)))
            .collect::<Vec<_>>();
        run_dynamic_batch(&self.0, py, sources, time_range, parallel)
    }

    #[pyo3(signature = (query, min_time=None, max_time=None))]
    /// Checks a moving obstacle against the scene over an inclusive time window.
    #[allow(clippy::option_if_let_else)]
    pub fn collides_dynamic(
        &self,
        query: &Bound<'_, PyAny>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<CollisionStatus> {
        let time_range = min_max_to_range(min_time, max_time)?;

        if let Ok(raw) = query.extract::<PyRef<'_, DynamicObstacle>>() {
            return Ok(self
                .0
                .collides_dynamic_range(raw.as_ref(), time_range)?
                .into());
        }
        if let Ok(prepared) = query.extract::<PyRef<'_, PreparedDynamicQuery>>() {
            return Ok(self
                .0
                .collides_dynamic_prepared_range(&prepared.0, time_range)?
                .into());
        }
        Err(PyTypeError::new_err(
            "query must be a DynamicObstacle or PreparedDynamicQuery",
        ))
    }

    #[pyo3(signature = (positioned_query_shapes, min_time = None, max_time = None))]
    /// Deprecated: use [`Self::collides_static_batch`] with `parallel=True`.
    pub fn par_static(
        &self,
        py: Python<'_>,
        positioned_query_shapes: Vec<(CollisionObject, Pose)>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<Vec<CollisionStatus>> {
        warn_deprecated(
            py,
            c"par_static() is deprecated; use collides_static_batch(..., parallel=True)",
        )?;
        let time_range = min_max_to_range(min_time, max_time)?;
        let sources = positioned_query_shapes
            .into_iter()
            .map(|(obstacle, position)| (StaticSource::Raw(obstacle.as_ref().clone()), position.0))
            .collect::<Vec<_>>();
        run_static_batch(&self.0, py, sources, time_range, true)
    }

    #[pyo3(signature = (dynamic_obstacles, min_time=None, max_time=None))]
    /// Deprecated: use [`Self::collides_dynamic_batch`] with `parallel=True`.
    pub fn par_dynamic(
        &self,
        py: Python<'_>,
        dynamic_obstacles: Vec<DynamicObstacle>,
        min_time: Option<TimeStepInner>,
        max_time: Option<TimeStepInner>,
    ) -> PyResult<Vec<CollisionStatus>> {
        warn_deprecated(
            py,
            c"par_dynamic() is deprecated; use collides_dynamic_batch(..., parallel=True)",
        )?;
        let time_range = min_max_to_range(min_time, max_time)?;
        let sources = dynamic_obstacles
            .into_iter()
            .map(|obstacle| DynamicSource::Raw(obstacle.as_ref().clone()))
            .collect::<Vec<_>>();
        run_dynamic_batch(&self.0, py, sources, time_range, true)
    }
}

/// Executes a classified static batch on the selected backend.
fn run_static_batch(
    checker: &RustCollisionChecker,
    py: Python<'_>,
    sources: Vec<(StaticSource, DPose2)>,
    time_range: RangeInclusive<TimeStep>,
    parallel: bool,
) -> PyResult<Vec<CollisionStatus>> {
    if sources.is_empty() {
        return Ok(Vec::new());
    }

    let mut has_raw = false;
    let mut has_prepared = false;
    let mut uniform: Option<Arc<RustPreparedStaticQuery>> = None;
    let mut multiple_prepared = false;
    for (source, _) in &sources {
        match source {
            StaticSource::Raw(_) => has_raw = true,
            StaticSource::Prepared(prepared) => {
                has_prepared = true;
                match &uniform {
                    None => uniform = Some(Arc::clone(prepared)),
                    Some(first) if !Arc::ptr_eq(first, prepared) => multiple_prepared = true,
                    _ => {}
                }
            }
        }
    }

    let plan = if has_raw && has_prepared || multiple_prepared {
        StaticBatchPlan::Heterogeneous(sources)
    } else if has_prepared {
        let positions = sources.iter().map(|(_, position)| *position).collect();
        #[allow(clippy::option_if_let_else)]
        uniform.map_or(StaticBatchPlan::Heterogeneous(sources), |query| {
            StaticBatchPlan::Prepared(query, positions)
        })
    } else {
        StaticBatchPlan::Raw(
            sources
                .into_iter()
                .filter_map(|(source, position)| match source {
                    StaticSource::Raw(object) => Some((object, position)),
                    StaticSource::Prepared(_) => None,
                })
                .collect(),
        )
    };

    let results: Vec<crate::collision_checker::CollisionResult> = py.detach(move || match plan {
        StaticBatchPlan::Raw(positioned) => {
            checker.collides_static_batch(&positioned, time_range.clone(), parallel)
        }
        StaticBatchPlan::Prepared(query, positions) => {
            checker.collides_static_prepared_batch(&query, &positions, time_range.clone(), parallel)
        }
        StaticBatchPlan::Heterogeneous(entries) => {
            let references: Vec<(RustStaticBatchQuery<'_>, DPose2)> = entries
                .iter()
                .map(|(source, position)| {
                    let query = match source {
                        StaticSource::Raw(object) => RustStaticBatchQuery::Raw(object),
                        StaticSource::Prepared(prepared) => {
                            RustStaticBatchQuery::Prepared(prepared)
                        }
                    };
                    (query, *position)
                })
                .collect();
            checker.collides_static_heterogeneous_batch(references, time_range, parallel)
        }
    });

    let statuses = results
        .into_iter()
        .map(|result| result.map(CollisionStatus::from))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(statuses)
}

/// Executes a classified dynamic batch on the selected backend.
fn run_dynamic_batch(
    checker: &RustCollisionChecker,
    py: Python<'_>,
    sources: Vec<DynamicSource>,
    time_range: RangeInclusive<TimeStep>,
    parallel: bool,
) -> PyResult<Vec<CollisionStatus>> {
    if sources.is_empty() {
        return Ok(Vec::new());
    }

    let mut has_raw = false;
    let mut has_prepared = false;
    for source in &sources {
        match source {
            DynamicSource::Raw(_) => has_raw = true,
            DynamicSource::Prepared(_) => has_prepared = true,
        }
    }

    let plan = if has_raw && has_prepared {
        DynamicBatchPlan::Heterogeneous(sources)
    } else if has_prepared {
        DynamicBatchPlan::Prepared(
            sources
                .into_iter()
                .filter_map(|source| match source {
                    DynamicSource::Prepared(prepared) => Some((*prepared).clone()),
                    DynamicSource::Raw(_) => None,
                })
                .collect(),
        )
    } else {
        DynamicBatchPlan::Raw(
            sources
                .into_iter()
                .filter_map(|source| match source {
                    DynamicSource::Raw(obstacle) => Some(obstacle),
                    DynamicSource::Prepared(_) => None,
                })
                .collect(),
        )
    };

    let results: Vec<crate::collision_checker::CollisionResult> = py.detach(move || match plan {
        DynamicBatchPlan::Raw(obstacles) => {
            checker.collides_dynamic_batch(&obstacles, time_range.clone(), parallel)
        }
        DynamicBatchPlan::Prepared(queries) => {
            checker.collides_dynamic_prepared_batch(&queries, time_range.clone(), parallel)
        }
        DynamicBatchPlan::Heterogeneous(entries) => {
            let references: Vec<RustDynamicBatchQuery<'_>> = entries
                .iter()
                .map(|source| match source {
                    DynamicSource::Raw(obstacle) => RustDynamicBatchQuery::Raw(obstacle),
                    DynamicSource::Prepared(prepared) => RustDynamicBatchQuery::Prepared(prepared),
                })
                .collect();
            checker.collides_dynamic_heterogeneous_batch(references, time_range, parallel)
        }
    });

    let statuses = results
        .into_iter()
        .map(|result| result.map(CollisionStatus::from))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(statuses)
}

pub(super) fn min_max_to_range(
    min_time: Option<TimeStepInner>,
    max_time: Option<TimeStepInner>,
) -> PyResult<RangeInclusive<TimeStep>> {
    if min_time.zip(max_time).is_some_and(|(min, max)| min > max) {
        return Err(PyValueError::new_err("min_time must not exceed max_time"));
    }
    Ok(match (min_time, max_time) {
        (Some(min_t), Some(max_t)) => TimeStep::from(min_t)..=TimeStep::from(max_t),
        (Some(min_t), None) => TimeStep::from(min_t)..=TimeStep::MAX,
        (None, Some(max_t)) => TimeStep::MIN..=TimeStep::from(max_t),
        (None, None) => TimeStep::MIN..=TimeStep::MAX,
    })
}

/// A fluent builder for an immutable `CollisionChecker`.
///
/// The constructor defaults to `Parry` in Python. Builder methods mutate and
/// return the same builder so calls may be chained. `build()` raises
/// `ValueError` if the selected engine is unavailable.
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
    pub fn with_engine(mut slf: PyRefMut<'_, Self>, engine: CollisionEngine) -> PyRefMut<'_, Self> {
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

    /// Adds geometry representing the space outside the supplied lanelets.
    pub fn with_road_boundary(
        mut slf: PyRefMut<'_, Self>,
        lanelets: Vec<Vec<(f64, f64)>>,
    ) -> PyRefMut<'_, Self> {
        let lanelets = lanelets
            .into_iter()
            .map(|exterior| geo::Polygon::new(exterior.into(), Vec::new()))
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
pub fn road_boundary(lanelets: Vec<Vec<(f64, f64)>>) -> CollisionObject {
    let lanelets = lanelets
        .into_iter()
        .map(|exterior| geo::Polygon::new(exterior.into(), Vec::new()))
        .collect::<Vec<_>>();

    CollisionObject::from(crate::collision_checker::road_boundary(&lanelets))
}

#[pymodule]
pub(super) mod collision_checker {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::{
        CollisionChecker, CollisionCheckerBuilder, CollisionEngine, CollisionStatus,
        PreparedDynamicQuery, PreparedStaticQuery, road_boundary,
    };

    /// Hack: workaround for <https://github.com/PyO3/pyo3/issues/759>.
    #[pymodule_init]
    fn init(m: &Bound<'_, PyModule>) -> PyResult<()> {
        Python::attach(|py| {
            py.import("sys")?
                .getattr("modules")?
                .set_item("crcc._core.collision_checker", m)
        })
    }
}
