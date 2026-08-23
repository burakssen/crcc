use crate::python::collision_checker::{CollisionChecker, CollisionStatus, min_max_to_range};
use crate::python::collision_object::CollisionObject;
use crate::python::pose::Pose;
use crate::time::TimeStepInner;
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
#[pyfunction(signature = (checker, positioned_query_shapes, threads, min_time = None, max_time = None))]
/// Benchmark-only helper: runs a static batch inside a freshly constructed
/// Rayon pool with the requested number of workers.
///
/// Pool construction time is included in the measurement by callers. This
/// function exists for benchmarking only and is not part of the public API.
pub fn collides_static_batch_fresh_pool(
    py: Python<'_>,
    checker: &CollisionChecker,
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
        .map_err(|error| PyRuntimeError::new_err(error.to_string()))?;

    let time_range = min_max_to_range(min_time, max_time)?;

    py.detach(|| {
        pool.install(|| {
            checker
                .as_ref()
                .collides_static_batch(&positioned_query_shapes, time_range, true)
                .into_iter()
                .map(|result| result.map(CollisionStatus::from))
                .collect::<Result<Vec<_>, _>>()
        })
        .map_err(Into::into)
    })
}

#[pymodule]
pub(super) mod benchmark {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::collides_static_batch_fresh_pool;

    /// Hack: workaround for <https://github.com/PyO3/pyo3/issues/759>.
    #[pymodule_init]
    fn init(m: &Bound<'_, PyModule>) -> PyResult<()> {
        Python::attach(|py| {
            py.import("sys")?
                .getattr("modules")?
                .set_item("crcc._core.benchmark", m)
        })
    }
}
