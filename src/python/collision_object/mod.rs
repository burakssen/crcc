use crate::collision_object::CollisionObject as RustCollisionObject;
use crate::python::collision_object::simple::SimpleCollisionObject;
use pyo3::prelude::*;

mod simple;

#[pyclass]
pub struct CollisionObject(pub(in crate::python) RustCollisionObject);

#[pymethods]
impl CollisionObject {
    #[new]
    fn new(simple_collision_objects: Vec<SimpleCollisionObject>) -> Self {
        // Note that this slices (https://en.wikipedia.org/wiki/Object_slicing) all subclass info
        // from the simple collision objects.
        // Right now, this is not a problem because all information is stored in the superclass
        Self(RustCollisionObject(
            simple_collision_objects
                .into_iter()
                .map(|obj| obj.0)
                .collect(),
        ))
    }
}

#[pymodule]
pub(super) mod collision_object {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::CollisionObject;
    #[pymodule_export]
    use super::simple::{
        Circle, Empty, HalfSpace, Polygon, Rectangle, SimpleCollisionObject, Triangle,
    };

    /// Hack: workaround for https://github.com/PyO3/pyo3/issues/759
    #[pymodule_init]
    fn init(m: &Bound<'_, PyModule>) -> PyResult<()> {
        Python::attach(|py| {
            py.import("sys")?
                .getattr("modules")?
                .set_item("commonroad_collision_checker._core.collision_object", m)
        })
    }
}
