use crate::collision_object::CollisionObject as RustCollisionObject;
use crate::collision_object::simple::SimpleCollisionObject;
use geo::{Polygon as GeoPolygon, Rect, Triangle as GeoTriangle, coord};
use itertools::Itertools;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use std::sync::Arc;

#[pyclass(subclass)]
#[derive(Clone)]
pub struct CollisionObject(Arc<RustCollisionObject>);

impl From<RustCollisionObject> for CollisionObject {
    fn from(value: RustCollisionObject) -> Self {
        Self(Arc::new(value))
    }
}

impl From<SimpleCollisionObject> for CollisionObject {
    fn from(value: SimpleCollisionObject) -> Self {
        Self(Arc::new(value.into()))
    }
}

impl AsRef<RustCollisionObject> for CollisionObject {
    fn as_ref(&self) -> &RustCollisionObject {
        &self.0
    }
}

#[pyclass(extends = CollisionObject)]
pub struct Compound;

#[pymethods]
impl Compound {
    #[new]
    fn new(collision_objects: Vec<CollisionObject>) -> (Self, CollisionObject) {
        // Note that this slices (https://en.wikipedia.org/wiki/Object_slicing) all subclass info
        // from the simple collision objects.
        // Right now, this is not a problem because all information is stored in the superclass
        (
            Self,
            RustCollisionObject::merge_all(
                collision_objects
                    .into_iter()
                    .map(|obj| obj.as_ref().clone()),
            )
            .into(),
        )
    }
}

#[pyclass(extends = CollisionObject)]
pub struct Circle;

#[pymethods]
impl Circle {
    #[new]
    #[pyo3(signature = (radius, center = (0.0, 0.0)))]
    fn new(radius: f64, center: (f64, f64)) -> PyResult<(Self, CollisionObject)> {
        if radius <= 0.0 {
            return Err(PyValueError::new_err("Radius must be positive"));
        }
        Ok((Self, SimpleCollisionObject::circle(center, radius).into()))
    }
}

#[pyclass(extends = CollisionObject)]
pub struct Empty;

#[pymethods]
impl Empty {
    #[new]
    fn new() -> (Self, CollisionObject) {
        (Self, SimpleCollisionObject::empty().into())
    }
}

#[pyclass(extends = CollisionObject)]
pub struct HalfSpace;

#[pymethods]
impl HalfSpace {
    #[new]
    #[pyo3(signature = (outward_normal, offset = 0.0))]
    fn new(outward_normal: (f64, f64), offset: f64) -> (Self, CollisionObject) {
        (
            Self,
            SimpleCollisionObject::half_space(outward_normal, offset).into(),
        )
    }

    #[staticmethod]
    fn from_points(py: Python<'_>, p1: (f64, f64), p2: (f64, f64)) -> PyResult<Py<PyAny>> {
        let init = PyClassInitializer::from(CollisionObject::from(
            SimpleCollisionObject::half_space_from_points(p1, p2),
        ))
        .add_subclass(Self);
        Ok(Py::new(py, init)?.into_any())
    }

    #[staticmethod]
    #[pyo3(signature = (a, b, c = 0.0))]
    fn from_coeffs(py: Python<'_>, a: f64, b: f64, c: f64) -> PyResult<Py<PyAny>> {
        let init = PyClassInitializer::from(CollisionObject::from(
            SimpleCollisionObject::half_space_from_coeffs(a, b, c),
        ))
        .add_subclass(Self);
        Ok(Py::new(py, init)?.into_any())
    }
}

#[pyclass(extends = CollisionObject)]
pub struct Polygon;

#[pymethods]
impl Polygon {
    #[new]
    #[pyo3(signature = (exterior, interiors = None))]
    fn new(
        exterior: Vec<(f64, f64)>,
        interiors: Option<Vec<Vec<(f64, f64)>>>,
    ) -> PyResult<(Self, CollisionObject)> {
        let interiors = interiors.unwrap_or_default();
        let collision_object = SimpleCollisionObject::polygon(GeoPolygon::new(
            exterior.into(),
            interiors.into_iter().map_into().collect(),
        ));
        if matches!(collision_object, SimpleCollisionObject::Empty(_)) {
            return Err(PyValueError::new_err("Polygon must not be empty"));
        }
        Ok((Self, collision_object.into()))
    }
}

#[pyclass(extends = CollisionObject)]
pub struct Rectangle;

#[pymethods]
impl Rectangle {
    #[new]
    #[pyo3(signature = (length, width, orientation = 0.0, center = (0.0, 0.0)))]
    fn new(
        length: f64,
        width: f64,
        orientation: f64,
        center: (f64, f64),
    ) -> PyResult<(Self, CollisionObject)> {
        if length <= 0.0 || width <= 0.0 {
            return Err(PyValueError::new_err("Length and width must be positive"));
        }
        let lower_left = coord! {x: center.0 - length / 2.0, y: center.1 - width / 2.0};
        let upper_right = coord! {x: center.0 + length / 2.0, y: center.1 + width / 2.0};
        Ok((
            Self,
            SimpleCollisionObject::rectangle(Rect::new(lower_left, upper_right), orientation)
                .into(),
        ))
    }
}

#[pyclass(extends = CollisionObject)]
pub struct Triangle;

#[pymethods]
impl Triangle {
    #[new]
    fn new(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> PyResult<(Self, CollisionObject)> {
        let collision_object =
            SimpleCollisionObject::triangle(GeoTriangle::new(a.into(), b.into(), c.into()));
        if matches!(collision_object, SimpleCollisionObject::Empty(..)) {
            return Err(PyValueError::new_err("Triangle must not be empty"));
        }
        Ok((Self, collision_object.into()))
    }
}

#[pymodule]
pub(super) mod collision_object {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::{
        Circle, CollisionObject, Compound, Empty, HalfSpace, Polygon, Rectangle, Triangle,
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
