use crate::collision_object::simple::SimpleCollisionObject as RustSimpleCollisionObject;
use geo::{Polygon as GeoPolygon, Rect, Triangle as GeoTriangle, coord};
use itertools::Itertools;
use nalgebra::{Unit, Vector2};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

#[pyclass(subclass)]
#[derive(Clone)]
pub struct SimpleCollisionObject(pub(in crate::python) RustSimpleCollisionObject);

#[pyclass(extends = SimpleCollisionObject)]
pub struct Circle {}

#[pymethods]
impl Circle {
    #[new]
    #[pyo3(signature = (radius, center = (0.0, 0.0)))]
    fn new(radius: f64, center: (f64, f64)) -> PyResult<(Self, SimpleCollisionObject)> {
        if radius <= 0.0 {
            return Err(PyValueError::new_err("Radius must be positive"));
        }
        Ok((
            Self {},
            SimpleCollisionObject(RustSimpleCollisionObject::circle(center, radius)),
        ))
    }
}

#[pyclass(extends = SimpleCollisionObject)]
pub struct Empty {}

#[pymethods]
impl Empty {
    #[new]
    fn new() -> (Self, SimpleCollisionObject) {
        (
            Self {},
            SimpleCollisionObject(RustSimpleCollisionObject::empty()),
        )
    }
}

#[pyclass(extends = SimpleCollisionObject)]
pub struct HalfSpace {}

#[pymethods]
impl HalfSpace {
    #[new]
    #[pyo3(signature = (outward_normal, offset = 0.0))]
    fn new(outward_normal: (f64, f64), offset: f64) -> (Self, SimpleCollisionObject) {
        let normalized = Unit::new_normalize(Vector2::new(outward_normal.0, outward_normal.1));
        (
            Self {},
            SimpleCollisionObject(RustSimpleCollisionObject::half_space(normalized, offset)),
        )
    }

    #[staticmethod]
    fn from_points(py: Python<'_>, p1: (f64, f64), p2: (f64, f64)) -> PyResult<Py<PyAny>> {
        let init = PyClassInitializer::from(SimpleCollisionObject(
            RustSimpleCollisionObject::half_space_from_points(p1, p2),
        ))
        .add_subclass(Self {});
        Ok(Py::new(py, init)?.into_any())
    }

    #[staticmethod]
    #[pyo3(signature = (a, b, c = 0.0))]
    fn from_coeffs(py: Python<'_>, a: f64, b: f64, c: f64) -> PyResult<Py<PyAny>> {
        let init = PyClassInitializer::from(SimpleCollisionObject(
            RustSimpleCollisionObject::half_space_from_coeffs(a, b, c),
        ))
        .add_subclass(Self {});
        Ok(Py::new(py, init)?.into_any())
    }
}

#[pyclass(extends = SimpleCollisionObject)]
pub struct Polygon {}

#[pymethods]
impl Polygon {
    #[new]
    #[pyo3(signature = (exterior, interiors = None))]
    fn new(
        exterior: Vec<(f64, f64)>,
        interiors: Option<Vec<Vec<(f64, f64)>>>,
    ) -> PyResult<(Self, SimpleCollisionObject)> {
        let interiors = interiors.unwrap_or_default();
        let collision_object = RustSimpleCollisionObject::polygon(GeoPolygon::new(
            exterior.into(),
            interiors.into_iter().map_into().collect(),
        ));
        if matches!(collision_object, RustSimpleCollisionObject::Empty(_)) {
            return Err(PyValueError::new_err("Polygon must not be empty"));
        }
        Ok((Self {}, SimpleCollisionObject(collision_object)))
    }
}

#[pyclass(extends = SimpleCollisionObject)]
pub struct Rectangle {}

#[pymethods]
impl Rectangle {
    #[new]
    #[pyo3(signature = (length, width, center = (0.0, 0.0)))]
    fn new(length: f64, width: f64, center: (f64, f64)) -> PyResult<(Self, SimpleCollisionObject)> {
        if length <= 0.0 || width <= 0.0 {
            return Err(PyValueError::new_err("Length and width must be positive"));
        }
        let lower_left = coord! {x: center.0 - length / 2.0, y: center.1 - width / 2.0};
        let upper_right = coord! {x: center.0 + length / 2.0, y: center.1 + width / 2.0};
        Ok((
            Self {},
            SimpleCollisionObject(RustSimpleCollisionObject::rectangle(Rect::new(
                lower_left,
                upper_right,
            ))),
        ))
    }
}

#[pyclass(extends = SimpleCollisionObject)]
pub struct Triangle {}

#[pymethods]
impl Triangle {
    #[new]
    fn new(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> PyResult<(Self, SimpleCollisionObject)> {
        let collision_object =
            RustSimpleCollisionObject::triangle(GeoTriangle::new(a.into(), b.into(), c.into()));
        if matches!(collision_object, RustSimpleCollisionObject::Empty(..)) {
            return Err(PyValueError::new_err("Triangle must not be empty"));
        }
        Ok((Self {}, SimpleCollisionObject(collision_object)))
    }
}
