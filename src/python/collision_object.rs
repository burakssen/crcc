use std::sync::LazyLock;

use geo::Polygon as GeoPolygon;
use itertools::Itertools;
use nalgebra::{Point2, Vector2};
use parry2d_f64::{
    query::{Unsupported, intersection_test},
    shape::{Ball, Cuboid, Shape as ParryShape, Triangle as ParryTriangle},
};
use pyo3::{exceptions::PyValueError, prelude::*};

use crate::{polygon::polygon_to_collision_shape, python::isometry::Isometry};

static IDENTITY: LazyLock<Isometry> = LazyLock::new(Isometry::identity);

#[pyclass(subclass)]
pub struct Shape {
    inner: Box<dyn ParryShape>,
}

#[pymethods]
impl Shape {
    #[pyo3(signature = (other, pos_self=None, pos_other=None))]
    fn collides(
        &self,
        other: &Shape,
        pos_self: Option<&Isometry>,
        pos_other: Option<&Isometry>,
    ) -> PyResult<bool> {
        intersection_test(
            &pos_self.unwrap_or(&IDENTITY).inner,
            self.inner.as_ref(),
            &pos_other.unwrap_or(&IDENTITY).inner,
            other.inner.as_ref(),
        )
        .map_err(|err| match err {
            Unsupported => PyValueError::new_err("Unsupported shape combination"),
        })
    }
}

#[pyclass(extends = Shape)]
pub struct Circle {}

#[pymethods]
impl Circle {
    #[new]
    fn new(radius: f64) -> (Self, Shape) {
        let shape = Ball::new(radius);
        (
            Circle {},
            Shape {
                inner: Box::new(shape),
            },
        )
    }
}

#[pyclass(extends = Shape)]
pub struct Rectangle {}

#[pymethods]
impl Rectangle {
    #[new]
    fn new(width: f64, height: f64) -> (Self, Shape) {
        let shape = Cuboid::new(Vector2::new(width / 2.0, height / 2.0));
        (
            Rectangle {},
            Shape {
                inner: Box::new(shape),
            },
        )
    }
}

#[pyclass(extends = Shape)]
pub struct Triangle {}

#[pymethods]
impl Triangle {
    #[new]
    fn new(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> (Self, Shape) {
        let shape = ParryTriangle::new(
            Point2::new(a.0, a.1),
            Point2::new(b.0, b.1),
            Point2::new(c.0, c.1),
        );
        (
            Triangle {},
            Shape {
                inner: Box::new(shape),
            },
        )
    }
}

#[pyclass(extends = Shape)]
pub struct Polygon {}

#[pymethods]
impl Polygon {
    #[new]
    fn new(exterior: Vec<(f64, f64)>, interiors: Vec<Vec<(f64, f64)>>) -> PyResult<(Self, Shape)> {
        let poly = GeoPolygon::new(exterior.into(), interiors.into_iter().map_into().collect());
        let shape = polygon_to_collision_shape(&poly).ok_or(PyValueError::new_err(
            "Cannot create collision shape from empty polygon",
        ))?;
        Ok((Polygon {}, Shape { inner: shape }))
    }
}

#[pymodule]
pub(super) mod collision_object {
    #[pymodule_export]
    use super::Circle;
    #[pymodule_export]
    use super::Polygon;
    #[pymodule_export]
    use super::Rectangle;
    #[pymodule_export]
    use super::Shape;
    #[pymodule_export]
    use super::Triangle;
}
