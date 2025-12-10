use geo::Polygon as GeoPolygon;
use itertools::Itertools;
use nalgebra::{Point2, Vector2};
use parry2d_f64::{
    query::{Unsupported, intersection_test},
    shape::{Ball, Cuboid, SharedShape, Triangle as ParryTriangle},
};
use pyo3::{exceptions::PyValueError, prelude::*};

use crate::{polygon::PolygonCollisionObject, python::isometry::Isometry};

#[pyclass(subclass)]
pub struct Shape(pub(crate) SharedShape);

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
            &pos_self.unwrap_or(&Isometry::identity()).0,
            self.0.as_ref(),
            &pos_other.unwrap_or(&Isometry::identity()).0,
            other.0.as_ref(),
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
        (Circle {}, Shape(SharedShape::new(shape)))
    }
}

#[pyclass(extends = Shape)]
pub struct Rectangle {}

#[pymethods]
impl Rectangle {
    #[new]
    fn new(width: f64, height: f64) -> (Self, Shape) {
        let shape = Cuboid::new(Vector2::new(width / 2.0, height / 2.0));
        (Rectangle {}, Shape(SharedShape::new(shape)))
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
        (Triangle {}, Shape(SharedShape::new(shape)))
    }
}

#[pyclass(extends = Shape)]
pub struct Polygon {}

#[pymethods]
impl Polygon {
    #[new]
    fn new(exterior: Vec<(f64, f64)>, interiors: Vec<Vec<(f64, f64)>>) -> PyResult<(Self, Shape)> {
        let poly = GeoPolygon::new(exterior.into(), interiors.into_iter().map_into().collect());
        let shape = PolygonCollisionObject::new(&poly).ok_or(PyValueError::new_err(
            "Cannot create collision shape from empty polygon",
        ))?;
        Ok((Polygon {}, Shape(shape.into_shared())))
    }
}

#[pymodule]
pub(super) mod collision_object {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::{Circle, Polygon, Rectangle, Shape, Triangle};

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
