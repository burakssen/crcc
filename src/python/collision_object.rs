use crate::collision_checker::engine::CollisionEngine;
use crate::collision_object::CollisionObject as RustCollisionObject;
use crate::collision_object::simple::SimpleCollisionObject;
use crate::python::pose::Pose;
use geo::{Polygon as GeoPolygon, Rect, Triangle as GeoTriangle, coord};
use itertools::Itertools;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use std::ops::{Add, Div, Sub};
use std::sync::Arc;

/// Base class for all queryable 2D geometry.
///
/// Pair methods accept an optional `CollisionEngine` and raise `ValueError` for
/// invalid geometry or unsupported engine and operation combinations.
#[pyclass(subclass)]
#[derive(Clone)]
pub struct CollisionObject(Arc<RustCollisionObject>);

#[pymethods]
impl CollisionObject {
    #[pyo3(signature = (
        other,
        pos_self = Pose::identity(),
        pos_other = Pose::identity(),
        engine = CollisionEngine::Parry
    ))]
    /// Returns whether two objects overlap at the supplied poses.
    ///
    /// # Errors
    ///
    /// Returns a Python exception when the selected collision engine does not
    /// support the requested operation.
    pub fn collides(
        &self,
        other: &Self,
        pos_self: Pose,
        pos_other: Pose,
        engine: CollisionEngine,
    ) -> PyResult<bool> {
        Ok(crate::collision_checker::engine::collides(
            self.as_ref(),
            pos_self.0,
            other.as_ref(),
            pos_other.0,
            engine,
        )?)
    }

    #[pyo3(signature = (
        start_pos_self,
        end_pos_self,
        other,
        start_pos_other,
        end_pos_other,
        engine = CollisionEngine::Parry
    ))]
    /// Conservatively checks two motions over one continuous interval.
    ///
    /// `False` certifies separation; `True` may be a conservative positive.
    ///
    /// # Errors
    ///
    /// Returns a Python exception when the selected collision engine does not
    /// support continuous collision detection for the supplied objects.
    pub fn collides_continuous(
        &self,
        start_pos_self: Pose,
        end_pos_self: Pose,
        other: &Self,
        start_pos_other: Pose,
        end_pos_other: Pose,
        engine: CollisionEngine,
    ) -> PyResult<bool> {
        Ok(crate::collision_checker::engine::collides_continuous(
            self.as_ref(),
            start_pos_self.0,
            end_pos_self.0,
            other.as_ref(),
            start_pos_other.0,
            end_pos_other.0,
            engine,
        )?)
    }

    #[pyo3(signature = (
        other,
        pos_self = Pose::identity(),
        pos_other = Pose::identity(),
        engine = CollisionEngine::Parry
    ))]
    /// Returns the non-negative separation distance between two objects.
    ///
    /// # Errors
    ///
    /// Returns a Python exception when the selected collision engine does not
    /// support distance queries for the supplied objects.
    pub fn distance(
        &self,
        other: &Self,
        pos_self: Pose,
        pos_other: Pose,
        engine: CollisionEngine,
    ) -> PyResult<f64> {
        Ok(crate::collision_checker::engine::distance(
            self.as_ref(),
            pos_self.0,
            other.as_ref(),
            pos_other.0,
            engine,
        )?)
    }

    /// Returns a compound containing this object and `other`.
    #[must_use]
    pub fn merge(&self, other: &Self) -> Self {
        Self::from(RustCollisionObject::merge(
            self.as_ref().clone(),
            other.as_ref().clone(),
        ))
    }

    #[staticmethod]
    /// Merges an iterable of objects into one compound.
    #[must_use]
    pub fn merge_all(collision_objects: Vec<Self>) -> Self {
        Self::from(RustCollisionObject::merge_all(
            collision_objects
                .into_iter()
                .map(|object| object.as_ref().clone()),
        ))
    }
}

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

/// A union of zero or more `CollisionObject` children.
#[pyclass(extends = CollisionObject)]
pub struct Compound;

#[pymethods]
impl Compound {
    #[new]
    pub fn new(collision_objects: Vec<CollisionObject>) -> (Self, CollisionObject) {
        // This slices all Python subclass information from the simple collision
        // objects. All geometry data is currently stored in the superclass.
        (
            Self,
            RustCollisionObject::merge_all(
                collision_objects
                    .into_iter()
                    .map(|object| object.as_ref().clone()),
            )
            .into(),
        )
    }
}

/// A circle defined by a positive radius and optional local-space center.
#[pyclass(extends = CollisionObject)]
pub struct Circle;

#[pymethods]
impl Circle {
    #[new]
    #[pyo3(signature = (radius, center = (0.0, 0.0)))]
    /// Creates a circle.
    ///
    /// # Errors
    ///
    /// Returns `ValueError` when `radius` is not positive or the geometry is
    /// otherwise invalid.
    pub fn new(radius: f64, center: (f64, f64)) -> PyResult<(Self, CollisionObject)> {
        if radius <= 0.0 {
            return Err(PyValueError::new_err("Radius must be positive"));
        }

        Ok((Self, SimpleCollisionObject::circle(center, radius)?.into()))
    }
}

/// Geometry that never collides.
#[pyclass(extends = CollisionObject)]
pub struct Empty;

#[pymethods]
impl Empty {
    #[new]
    pub fn new() -> (Self, CollisionObject) {
        (Self, SimpleCollisionObject::empty().into())
    }
}

/// Geometry occupying the entire plane.
#[pyclass(extends = CollisionObject)]
pub struct FullSpace;

#[pymethods]
impl FullSpace {
    #[new]
    pub fn new() -> (Self, CollisionObject) {
        (Self, SimpleCollisionObject::full_space().into())
    }
}

/// The region `outward_normal dot point <= offset`.
#[pyclass(extends = CollisionObject)]
pub struct HalfSpace;

#[pymethods]
impl HalfSpace {
    #[new]
    #[pyo3(signature = (outward_normal, offset = 0.0))]
    /// Creates a half-space.
    ///
    /// # Errors
    ///
    /// Returns `ValueError` when the normal or offset does not define a valid
    /// half-space.
    pub fn new(outward_normal: (f64, f64), offset: f64) -> PyResult<(Self, CollisionObject)> {
        Ok((
            Self,
            SimpleCollisionObject::half_space(outward_normal, offset)?.into(),
        ))
    }

    #[staticmethod]
    /// Creates a half-space from two boundary points.
    ///
    /// # Errors
    ///
    /// Returns `ValueError` when the points do not define a valid boundary.
    pub fn from_points(
        python: Python<'_>,
        point_1: (f64, f64),
        point_2: (f64, f64),
    ) -> PyResult<Py<PyAny>> {
        let initializer = PyClassInitializer::from(CollisionObject::from(
            SimpleCollisionObject::half_space_from_points(point_1, point_2)?,
        ))
        .add_subclass(Self);

        Ok(Py::new(python, initializer)?.into_any())
    }

    #[staticmethod]
    #[pyo3(signature = (a, b, c = 0.0))]
    /// Creates a half-space from linear coefficients.
    ///
    /// # Errors
    ///
    /// Returns `ValueError` when the coefficients do not define a valid
    /// half-space.
    pub fn from_coeffs(python: Python<'_>, a: f64, b: f64, c: f64) -> PyResult<Py<PyAny>> {
        let initializer = PyClassInitializer::from(CollisionObject::from(
            SimpleCollisionObject::half_space_from_coeffs(a, b, c)?,
        ))
        .add_subclass(Self);

        Ok(Py::new(python, initializer)?.into_any())
    }
}

/// A polygon defined by an exterior ring and optional interior rings.
///
/// Rings are sequences of `(x, y)` pairs. Invalid or empty polygons raise
/// `ValueError`.
#[pyclass(extends = CollisionObject)]
pub struct Polygon;

#[pymethods]
impl Polygon {
    #[new]
    #[pyo3(signature = (exterior, interiors = None))]
    /// Creates a polygon.
    ///
    /// # Errors
    ///
    /// Returns `ValueError` when the supplied rings do not define a valid
    /// polygon.
    pub fn new(
        exterior: Vec<(f64, f64)>,
        interiors: Option<Vec<Vec<(f64, f64)>>>,
    ) -> PyResult<(Self, CollisionObject)> {
        let interiors = interiors.unwrap_or_default();

        let collision_object = SimpleCollisionObject::polygon(GeoPolygon::new(
            exterior.into(),
            interiors.into_iter().map_into().collect(),
        ))?;

        Ok((Self, collision_object.into()))
    }
}

/// An oriented rectangle defined by length, width, angle, and center.
#[pyclass(extends = CollisionObject)]
pub struct Rectangle;

#[pymethods]
impl Rectangle {
    #[new]
    #[pyo3(signature = (
        length,
        width,
        orientation = 0.0,
        center = (0.0, 0.0)
    ))]
    /// Creates an oriented rectangle.
    ///
    /// # Errors
    ///
    /// Returns `ValueError` when `length` or `width` is not positive or the
    /// resulting geometry is invalid.
    pub fn new(
        length: f64,
        width: f64,
        orientation: f64,
        center: (f64, f64),
    ) -> PyResult<(Self, CollisionObject)> {
        if length <= 0.0 || width <= 0.0 {
            return Err(PyValueError::new_err("Length and width must be positive"));
        }

        let half_length = length.div(2.0);
        let half_width = width.div(2.0);

        let lower_left = coord! {
            x: center.0.sub(half_length),
            y: center.1.sub(half_width),
        };

        let upper_right = coord! {
            x: center.0.add(half_length),
            y: center.1.add(half_width),
        };

        Ok((
            Self,
            SimpleCollisionObject::rectangle(Rect::new(lower_left, upper_right), orientation)?
                .into(),
        ))
    }
}

/// A triangle defined by three finite `(x, y)` vertices.
#[pyclass(extends = CollisionObject)]
pub struct Triangle;

#[pymethods]
impl Triangle {
    #[new]
    /// Creates a triangle.
    ///
    /// # Errors
    ///
    /// Returns `ValueError` when the points do not define a valid triangle.
    pub fn new(
        point_a: (f64, f64),
        point_b: (f64, f64),
        point_c: (f64, f64),
    ) -> PyResult<(Self, CollisionObject)> {
        let collision_object = SimpleCollisionObject::triangle(GeoTriangle::new(
            point_a.into(),
            point_b.into(),
            point_c.into(),
        ))?;

        Ok((Self, collision_object.into()))
    }
}

#[pymodule]
pub(super) mod collision_object {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::{
        Circle, CollisionObject, Compound, Empty, FullSpace, HalfSpace, Polygon, Rectangle,
        Triangle,
    };

    /// Hack: workaround for <https://github.com/PyO3/pyo3/issues/759>.
    #[pymodule_init]
    fn init(module: &Bound<'_, PyModule>) -> PyResult<()> {
        Python::attach(|python| {
            python
                .import("sys")?
                .getattr("modules")?
                .set_item("crcc._core.collision_object", module)
        })
    }
}
