use crate::error::{CrccError, CrccResult};
use enum_dispatch::enum_dispatch;
use geo::{
    AffineOps, AffineTransform, Area, BooleanOps, ConvexHull, HasDimensions, IsConvex, Polygon,
    Rect, Rotate, Triangle as GeoTriangle, Validation,
};
use glamx::{DPose2, DVec2};
use itertools::Itertools;
use std::ops::{Deref, Div, Mul, Sub};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
/// Geometry representing the empty set.
pub struct Empty;

impl Empty {
    /// Creates empty geometry.
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
/// Geometry representing the entire plane.
pub struct FullSpace;

impl FullSpace {
    /// Creates geometry representing the entire plane.
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

#[derive(Debug, Clone, PartialEq)]
/// Non-degenerate triangle geometry accepted by [`crate::CollisionObject::from`].
pub struct Triangle(pub(super) GeoTriangle);

impl Triangle {
    /// Creates a finite, non-empty triangle.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidGeometry`] when any coordinate is not
    /// finite, or [`CrccError::EmptyShape`] when the triangle is degenerate.
    pub fn new(triangle: GeoTriangle) -> CrccResult<Self> {
        if [triangle.0, triangle.1, triangle.2]
            .iter()
            .any(|coord| !coord.x.is_finite() || !coord.y.is_finite())
        {
            return Err(CrccError::InvalidGeometry(
                "triangle coordinates must be finite",
            ));
        }
        if triangle.is_empty() || triangle.unsigned_area() == 0.0 {
            return Err(CrccError::EmptyShape);
        }
        Ok(Self(triangle))
    }
}

impl Deref for Triangle {
    type Target = GeoTriangle;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Debug, Clone, PartialEq)]
/// A rectangle with a local orientation in radians.
pub struct Rectangle {
    pub(super) rect: Rect,
    pub(super) orientation: f64,
}

impl Rectangle {
    /// Creates a finite, non-empty oriented rectangle.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidGeometry`] when a coordinate or the
    /// orientation is not finite, or [`CrccError::EmptyShape`] when the
    /// rectangle is empty.
    pub fn new(rect: Rect, orientation: f64) -> CrccResult<Self> {
        if !rect.min().x.is_finite()
            || !rect.min().y.is_finite()
            || !rect.max().x.is_finite()
            || !rect.max().y.is_finite()
            || !orientation.is_finite()
        {
            return Err(CrccError::InvalidGeometry(
                "rectangle coordinates and orientation must be finite",
            ));
        }
        if rect.is_empty() {
            return Err(CrccError::EmptyShape);
        }
        let center = rect.center();
        if !rect.width().is_finite()
            || !rect.height().is_finite()
            || !center.x.is_finite()
            || !center.y.is_finite()
        {
            return Err(CrccError::InvalidGeometry(
                "rectangle dimensions and center must be finite",
            ));
        }
        Ok(Self { rect, orientation })
    }

    /// Returns the underlying axis-aligned local rectangle.
    #[must_use]
    pub const fn rect(&self) -> &Rect {
        &self.rect
    }

    /// Returns the local-space center.
    #[must_use]
    pub fn center(&self) -> (f64, f64) {
        self.rect.center().into()
    }

    /// Returns the local x extent.
    #[must_use]
    pub fn width(&self) -> f64 {
        self.rect.width()
    }

    /// Returns the local y extent.
    #[must_use]
    pub fn height(&self) -> f64 {
        self.rect.height()
    }

    /// Returns the local orientation in radians.
    #[must_use]
    pub const fn orientation(&self) -> f64 {
        self.orientation
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ConvexPolygon(pub(super) Polygon);

impl ConvexPolygon {
    /// Creates a convex polygon without interior rings.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::HasHoles`] when the polygon has interior rings, or
    /// [`CrccError::NotConvex`] when its exterior ring is not convex.
    pub fn new(polygon: Polygon) -> CrccResult<Self> {
        if !polygon.interiors().is_empty() {
            return Err(CrccError::HasHoles);
        }
        if !polygon.exterior().is_convex() {
            return Err(CrccError::NotConvex);
        }
        Ok(Self(polygon))
    }

    /// Returns the underlying polygon.
    #[must_use]
    pub const fn polygon(&self) -> &Polygon {
        &self.0
    }
}

impl Deref for ConvexPolygon {
    type Target = Polygon;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct NonConvexPolygon(pub(super) Polygon);

impl NonConvexPolygon {
    /// Creates a polygon without interior rings.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::HasHoles`] when the polygon has interior rings.
    pub fn new(polygon: Polygon) -> CrccResult<Self> {
        if !polygon.interiors().is_empty() {
            return Err(CrccError::HasHoles);
        }
        Ok(Self(polygon))
    }

    /// Returns the underlying polygon.
    #[must_use]
    pub const fn polygon(&self) -> &Polygon {
        &self.0
    }
}

impl Deref for NonConvexPolygon {
    type Target = Polygon;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PolygonWithHoles(pub(super) Polygon);

impl PolygonWithHoles {
    /// Creates a polygon that may contain interior rings.
    #[must_use]
    pub const fn new(polygon: Polygon) -> Self {
        Self(polygon)
    }
}

impl Deref for PolygonWithHoles {
    type Target = Polygon;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Debug, Clone, PartialEq)]
/// Circle geometry accepted by [`crate::CollisionObject::from`].
pub struct Circle {
    pub(super) center: (f64, f64),
    pub(super) radius: f64,
}

impl Circle {
    /// Creates a circle with a finite center and strictly positive radius.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidRadius`] when the center or radius is not
    /// finite, or when the radius is not strictly positive.
    pub fn new(center: (f64, f64), radius: f64) -> CrccResult<Self> {
        if !center.0.is_finite() || !center.1.is_finite() || !radius.is_finite() || radius <= 0.0 {
            return Err(CrccError::InvalidRadius(radius));
        }
        Ok(Self { center, radius })
    }

    /// Returns the local-space center.
    #[must_use]
    pub const fn center(&self) -> (f64, f64) {
        self.center
    }

    /// Returns the radius.
    #[must_use]
    pub const fn radius(&self) -> f64 {
        self.radius
    }
}

#[derive(Debug, Clone, PartialEq)]
/// The half-space `outward_normal · point <= offset`.
pub struct HalfSpace {
    /// The normalized outward normal.
    pub outward_normal: DVec2,
    /// The signed boundary offset along the normal.
    pub offset: f64,
}

impl HalfSpace {
    fn normalized(outward_normal: DVec2, offset: f64) -> CrccResult<Self> {
        let length = outward_normal.x.hypot(outward_normal.y);
        if !outward_normal.is_finite()
            || !offset.is_finite()
            || !length.is_finite()
            || length == 0.0
        {
            return Err(CrccError::InvalidGeometry(
                "half-space requires a finite nonzero normal and finite offset",
            ));
        }

        let outward_normal = outward_normal.div(length);
        let offset = offset / length;
        if !outward_normal.is_finite() || !offset.is_finite() {
            return Err(CrccError::InvalidGeometry(
                "half-space normalization must produce finite values",
            ));
        }

        Ok(Self {
            outward_normal,
            offset,
        })
    }

    /// Creates a half-space from a directed boundary line.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidGeometry`] when either point is not finite
    /// or when both points are equal.
    pub fn from_points(p1: impl Into<DVec2>, p2: impl Into<DVec2>) -> CrccResult<Self> {
        let p1 = p1.into();
        let p2 = p2.into();
        let dir = p2.sub(p1);
        if !p1.is_finite() || !p2.is_finite() || !dir.is_finite() {
            return Err(CrccError::InvalidGeometry(
                "half-space points must be finite and distinct",
            ));
        }
        let normal = DVec2::new(-dir.y, dir.x);
        Self::normalized(normal, normal.dot(p1))
    }

    /// Creates the half-space `a*x + b*y <= c`.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidGeometry`] when a coefficient is not finite
    /// or when `(a, b)` is the zero vector.
    pub fn from_coeffs(a: f64, b: f64, c: f64) -> CrccResult<Self> {
        Self::normalized(DVec2::new(a, b), c)
    }

    /// Compares two normalized half-spaces with the default tolerance.
    #[must_use]
    pub fn almost_equal(&self, other: &Self) -> bool {
        self.almost_equal_with_tol(other, 1e-9)
    }

    /// Compares two normalized half-spaces with `tol`.
    #[must_use]
    pub fn almost_equal_with_tol(&self, other: &Self, tol: f64) -> bool {
        self.outward_normal.sub(other.outward_normal).length().abs() < tol
            && (self.offset - other.offset).abs() < tol
    }
}

#[derive(Debug, Clone, PartialEq)]
#[enum_dispatch(SweptArea)]
pub enum SimpleCollisionObject {
    Empty(Empty),
    FullSpace(FullSpace),
    HalfSpace(HalfSpace),
    Circle(Circle),
    Rectangle(Rectangle),
    Triangle(Triangle),
    ConvexPolygon(ConvexPolygon),
    NonConvexPolygon(NonConvexPolygon),
    PolygonWithHoles(PolygonWithHoles),
}

impl SimpleCollisionObject {
    /// Creates an empty collision object.
    #[must_use]
    pub const fn empty() -> Self {
        Self::Empty(Empty)
    }

    /// Creates a collision object occupying the entire plane.
    #[must_use]
    pub const fn full_space() -> Self {
        Self::FullSpace(FullSpace)
    }

    /// Creates a half-space from a normal and offset.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidGeometry`] when the normal is zero or when
    /// the normal or offset is not finite.
    pub fn half_space(outward_normal: impl Into<DVec2>, offset: f64) -> CrccResult<Self> {
        Ok(Self::HalfSpace(HalfSpace::normalized(
            outward_normal.into(),
            offset,
        )?))
    }

    /// Creates a half-space from a directed boundary line.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidGeometry`] when either point is not finite
    /// or when both points are equal.
    pub fn half_space_from_points(p1: (f64, f64), p2: (f64, f64)) -> CrccResult<Self> {
        Ok(Self::HalfSpace(HalfSpace::from_points(p1, p2)?))
    }

    /// Creates the half-space `a*x + b*y <= c`.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidGeometry`] when a coefficient is not finite
    /// or when `(a, b)` is the zero vector.
    pub fn half_space_from_coeffs(a: f64, b: f64, c: f64) -> CrccResult<Self> {
        Ok(Self::HalfSpace(HalfSpace::from_coeffs(a, b, c)?))
    }

    /// Creates a circle.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidRadius`] when the center or radius is not
    /// finite, or when the radius is not strictly positive.
    pub fn circle(center: (f64, f64), radius: f64) -> CrccResult<Self> {
        Ok(Self::Circle(Circle::new(center, radius)?))
    }

    /// Creates an oriented rectangle.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidGeometry`] when a coordinate or orientation
    /// is not finite, or [`CrccError::EmptyShape`] when the rectangle is empty.
    pub fn rectangle(rect: impl Into<Rect>, orientation: f64) -> CrccResult<Self> {
        Ok(Self::Rectangle(Rectangle::new(rect.into(), orientation)?))
    }

    /// Creates a non-degenerate triangle.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidGeometry`] when a coordinate is not finite,
    /// or [`CrccError::EmptyShape`] when the triangle is degenerate.
    pub fn triangle(triangle: impl Into<GeoTriangle>) -> CrccResult<Self> {
        Ok(Self::Triangle(Triangle::new(triangle.into())?))
    }

    /// Creates a validated polygon collision object.
    ///
    /// # Errors
    ///
    /// Returns an error when the polygon contains non-finite coordinates, is
    /// topologically invalid, is empty, or cannot be classified as supported
    /// collision geometry.
    pub fn polygon(polygon: impl Into<Polygon>) -> CrccResult<Self> {
        let polygon = polygon.into();
        if polygon
            .exterior()
            .0
            .iter()
            .chain(polygon.interiors().iter().flat_map(|ring| ring.0.iter()))
            .any(|coord| !coord.x.is_finite() || !coord.y.is_finite())
        {
            return Err(CrccError::InvalidGeometry(
                "polygon coordinates must be finite",
            ));
        }
        if !polygon.is_valid() {
            return Err(CrccError::InvalidGeometry(
                "polygon must be topologically valid",
            ));
        }
        if polygon.is_empty() {
            return Err(CrccError::EmptyShape);
        }
        match (
            polygon.exterior().is_convex(),
            polygon.interiors().is_empty(),
        ) {
            (true, true) => Ok(Self::ConvexPolygon(ConvexPolygon::new(polygon)?)),
            (false, true) => Ok(Self::NonConvexPolygon(NonConvexPolygon::new(polygon)?)),
            _ => Ok(Self::PolygonWithHoles(PolygonWithHoles(polygon))),
        }
    }

    /// Returns whether this object represents the empty set.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        matches!(self, Self::Empty(_))
    }

    /// Returns whether this object represents the entire plane.
    #[must_use]
    pub const fn is_full_space(&self) -> bool {
        matches!(self, Self::FullSpace(_))
    }
}

#[enum_dispatch]
pub trait SweptArea {
    /// Overapproximates the area the object covers while moving through the given positions.
    ///
    /// The returned vector has length `positions.len() - 1`, with each entry
    /// corresponding to the swept area between two consecutive positions.
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject>;

    #[cfg(test)]
    fn swept_area(&self, start_pos: DPose2, end_pos: DPose2) -> Option<SimpleCollisionObject> {
        self.swept_areas(&[start_pos, end_pos]).pop()
    }
}
impl SweptArea for Empty {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        vec![SimpleCollisionObject::empty(); positions.len().saturating_sub(1)]
    }
}

impl SweptArea for FullSpace {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        vec![SimpleCollisionObject::full_space(); positions.len().saturating_sub(1)]
    }
}

impl SweptArea for Triangle {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        swept_areas(&self.0, positions)
    }
}

impl SweptArea for Rectangle {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        let mut poly = Polygon::from(self.rect);
        poly.rotate_around_center_mut(self.orientation.to_degrees());
        swept_areas(&poly, positions)
    }
}

impl SweptArea for ConvexPolygon {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        swept_areas(&self.0, positions)
    }
}

impl SweptArea for NonConvexPolygon {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        swept_areas(&self.0, positions)
    }
}

impl SweptArea for PolygonWithHoles {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        swept_areas(&self.0, positions)
    }
}

fn conservative_result(result: CrccResult<SimpleCollisionObject>) -> SimpleCollisionObject {
    result.unwrap_or_else(|_| SimpleCollisionObject::full_space())
}

impl SweptArea for Circle {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        let mut swept_areas = Vec::with_capacity(positions.len().saturating_sub(1));
        for (start_pos, end_pos) in positions.iter().tuple_windows() {
            if rotation_changed(*start_pos, *end_pos) && self.center != (0.0, 0.0) {
                let bound_radius = DVec2::from(self.center).length() + self.radius;
                swept_areas.push(conservative_result(SimpleCollisionObject::rectangle(
                    Rect::new(
                        (
                            start_pos.translation.x.min(end_pos.translation.x) - bound_radius,
                            start_pos.translation.y.min(end_pos.translation.y) - bound_radius,
                        ),
                        (
                            start_pos.translation.x.max(end_pos.translation.x) + bound_radius,
                            start_pos.translation.y.max(end_pos.translation.y) + bound_radius,
                        ),
                    ),
                    0.0,
                )));
                continue;
            }
            let center = DVec2::from(self.center);
            let start = (*start_pos).mul(center);
            let end = (*end_pos).mul(center);
            swept_areas.push(conservative_result(SimpleCollisionObject::rectangle(
                Rect::new(
                    (
                        start.x.min(end.x) - self.radius,
                        start.y.min(end.y) - self.radius,
                    ),
                    (
                        start.x.max(end.x) + self.radius,
                        start.y.max(end.y) + self.radius,
                    ),
                ),
                0.0,
            )));
        }
        swept_areas
    }
}

impl SweptArea for HalfSpace {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        let mut swept_areas = Vec::with_capacity(positions.len().saturating_sub(1));
        for (start_pos, end_pos) in positions.iter().tuple_windows() {
            if rotation_changed(*start_pos, *end_pos) {
                // Any half-space rotation needs an unbounded conservative sweep.
                swept_areas.push(SimpleCollisionObject::full_space());
            } else {
                let outward_normal = start_pos.rotation.mul(self.outward_normal);
                let start_offset = outward_normal.dot(start_pos.translation);
                let end_offset = outward_normal.dot(end_pos.translation);
                let offset = start_offset.max(end_offset) + self.offset;
                swept_areas.push(conservative_result(SimpleCollisionObject::half_space(
                    outward_normal,
                    offset,
                )));
            }
        }
        swept_areas
    }
}

/// Common helper function for the `swept_areas` implementations
fn swept_areas(
    shape: &(impl AffineOps<f64> + Clone + Into<Polygon>),
    positions: &[DPose2],
) -> Vec<SimpleCollisionObject> {
    let transformed_shapes = positions
        .iter()
        .copied()
        .map(pose_to_affine)
        .map(|affine| shape.affine_transform(&affine))
        .map_into()
        .collect_vec();
    positions
        .iter()
        .tuple_windows()
        .zip(transformed_shapes.iter().tuple_windows())
        .map(|((start_pose, end_pose), (start, end))| {
            if rotation_changed(*start_pose, *end_pose) {
                rotational_swept_bound(shape, *start_pose, *end_pose)
            } else {
                conservative_result(SimpleCollisionObject::polygon(
                    start.union(end).convex_hull(),
                ))
            }
        })
        .collect()
}

fn rotational_swept_bound(
    shape: &(impl AffineOps<f64> + Clone + Into<Polygon>),
    start: DPose2,
    end: DPose2,
) -> SimpleCollisionObject {
    let polygon: Polygon = shape.clone().into();
    let radius = polygon
        .exterior()
        .0
        .iter()
        .chain(polygon.interiors().iter().flat_map(|ring| ring.0.iter()))
        .map(|coord| coord.x.hypot(coord.y))
        .fold(0.0, f64::max);
    conservative_result(SimpleCollisionObject::rectangle(
        Rect::new(
            (
                start.translation.x.min(end.translation.x) - radius,
                start.translation.y.min(end.translation.y) - radius,
            ),
            (
                start.translation.x.max(end.translation.x) + radius,
                start.translation.y.max(end.translation.y) + radius,
            ),
        ),
        0.0,
    ))
}

pub(crate) fn rotation_changed(start: DPose2, end: DPose2) -> bool {
    start.rotation != end.rotation
}

pub(crate) fn pose_to_affine(pose: DPose2) -> AffineTransform {
    AffineTransform::rotate(pose.rotation.angle().to_degrees(), (0.0, 0.0))
        .translated(pose.translation.x, pose.translation.y)
}
#[cfg(test)]
#[allow(clippy::panic_in_result_fn)]
mod tests {
    use super::*;
    use geo::BoundingRect;
    use glamx::approx::assert_relative_eq;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn pose_to_affine_matches_pose_transform() {
        let pose = DPose2::new(DVec2::new(1.0, 2.0), FRAC_PI_2);
        let affine = pose_to_affine(pose);
        let point = (1.0, 2.0);

        let pose_point = pose.mul(DVec2::from(point));
        let affine_point = affine.apply(point.into());

        assert_relative_eq!(pose_point.x, affine_point.x);
        assert_relative_eq!(pose_point.y, affine_point.y);
    }

    #[test]
    fn half_space_scales_offset_with_normal() {
        let direct = SimpleCollisionObject::half_space((2.0, 0.0), 2.0);
        let coefficients = SimpleCollisionObject::half_space_from_coeffs(2.0, 0.0, 2.0);

        assert_eq!(direct, coefficients);
    }

    #[test]
    fn half_space_rejects_unrepresentable_normalization() {
        assert_eq!(
            SimpleCollisionObject::half_space((f64::MAX, f64::MAX), 1.0),
            Err(CrccError::InvalidGeometry(
                "half-space requires a finite nonzero normal and finite offset",
            )),
        );
        assert_eq!(
            SimpleCollisionObject::half_space_from_points((-f64::MAX, 0.0), (f64::MAX, 0.0),),
            Err(CrccError::InvalidGeometry(
                "half-space points must be finite and distinct",
            )),
        );
    }

    #[test]
    fn rectangle_rejects_non_finite_derived_dimensions() {
        assert_eq!(
            SimpleCollisionObject::rectangle(Rect::new((-f64::MAX, -1.0), (f64::MAX, 1.0)), 0.0,),
            Err(CrccError::InvalidGeometry(
                "rectangle dimensions and center must be finite",
            )),
        );
    }

    #[test]
    fn constructors_report_exact_degenerate_errors() {
        assert_eq!(
            SimpleCollisionObject::circle((0.0, 0.0), 0.0),
            Err(CrccError::InvalidRadius(0.0)),
        );
        assert_eq!(
            SimpleCollisionObject::triangle(GeoTriangle::new(
                (0.0, 0.0).into(),
                (1.0, 1.0).into(),
                (2.0, 2.0).into(),
            )),
            Err(CrccError::EmptyShape),
        );
    }

    #[test]
    fn rotating_finite_shape_uses_exact_radial_extrema() -> CrccResult<()> {
        let rectangle =
            SimpleCollisionObject::rectangle(Rect::new((-10.0, -0.1), (10.0, 0.1)), 0.0)?;
        let sweep = rectangle.swept_area(DPose2::IDENTITY, DPose2::rotation(FRAC_PI_2));
        let Some(SimpleCollisionObject::Rectangle(bound)) = sweep else {
            return Err(CrccError::InvalidGeometry(
                "test expected a rectangular rotational bound",
            ));
        };
        let radius = 10.0_f64.hypot(0.1);

        assert_relative_eq!(bound.rect().min().x, -radius);
        assert_relative_eq!(bound.rect().min().y, -radius);
        assert_relative_eq!(bound.rect().max().x, radius);
        assert_relative_eq!(bound.rect().max().y, radius);
        Ok(())
    }

    #[test]
    fn polygon_rejects_self_intersection() {
        let bow_tie = Polygon::new(
            vec![(0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0), (0.0, 0.0)].into(),
            Vec::new(),
        );

        assert_eq!(
            SimpleCollisionObject::polygon(bow_tie),
            Err(CrccError::InvalidGeometry(
                "polygon must be topologically valid",
            )),
        );
    }

    #[test]
    fn translated_circle_sweep_has_exact_extrema() -> CrccResult<()> {
        let circle = SimpleCollisionObject::circle((2.0, -1.0), 0.5)?;
        let sweep = circle.swept_area(DPose2::IDENTITY, DPose2::translation(3.0, 4.0));
        let Some(SimpleCollisionObject::Rectangle(bound)) = sweep else {
            return Err(CrccError::InvalidGeometry(
                "test expected a rectangular circle sweep",
            ));
        };

        assert_eq!(bound.rect(), &Rect::new((1.5, -1.5), (5.5, 3.5)));
        Ok(())
    }

    #[test]
    fn translated_triangle_sweep_contains_endpoint_extrema() -> CrccResult<()> {
        let triangle = SimpleCollisionObject::triangle(GeoTriangle::new(
            (0.0, 0.0).into(),
            (2.0, 0.0).into(),
            (0.0, 1.0).into(),
        ))?;
        let sweep = triangle.swept_area(DPose2::IDENTITY, DPose2::translation(3.0, -2.0));
        let Some(SimpleCollisionObject::ConvexPolygon(bound)) = sweep else {
            return Err(CrccError::InvalidGeometry(
                "test expected a convex polygon translation sweep",
            ));
        };

        assert_eq!(
            bound.bounding_rect(),
            Some(Rect::new((0.0, -2.0), (5.0, 1.0)))
        );
        Ok(())
    }

    #[test]
    fn swept_area_for_translated_half_space_keeps_expected_boundary() -> CrccResult<()> {
        let half_space = SimpleCollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0)?;
        let swept_area = half_space.swept_area(DPose2::IDENTITY, DPose2::translation(-1.0, 0.0));
        let expected = SimpleCollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0)?;
        let (
            Some(SimpleCollisionObject::HalfSpace(swept)),
            SimpleCollisionObject::HalfSpace(expected),
        ) = (swept_area, expected)
        else {
            return Err(CrccError::InvalidGeometry("test expected two half-spaces"));
        };

        assert!(
            swept.almost_equal(&expected),
            "expected {expected:?}, got {swept:?}",
        );
        Ok(())
    }

    #[test]
    fn any_half_space_rotation_uses_full_space_swept_bound() -> CrccResult<()> {
        let half_space = SimpleCollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0)?;
        let swept_area = half_space
            .swept_area(DPose2::IDENTITY, DPose2::rotation(5e-10))
            .ok_or(CrccError::InvalidGeometry("test expected one swept area"))?;

        assert!(swept_area.is_full_space());
        Ok(())
    }
}
