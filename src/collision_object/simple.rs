use crate::error::{CrccError, CrccResult};
use enum_dispatch::enum_dispatch;
use geo::{
    AffineOps, AffineTransform, BooleanOps, ConvexHull, HasDimensions, IsConvex, Polygon, Rect,
    Rotate, Triangle as GeoTriangle, Validation,
};
use glamx::{DPose2, DVec2};
use itertools::Itertools;
use std::ops::Deref;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
/// Geometry representing the empty set.
pub struct Empty;

impl Empty {
    pub const fn new() -> Self {
        Self
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
/// Geometry representing the entire plane.
pub struct FullSpace;

impl FullSpace {
    pub const fn new() -> Self {
        Self
    }
}

#[derive(Debug, Clone, PartialEq)]
/// Non-degenerate triangle geometry accepted by [`crate::CollisionObject::from`].
pub struct Triangle(pub(super) GeoTriangle);

impl Triangle {
    /// Creates a finite, non-empty triangle.
    pub fn new(triangle: GeoTriangle) -> CrccResult<Triangle> {
        if [triangle.0, triangle.1, triangle.2]
            .iter()
            .any(|coord| !coord.x.is_finite() || !coord.y.is_finite())
        {
            return Err(CrccError::InvalidGeometry(
                "triangle coordinates must be finite",
            ));
        }
        if triangle.is_empty() {
            return Err(CrccError::EmptyShape);
        }
        Ok(Triangle(triangle))
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
    pub fn new(rect: Rect, orientation: f64) -> CrccResult<Rectangle> {
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
        Ok(Rectangle { rect, orientation })
    }

    /// Returns the underlying axis-aligned local rectangle.
    pub fn rect(&self) -> &Rect {
        &self.rect
    }

    /// Returns the local-space center.
    pub fn center(&self) -> (f64, f64) {
        self.rect.center().into()
    }

    /// Returns the local x extent.
    pub fn width(&self) -> f64 {
        self.rect.width()
    }

    /// Returns the local y extent.
    pub fn height(&self) -> f64 {
        self.rect.height()
    }

    /// Returns the local orientation in radians.
    pub fn orientation(&self) -> f64 {
        self.orientation
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ConvexPolygon(pub(super) Polygon);

impl ConvexPolygon {
    pub fn new(polygon: Polygon) -> CrccResult<ConvexPolygon> {
        if !polygon.interiors().is_empty() {
            return Err(CrccError::HasHoles);
        }
        if !polygon.exterior().is_convex() {
            return Err(CrccError::NotConvex);
        }
        Ok(ConvexPolygon(polygon))
    }

    pub fn polygon(&self) -> &Polygon {
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
    pub fn new(polygon: Polygon) -> CrccResult<NonConvexPolygon> {
        if !polygon.interiors().is_empty() {
            return Err(CrccError::HasHoles);
        }
        Ok(NonConvexPolygon(polygon))
    }

    pub fn polygon(&self) -> &Polygon {
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
    pub fn new(polygon: Polygon) -> PolygonWithHoles {
        PolygonWithHoles(polygon)
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
    pub fn new(center: (f64, f64), radius: f64) -> CrccResult<Circle> {
        if !center.0.is_finite() || !center.1.is_finite() || !radius.is_finite() || radius <= 0.0 {
            return Err(CrccError::InvalidRadius(radius));
        }
        Ok(Circle { center, radius })
    }

    /// Returns the local-space center.
    pub fn center(&self) -> (f64, f64) {
        self.center
    }

    /// Returns the radius.
    pub fn radius(&self) -> f64 {
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
    /// Creates a half-space from a directed boundary line.
    pub fn from_points(p1: impl Into<DVec2>, p2: impl Into<DVec2>) -> CrccResult<Self> {
        let p1 = p1.into();
        let p2 = p2.into();
        let dir = p2 - p1;
        if !p1.is_finite() || !p2.is_finite() || dir.length_squared() == 0.0 {
            return Err(CrccError::InvalidGeometry(
                "half-space points must be finite and distinct",
            ));
        }
        let unit_normal = DVec2::new(-dir.y, dir.x).normalize();
        let offset = unit_normal.dot(p1);
        Ok(Self {
            outward_normal: unit_normal,
            offset,
        })
    }

    /// Creates the half-space `a*x + b*y <= c`.
    pub fn from_coeffs(a: f64, b: f64, c: f64) -> CrccResult<Self> {
        let normal = DVec2::new(a, b);
        if !normal.is_finite() || !c.is_finite() || normal.length_squared() == 0.0 {
            return Err(CrccError::InvalidGeometry(
                "half-space coefficients must be finite with a nonzero normal",
            ));
        }
        let offset = c / normal.length();
        Ok(Self {
            outward_normal: normal.normalize(),
            offset,
        })
    }

    /// Compares two normalized half-spaces with the default tolerance.
    pub fn almost_equal(&self, other: &HalfSpace) -> bool {
        self.almost_equal_with_tol(other, 1e-9)
    }

    /// Compares two normalized half-spaces with `tol`.
    pub fn almost_equal_with_tol(&self, other: &HalfSpace, tol: f64) -> bool {
        (self.outward_normal - other.outward_normal).length().abs() < tol
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
    pub fn empty() -> Self {
        Self::Empty(Empty)
    }

    pub fn full_space() -> Self {
        Self::FullSpace(FullSpace)
    }

    pub fn half_space(outward_normal: impl Into<DVec2>, offset: f64) -> CrccResult<Self> {
        let outward_normal = outward_normal.into();
        if !outward_normal.is_finite()
            || !offset.is_finite()
            || outward_normal.length_squared() == 0.0
        {
            return Err(CrccError::InvalidGeometry(
                "half-space requires a finite nonzero normal and finite offset",
            ));
        }
        Ok(Self::HalfSpace(HalfSpace {
            outward_normal: outward_normal.normalize(),
            offset,
        }))
    }

    pub fn half_space_from_points(p1: (f64, f64), p2: (f64, f64)) -> CrccResult<Self> {
        Ok(Self::HalfSpace(HalfSpace::from_points(p1, p2)?))
    }

    pub fn half_space_from_coeffs(a: f64, b: f64, c: f64) -> CrccResult<Self> {
        Ok(Self::HalfSpace(HalfSpace::from_coeffs(a, b, c)?))
    }

    pub fn circle(center: (f64, f64), radius: f64) -> CrccResult<Self> {
        Ok(Self::Circle(Circle::new(center, radius)?))
    }

    pub fn rectangle(rect: impl Into<Rect>, orientation: f64) -> CrccResult<Self> {
        Ok(Self::Rectangle(Rectangle::new(rect.into(), orientation)?))
    }

    pub fn triangle(triangle: impl Into<GeoTriangle>) -> CrccResult<Self> {
        Ok(Self::Triangle(Triangle::new(triangle.into())?))
    }

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

    pub fn is_empty(&self) -> bool {
        matches!(self, Self::Empty(_))
    }

    pub fn is_full_space(&self) -> bool {
        matches!(self, Self::FullSpace(_))
    }
}

#[enum_dispatch]
pub trait SweptArea {
    /// Overapproximates the area the object covers while moving through the given positions.
    /// The returned vector has length `positions.len() - 1`, with each entry corresponding to
    /// the swept area between two consecutive positions.
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject>;

    #[cfg(test)]
    fn swept_area(&self, start_pos: DPose2, end_pos: DPose2) -> SimpleCollisionObject {
        self.swept_areas(&[start_pos, end_pos])
            .pop()
            .expect("two positions produce one swept area")
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

impl SweptArea for Circle {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        let mut swept_areas = Vec::with_capacity(positions.len().saturating_sub(1));
        for (start_pos, end_pos) in positions.iter().tuple_windows() {
            if rotation_changed(*start_pos, *end_pos) && self.center != (0.0, 0.0) {
                let bound_radius = DVec2::from(self.center).length() + self.radius;
                swept_areas.push(
                    SimpleCollisionObject::rectangle(
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
                    )
                    .expect("a finite circle has a valid rotational swept bound"),
                );
                continue;
            }
            let center = DVec2::from(self.center);
            let start = start_pos * center;
            let end = end_pos * center;
            swept_areas.push(
                SimpleCollisionObject::rectangle(
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
                )
                .expect("a finite circle has a valid swept bounding rectangle"),
            );
        }
        swept_areas
    }
}

impl SweptArea for HalfSpace {
    fn swept_areas(&self, positions: &[DPose2]) -> Vec<SimpleCollisionObject> {
        let mut swept_areas = Vec::with_capacity(positions.len().saturating_sub(1));
        for (start_pos, end_pos) in positions.iter().tuple_windows() {
            if (start_pos.rotation.angle() - end_pos.rotation.angle()).abs() > 1e-9 {
                swept_areas.push(SimpleCollisionObject::full_space());
            } else {
                let outward_normal = start_pos.rotation * self.outward_normal;
                let start_offset = outward_normal.dot(start_pos.translation);
                let end_offset = outward_normal.dot(end_pos.translation);
                let offset = start_offset.max(end_offset) + self.offset;
                swept_areas.push(
                    SimpleCollisionObject::half_space(outward_normal, offset)
                        .expect("a transformed valid half-space remains valid"),
                );
            }
        }
        swept_areas
    }
}

/// Common helper function for the swept_areas implementations
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
                SimpleCollisionObject::polygon(start.union(end).convex_hull())
                    .expect("Swept area polygon should be valid")
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
    SimpleCollisionObject::rectangle(
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
    )
    .expect("a finite shape has a valid rotational swept bound")
}

pub(crate) fn rotation_changed(start: DPose2, end: DPose2) -> bool {
    (start.rotation.angle() - end.rotation.angle()).abs() > 1e-12
}

pub(crate) fn pose_to_affine(pose: DPose2) -> AffineTransform {
    AffineTransform::rotate(pose.rotation.angle().to_degrees(), (0.0, 0.0))
        .translated(pose.translation.x, pose.translation.y)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision_object::CollisionObject;
    use glamx::approx::assert_relative_eq;
    use rstest::rstest;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

    #[test]
    fn pose_to_affine_matches_pose_transform() {
        let pose = DPose2::new(DVec2::new(1.0, 2.0), FRAC_PI_2);
        let affine = pose_to_affine(pose);
        let point = (1.0, 2.0);
        let pose_point = pose * DVec2::from(point);
        let affine_point = affine.apply(point.into());
        assert_relative_eq!(pose_point.x, affine_point.x);
        assert_relative_eq!(pose_point.y, affine_point.y);
    }

    #[test]
    fn rotating_finite_shape_uses_finite_conservative_sweep() {
        let rectangle =
            SimpleCollisionObject::rectangle(Rect::new((-10.0, -0.1), (10.0, 0.1)), 0.0).unwrap();
        let sweep = rectangle.swept_area(DPose2::IDENTITY, DPose2::new(DVec2::ZERO, FRAC_PI_2));
        let SimpleCollisionObject::Rectangle(bound) = sweep else {
            panic!("expected a finite rectangular bound");
        };
        assert!(bound.rect().min().x <= -10.0);
        assert!(bound.rect().max().x >= 10.0);
        assert!(bound.rect().min().y <= -10.0);
        assert!(bound.rect().max().y >= 10.0);
    }

    #[test]
    fn polygon_rejects_self_intersection() {
        let bow_tie = Polygon::new(
            vec![(0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0), (0.0, 0.0)].into(),
            vec![],
        );
        assert!(SimpleCollisionObject::polygon(bow_tie).is_err());
    }

    #[rstest]
    fn swept_areas_cover_interpolated_shape_positions(
        #[values(
            SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap(),
            SimpleCollisionObject::rectangle(Rect::new((-2.0, -1.0), (2.0, 1.0)), 0.0).unwrap(),
            SimpleCollisionObject::triangle(GeoTriangle::new(
                (0.0, 0.0).into(),
                (1.0, 0.0).into(),
                (0.0, 1.0).into(),
            )).unwrap(),
            SimpleCollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0).unwrap(),
            // Convex polygon
            SimpleCollisionObject::polygon(Polygon::new(
                vec![(0.0, 0.0), (2.0, 0.0), (1.0, 1.0)].into(),
                vec![],
            )).unwrap(),
            // Non-convex polygon
            SimpleCollisionObject::polygon(Polygon::new(
                vec![
                    (0.0, 0.0),
                    (2.0, 0.0),
                    (2.0, 2.0),
                    (1.0, 1.0),
                    (0.0, 2.0),
                ]
                .into(),
                vec![],
            )).unwrap(),
            // Polygon with holes
            SimpleCollisionObject::polygon(Polygon::new(
                vec![(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)].into(),
                vec![vec![(1.0, 1.0), (3.0, 1.0), (2.0, 3.0)].into()],
            )).unwrap(),
            SimpleCollisionObject::full_space(),
        )]
        shape: SimpleCollisionObject,
        #[values(&[
            DPose2::IDENTITY,
            DPose2::new(DVec2::new(10.0, 20.0), FRAC_PI_4),
            DPose2::new(DVec2::new(20.0, 40.0), FRAC_PI_2),
        ])]
        positions: &[DPose2],
    ) {
        let swept_areas = shape
            .swept_areas(positions)
            .into_iter()
            .map(CollisionObject::from)
            .collect_vec();
        let shape_collision_object = CollisionObject::from(shape.clone());
        assert_eq!(swept_areas.len(), positions.len().saturating_sub(1));
        for ((start_pos, end_pos), swept_area) in
            positions.iter().tuple_windows().zip(swept_areas.iter())
        {
            // Interpolate 5 points between start_pos and end_pos
            for i in 0..=5 {
                let t = i as f64 / 5.0;
                let interpolated_position = DPose2::from_parts(
                    start_pos.translation.lerp(end_pos.translation, t),
                    start_pos.rotation.slerp(&end_pos.rotation, t),
                );
                // Check that the swept area collides with the shape at the interpolated position
                assert!(
                    crate::collision_checker::engine::collides(
                        swept_area,
                        DPose2::IDENTITY,
                        &shape_collision_object,
                        interpolated_position,
                        crate::collision_checker::engine::CollisionEngine::default()
                    )
                    .unwrap()
                );
            }
        }
    }

    #[test]
    fn swept_area_for_translated_half_space_keeps_expected_boundary() {
        let hs = SimpleCollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0).unwrap(); // x <= 0.0
        let swept_area = hs.swept_area(DPose2::IDENTITY, DPose2::translation(-1.0, 0.0));
        // The swept area should be x <= 0.0
        let expected_hs = SimpleCollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0).unwrap();
        match (swept_area, expected_hs) {
            (SimpleCollisionObject::HalfSpace(sa), SimpleCollisionObject::HalfSpace(ea)) => {
                assert!(sa.almost_equal(&ea), "Expected {:?}, got {:?}", ea, sa);
            }
            _ => panic!("Swept area is not a half space"),
        }
    }
}
