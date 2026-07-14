use crate::error::{CrccError, CrccResult};
use enum_dispatch::enum_dispatch;
use geo::{
    AffineOps, AffineTransform, BooleanOps, ConvexHull, HasDimensions, IsConvex, Polygon, Rect,
    Triangle as GeoTriangle, Validation,
};
use glamx::{DPose2, DVec2};
use itertools::Itertools;

pub use circle::Circle;
pub use convex_polygon::ConvexPolygon;
pub use empty::Empty;
pub use full_space::FullSpace;
pub use half_space::HalfSpace;
pub use non_convex_polygon::NonConvexPolygon;
pub use polygon_with_holes::PolygonWithHoles;
pub use rectangle::Rectangle;
pub use triangle::Triangle;

mod circle;
mod convex_polygon;
mod empty;
mod full_space;
mod half_space;
mod non_convex_polygon;
mod polygon_with_holes;
mod rectangle;
mod triangle;

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
        // Create a half space defined by the line through p1 and p2,
        // with the outward normal pointing to the left of the line
        Ok(Self::HalfSpace(HalfSpace::from_points(p1, p2)?))
    }

    pub fn half_space_from_coeffs(a: f64, b: f64, c: f64) -> CrccResult<Self> {
        // Represents the half space ax + by <= c
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

    #[cfg(feature = "parry")]
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
