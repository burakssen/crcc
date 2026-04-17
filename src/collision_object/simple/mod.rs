use crate::error::{CrccError, CrccResult};
use enum_dispatch::enum_dispatch;
use geo::{
    AffineOps, AffineTransform, BooleanOps, ConvexHull, HasDimensions, IsConvex, Polygon, Rect,
    Triangle as GeoTriangle,
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

    pub fn half_space(outward_normal: impl Into<DVec2>, offset: f64) -> Self {
        Self::HalfSpace(HalfSpace {
            outward_normal: outward_normal.into().normalize(),
            offset,
        })
    }

    pub fn half_space_from_points(p1: (f64, f64), p2: (f64, f64)) -> Self {
        // Create a half space defined by the line through p1 and p2,
        // with the outward normal pointing to the left of the line
        Self::HalfSpace(HalfSpace::from_points(p1, p2))
    }

    pub fn half_space_from_coeffs(a: f64, b: f64, c: f64) -> Self {
        // Represents the half space ax + by <= c
        Self::HalfSpace(HalfSpace::from_coeffs(a, b, c))
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

    fn swept_area(&self, start_pos: DPose2, end_pos: DPose2) -> SimpleCollisionObject {
        self.swept_areas(&[start_pos, end_pos])
            .pop()
            .expect("Should return exactly one area, as two positions were given.")
    }
}

/// Common helper function for the swept_areas implementations
fn swept_areas(
    shape: &(impl AffineOps<f64> + Into<Polygon>),
    positions: &[DPose2],
) -> Vec<SimpleCollisionObject> {
    let transformed_shapes = positions
        .iter()
        .copied()
        .map(pose_to_affine)
        .map(|affine| shape.affine_transform(&affine))
        .map_into()
        .collect_vec();
    transformed_shapes
        .iter()
        .tuple_windows()
        .map(|(start, end)| start.union(end).convex_hull())
        .map(|poly| {
            SimpleCollisionObject::polygon(poly).expect("Swept area polygon should be valid")
        })
        .collect()
}

fn pose_to_affine(pose: DPose2) -> AffineTransform {
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
            SimpleCollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0),
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
        let hs = SimpleCollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0); // x <= 0.0
        let swept_area = hs.swept_area(DPose2::IDENTITY, DPose2::translation(-1.0, 0.0));
        // The swept area should be x <= -1.0
        let expected_hs = SimpleCollisionObject::half_space_from_coeffs(1.0, 0.0, -1.0);
        match (swept_area, expected_hs) {
            (SimpleCollisionObject::HalfSpace(sa), SimpleCollisionObject::HalfSpace(ea)) => {
                assert!(sa.almost_equal(&ea), "Expected {:?}, got {:?}", ea, sa);
            }
            _ => panic!("Swept area is not a half space"),
        }
    }
}
