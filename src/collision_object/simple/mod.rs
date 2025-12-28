pub use circle::Circle;
pub use convex_polygon::ConvexPolygon;
pub use empty::Empty;
use enum_dispatch::enum_dispatch;
use geo::{
    AffineOps, AffineTransform, BooleanOps, ConvexHull, HasDimensions, IsConvex, Polygon, Rect,
    Triangle as GeoTriangle,
};
pub use half_space::HalfSpace;
use itertools::Itertools;
use nalgebra::{Isometry2, Unit, Vector2};
pub use non_convex_polygon::NonConvexPolygon;
pub use polygon_with_holes::PolygonWithHoles;
pub use rectangle::Rectangle;
use std::f64::consts::PI;
pub use triangle::Triangle;

mod circle;
mod convex_polygon;
mod empty;
mod half_space;
mod non_convex_polygon;
mod polygon_with_holes;
mod rectangle;
mod triangle;

#[derive(Debug, Clone)]
#[enum_dispatch(SimpleCollisionObjectOps)]
pub enum SimpleCollisionObject {
    Empty(Empty),
    HalfSpace(HalfSpace),
    Circle(Circle),
    Rectangle(Rectangle),
    Triangle(Triangle),
    ConvexPolygon(ConvexPolygon),
    NonConvexPolygon(NonConvexPolygon),
    PolygonWithHoles(PolygonWithHoles),
}

impl SimpleCollisionObject {
    pub fn empty() -> SimpleCollisionObject {
        SimpleCollisionObject::Empty(Empty {})
    }

    pub fn half_space(outward_normal: Unit<Vector2<f64>>, offset: f64) -> SimpleCollisionObject {
        SimpleCollisionObject::HalfSpace(HalfSpace {
            outward_normal,
            offset,
        })
    }

    pub fn half_space_from_points(p1: (f64, f64), p2: (f64, f64)) -> SimpleCollisionObject {
        // Create a half space defined by the line through p1 and p2,
        // with the outward normal pointing to the left of the line
        SimpleCollisionObject::HalfSpace(HalfSpace::from_points(p1, p2))
    }

    pub fn half_space_from_coeffs(a: f64, b: f64, c: f64) -> SimpleCollisionObject {
        // Represents the half space ax + by <= c
        SimpleCollisionObject::HalfSpace(HalfSpace::from_coeffs(a, b, c))
    }

    pub fn circle(center: (f64, f64), radius: f64) -> SimpleCollisionObject {
        if radius <= 0.0 {
            SimpleCollisionObject::empty()
        } else {
            SimpleCollisionObject::Circle(Circle { center, radius })
        }
    }

    pub fn rectangle(rect: Rect) -> SimpleCollisionObject {
        if rect.is_empty() {
            SimpleCollisionObject::empty()
        } else {
            SimpleCollisionObject::Rectangle(Rectangle(rect))
        }
    }

    pub fn triangle(triangle: GeoTriangle) -> SimpleCollisionObject {
        if triangle.is_empty() {
            SimpleCollisionObject::empty()
        } else {
            SimpleCollisionObject::Triangle(Triangle(triangle))
        }
    }

    pub fn polygon(polygon: Polygon) -> SimpleCollisionObject {
        if polygon.is_empty() {
            return SimpleCollisionObject::empty();
        }
        match (
            polygon.exterior().is_convex(),
            polygon.interiors().is_empty(),
        ) {
            (true, true) => SimpleCollisionObject::ConvexPolygon(ConvexPolygon(polygon)),
            (false, true) => SimpleCollisionObject::NonConvexPolygon(NonConvexPolygon(polygon)),
            _ => SimpleCollisionObject::PolygonWithHoles(PolygonWithHoles(polygon)),
        }
    }
    pub fn is_empty(&self) -> bool {
        matches!(self, SimpleCollisionObject::Empty(_))
    }
}

#[enum_dispatch]
pub trait SimpleCollisionObjectOps {
    fn swept_areas(&self, positions: &[Isometry2<f64>]) -> Vec<SimpleCollisionObject>;

    fn swept_area(
        &self,
        start_pos: &Isometry2<f64>,
        end_pos: &Isometry2<f64>,
    ) -> SimpleCollisionObject {
        self.swept_areas(&[*start_pos, *end_pos])
            .pop()
            .expect("Should return exactly one area, as two positions were given.")
    }
}

/// Common helper function for the swept_areas implementations
fn swept_areas(
    shape: &(impl AffineOps<f64> + Into<Polygon>),
    positions: &[Isometry2<f64>],
) -> Vec<SimpleCollisionObject> {
    let transformed_shapes = positions
        .iter()
        .map(isometry_to_affine)
        .map(|affine| shape.affine_transform(&affine))
        .map_into()
        .collect_vec();
    transformed_shapes
        .iter()
        .tuple_windows()
        .map(|(start, end)| start.union(end).convex_hull())
        .map(SimpleCollisionObject::polygon)
        .collect()
}

fn isometry_to_affine(isometry: &Isometry2<f64>) -> AffineTransform {
    AffineTransform::rotate(isometry.rotation.angle() * (180.0 / PI), (0.0, 0.0))
        .translated(isometry.translation.x, isometry.translation.y)
}
