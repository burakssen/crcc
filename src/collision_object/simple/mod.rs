use geo::{HasDimensions, IsConvex, Polygon, Rect, Triangle as GeoTriangle};
use nalgebra::{Unit, Vector2};

pub use circle::Circle;
pub use convex_polygon::ConvexPolygon;
pub use half_space::HalfSpace;
pub use non_convex_polygon::NonConvexPolygon;
pub use polygon_with_holes::PolygonWithHoles;
pub use rectangle::Rectangle;
pub use triangle::Triangle;

mod circle;
mod convex_polygon;
mod half_space;
mod non_convex_polygon;
mod polygon_with_holes;
mod rectangle;
mod triangle;

#[derive(Debug, Clone, Default)]
pub enum SimpleCollisionObject {
    #[default]
    Empty,
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
        SimpleCollisionObject::Empty
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
            SimpleCollisionObject::Empty
        } else {
            SimpleCollisionObject::Circle(Circle { center, radius })
        }
    }

    pub fn rectangle(rect: Rect) -> SimpleCollisionObject {
        if rect.is_empty() {
            SimpleCollisionObject::Empty
        } else {
            SimpleCollisionObject::Rectangle(Rectangle(rect))
        }
    }

    pub fn triangle(triangle: GeoTriangle) -> SimpleCollisionObject {
        if triangle.is_empty() {
            SimpleCollisionObject::Empty
        } else {
            SimpleCollisionObject::Triangle(Triangle(triangle))
        }
    }

    pub fn polygon(polygon: Polygon) -> SimpleCollisionObject {
        if polygon.is_empty() {
            return SimpleCollisionObject::Empty;
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
}
