use geo::{HasDimensions, IsConvex, Polygon, Rect, Triangle as GeoTriangle};
use nalgebra::{Unit, Vector2};

mod circle;
mod convex_polygon;
mod half_space;
mod non_convex_polygon;
mod polygon_with_holes;
mod rectangle;
mod triangle;

pub use circle::Circle;
pub use convex_polygon::ConvexPolygon;
pub use half_space::HalfSpace;
pub use non_convex_polygon::NonConvexPolygon;
pub use polygon_with_holes::PolygonWithHoles;
pub use rectangle::Rectangle;
pub use triangle::Triangle;

#[derive(Debug, Clone)]
pub enum CollisionObject {
    Empty,
    HalfSpace(HalfSpace),
    Circle(Circle),
    Rectangle(Rectangle),
    Triangle(Triangle),
    ConvexPolygon(ConvexPolygon),
    NonConvexPolygon(NonConvexPolygon),
    PolygonWithHoles(PolygonWithHoles),
}

impl CollisionObject {
    pub fn empty() -> CollisionObject {
        CollisionObject::Empty
    }

    pub fn half_space(outward_normal: Unit<Vector2<f64>>, offset: f64) -> CollisionObject {
        CollisionObject::HalfSpace(HalfSpace {
            outward_normal,
            offset,
        })
    }

    pub fn half_space_from_points(p1: (f64, f64), p2: (f64, f64)) -> CollisionObject {
        // Create a half space defined by the line through p1 and p2,
        // with the outward normal pointing to the left of the line
        CollisionObject::HalfSpace(HalfSpace::from_points(p1, p2))
    }

    pub fn half_space_from_coeffs(a: f64, b: f64, c: f64) -> CollisionObject {
        // Represents the half space ax + by <= c
        CollisionObject::HalfSpace(HalfSpace::from_coeffs(a, b, c))
    }

    pub fn circle(center: (f64, f64), radius: f64) -> CollisionObject {
        if radius <= 0.0 {
            CollisionObject::Empty
        } else {
            CollisionObject::Circle(Circle { center, radius })
        }
    }

    pub fn rectangle(rect: Rect) -> CollisionObject {
        if rect.is_empty() {
            CollisionObject::Empty
        } else {
            CollisionObject::Rectangle(Rectangle(rect))
        }
    }

    pub fn triangle(triangle: GeoTriangle) -> CollisionObject {
        if triangle.is_empty() {
            CollisionObject::Empty
        } else {
            CollisionObject::Triangle(Triangle(triangle))
        }
    }

    pub fn polygon(polygon: Polygon) -> CollisionObject {
        if polygon.is_empty() {
            return CollisionObject::Empty;
        }
        match (
            polygon.exterior().is_convex(),
            polygon.interiors().is_empty(),
        ) {
            (true, true) => CollisionObject::ConvexPolygon(ConvexPolygon(polygon)),
            (false, true) => CollisionObject::NonConvexPolygon(NonConvexPolygon(polygon)),
            _ => CollisionObject::PolygonWithHoles(PolygonWithHoles(polygon)),
        }
    }
}
