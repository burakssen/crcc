use geo::{HasDimensions, IsConvex, Polygon, Rect, Triangle as GeoTriangle};

pub mod circle;
pub mod convex_polygon;
pub mod non_convex_polygon;
pub mod polygon_with_holes;
pub mod rectangle;
pub mod triangle;

pub use circle::Circle;
pub use convex_polygon::ConvexPolygon;
pub use non_convex_polygon::NonConvexPolygon;
pub use polygon_with_holes::PolygonWithHoles;
pub use rectangle::Rectangle;
pub use triangle::Triangle;

pub enum CollisionObject {
    Empty,
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
