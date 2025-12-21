use crate::collision_object::{
    Circle, CollisionObject, ConvexPolygon, HalfSpace, NonConvexPolygon, PolygonWithHoles,
    Rectangle, Triangle,
};
use geo::{LineString, TriangulateEarcut, Winding};
use itertools::Itertools;
use nalgebra::{Isometry2, Point2, Vector2};
use parry2d_f64::shape::{
    Ball, ConvexPolygon as ParryConvexPolygon, Cuboid, HalfSpace as ParryHalfSpace, Shape,
    SharedShape, TriMesh, Triangle as ParryTriangle,
};

pub struct ParryCollisionObject(pub(super) ParryCollisionObjectInner);

pub(super) enum ParryCollisionObjectInner {
    Empty,
    TriMesh(Box<TriMesh>),
    Generic {
        shape: Box<dyn Shape>,
        position: Isometry2<f64>,
    },
}

impl ParryCollisionObjectInner {
    pub fn into_shared_shape(self) -> Option<(Isometry2<f64>, SharedShape)> {
        match self {
            ParryCollisionObjectInner::Empty => None,
            ParryCollisionObjectInner::TriMesh(mesh) => {
                Some((Isometry2::identity(), SharedShape::new(*mesh)))
            }
            ParryCollisionObjectInner::Generic { shape, position } => {
                Some((position, SharedShape(shape.into())))
            }
        }
    }
}

impl From<CollisionObject> for ParryCollisionObject {
    fn from(collision_object: CollisionObject) -> Self {
        let inner = match collision_object {
            CollisionObject::Empty => ParryCollisionObjectInner::Empty,
            CollisionObject::HalfSpace(half_space) => convert_half_space(half_space),
            CollisionObject::Circle(circle) => convert_circle(circle),
            CollisionObject::Rectangle(rect) => convert_rectangle(rect),
            CollisionObject::Triangle(triangle) => convert_triangle(triangle),
            CollisionObject::ConvexPolygon(convex_polygon) => {
                convert_convex_polygon(convex_polygon)
            }
            CollisionObject::NonConvexPolygon(non_convex_polygon) => {
                convert_non_convex_polygon(non_convex_polygon)
            }
            CollisionObject::PolygonWithHoles(polygon_with_holes) => {
                convert_polygon_with_holes(polygon_with_holes)
            }
        };
        ParryCollisionObject(inner)
    }
}

fn convert_half_space(half_space: HalfSpace) -> ParryCollisionObjectInner {
    let support = half_space.offset * *half_space.outward_normal;
    ParryCollisionObjectInner::Generic {
        shape: Box::new(ParryHalfSpace::new(half_space.outward_normal)),
        position: Isometry2::translation(support.x, support.y),
    }
}

fn convert_circle(circle: Circle) -> ParryCollisionObjectInner {
    ParryCollisionObjectInner::Generic {
        shape: Box::new(Ball::new(circle.radius())),
        position: make_isometry(circle.center(), 0.0),
    }
}

fn convert_rectangle(rect: Rectangle) -> ParryCollisionObjectInner {
    let half_extents = Vector2::new(rect.width() / 2.0, rect.height() / 2.0);
    ParryCollisionObjectInner::Generic {
        shape: Box::new(Cuboid::new(half_extents)),
        position: make_isometry(rect.center().into(), 0.0),
    }
}

fn convert_triangle(triangle: Triangle) -> ParryCollisionObjectInner {
    ParryCollisionObjectInner::Generic {
        shape: Box::new(ParryTriangle::new(
            Point2::new(triangle.0.x, triangle.0.y),
            Point2::new(triangle.1.x, triangle.1.y),
            Point2::new(triangle.2.x, triangle.2.y),
        )),
        position: Isometry2::identity(),
    }
}

fn convert_convex_polygon(convex_polygon: ConvexPolygon) -> ParryCollisionObjectInner {
    let parry_convex = ParryConvexPolygon::from_convex_polyline(geo_line_string_to_parry_polyline(
        convex_polygon.exterior(),
    ))
    .expect("Convex polygon should be a valid convex polygon");
    ParryCollisionObjectInner::Generic {
        shape: Box::new(parry_convex),
        position: Isometry2::identity(),
    }
}

fn convert_non_convex_polygon(non_convex_polygon: NonConvexPolygon) -> ParryCollisionObjectInner {
    let trimesh = TriMesh::from_polygon(geo_line_string_to_parry_polyline(
        non_convex_polygon.exterior(),
    ))
    .expect("Non-convex polygon should be a valid polygon");
    ParryCollisionObjectInner::TriMesh(Box::new(trimesh))
}

fn convert_polygon_with_holes(polygon_with_holes: PolygonWithHoles) -> ParryCollisionObjectInner {
    let triangulation = polygon_with_holes.earcut_triangles_raw();
    let trimesh = TriMesh::new(
        triangulation
            .vertices
            .into_iter()
            .tuples()
            .map(|(x, y)| Point2::new(x, y))
            .collect(),
        triangulation
            .triangle_indices
            .into_iter()
            .tuples()
            .map(|(a, b, c)| [a as u32, b as u32, c as u32])
            .collect(),
    )
    .expect("Triangulated polygon should be a valid trimesh");
    ParryCollisionObjectInner::TriMesh(Box::new(trimesh))
}

fn geo_line_string_to_parry_polyline(line_string: &LineString) -> Vec<Point2<f64>> {
    line_string
        .points_ccw()
        .skip(1) // Skip duplicate first point
        .map(|point| Point2::new(point.x(), point.y()))
        .collect()
}

fn make_isometry(translation: (f64, f64), rotation: f64) -> Isometry2<f64> {
    Isometry2::new(Vector2::new(translation.0, translation.1), rotation)
}
