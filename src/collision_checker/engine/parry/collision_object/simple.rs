use crate::collision_object::simple::{
    Circle, ConvexPolygon, HalfSpace, NonConvexPolygon, PolygonWithHoles, Rectangle,
    SimpleCollisionObject, Triangle,
};
use geo::{LineString, TriangulateEarcut, Winding};
use itertools::Itertools;
use nalgebra::{Isometry2, Point2, Vector2};
use parry2d_f64::shape::{
    Ball, ConvexPolygon as ParryConvexPolygon, Cuboid, HalfSpace as ParryHalfSpace, Shape,
    SharedShape, TriMesh, Triangle as ParryTriangle,
};

pub enum ParrySimpleCollisionObject {
    Empty,
    TriMesh(Box<TriMesh>),
    Generic {
        shape: Box<dyn Shape>,
        position: Isometry2<f64>,
    },
}

impl ParrySimpleCollisionObject {
    pub fn into_shared_shape(self) -> Option<(Isometry2<f64>, SharedShape)> {
        match self {
            ParrySimpleCollisionObject::Empty => None,
            ParrySimpleCollisionObject::TriMesh(mesh) => {
                Some((Isometry2::identity(), SharedShape::new(*mesh)))
            }
            ParrySimpleCollisionObject::Generic { shape, position } => {
                Some((position, SharedShape(shape.into())))
            }
        }
    }
}

impl From<SimpleCollisionObject> for ParrySimpleCollisionObject {
    fn from(collision_object: SimpleCollisionObject) -> Self {
        match collision_object {
            SimpleCollisionObject::Empty => ParrySimpleCollisionObject::Empty,
            SimpleCollisionObject::HalfSpace(half_space) => convert_half_space(half_space),
            SimpleCollisionObject::Circle(circle) => convert_circle(circle),
            SimpleCollisionObject::Rectangle(rect) => convert_rectangle(rect),
            SimpleCollisionObject::Triangle(triangle) => convert_triangle(triangle),
            SimpleCollisionObject::ConvexPolygon(convex_polygon) => {
                convert_convex_polygon(convex_polygon)
            }
            SimpleCollisionObject::NonConvexPolygon(non_convex_polygon) => {
                convert_non_convex_polygon(non_convex_polygon)
            }
            SimpleCollisionObject::PolygonWithHoles(polygon_with_holes) => {
                convert_polygon_with_holes(polygon_with_holes)
            }
        }
    }
}

fn convert_half_space(half_space: HalfSpace) -> ParrySimpleCollisionObject {
    let support = half_space.offset * *half_space.outward_normal;
    ParrySimpleCollisionObject::Generic {
        shape: Box::new(ParryHalfSpace::new(half_space.outward_normal)),
        position: Isometry2::translation(support.x, support.y),
    }
}

fn convert_circle(circle: Circle) -> ParrySimpleCollisionObject {
    ParrySimpleCollisionObject::Generic {
        shape: Box::new(Ball::new(circle.radius())),
        position: make_isometry(circle.center(), 0.0),
    }
}

fn convert_rectangle(rect: Rectangle) -> ParrySimpleCollisionObject {
    let half_extents = Vector2::new(rect.width() / 2.0, rect.height() / 2.0);
    ParrySimpleCollisionObject::Generic {
        shape: Box::new(Cuboid::new(half_extents)),
        position: make_isometry(rect.center().into(), 0.0),
    }
}

fn convert_triangle(triangle: Triangle) -> ParrySimpleCollisionObject {
    ParrySimpleCollisionObject::Generic {
        shape: Box::new(ParryTriangle::new(
            Point2::new(triangle.0.x, triangle.0.y),
            Point2::new(triangle.1.x, triangle.1.y),
            Point2::new(triangle.2.x, triangle.2.y),
        )),
        position: Isometry2::identity(),
    }
}

fn convert_convex_polygon(convex_polygon: ConvexPolygon) -> ParrySimpleCollisionObject {
    let parry_convex = ParryConvexPolygon::from_convex_polyline(geo_line_string_to_parry_polyline(
        convex_polygon.exterior(),
    ))
    .expect("Convex polygon should be a valid convex polygon");
    ParrySimpleCollisionObject::Generic {
        shape: Box::new(parry_convex),
        position: Isometry2::identity(),
    }
}

fn convert_non_convex_polygon(non_convex_polygon: NonConvexPolygon) -> ParrySimpleCollisionObject {
    let trimesh = TriMesh::from_polygon(geo_line_string_to_parry_polyline(
        non_convex_polygon.exterior(),
    ))
    .expect("Non-convex polygon should be a valid polygon");
    ParrySimpleCollisionObject::TriMesh(Box::new(trimesh))
}

fn convert_polygon_with_holes(polygon_with_holes: PolygonWithHoles) -> ParrySimpleCollisionObject {
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
    ParrySimpleCollisionObject::TriMesh(Box::new(trimesh))
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
