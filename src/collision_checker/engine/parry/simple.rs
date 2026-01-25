use crate::collision_object::simple::{
    Circle, ConvexPolygon, HalfSpace, NonConvexPolygon, PolygonWithHoles, Rectangle,
    SimpleCollisionObject, Triangle,
};
use geo::{LineString, TriangulateEarcut, Winding};
use glamx::{DPose2, DVec2};
use itertools::Itertools;
use parry2d_f64::shape::{
    Ball, ConvexPolygon as ParryConvexPolygon, Cuboid, HalfSpace as ParryHalfSpace, Shape,
    SharedShape, TriMesh, Triangle as ParryTriangle,
};

pub enum ParrySimpleCollisionObject {
    Empty,
    FullSpace,
    TriMesh(Box<TriMesh>),
    Shape {
        shape: Box<dyn Shape>,
        position: DPose2,
    },
}

impl ParrySimpleCollisionObject {
    pub fn into_shared_shape(self) -> Option<(DPose2, SharedShape)> {
        match self {
            ParrySimpleCollisionObject::Empty | ParrySimpleCollisionObject::FullSpace => None,
            ParrySimpleCollisionObject::TriMesh(mesh) => {
                Some((DPose2::IDENTITY, SharedShape::new(*mesh)))
            }
            ParrySimpleCollisionObject::Shape { shape, position } => {
                Some((position, SharedShape(shape.into())))
            }
        }
    }
}

impl From<SimpleCollisionObject> for ParrySimpleCollisionObject {
    fn from(collision_object: SimpleCollisionObject) -> Self {
        match collision_object {
            SimpleCollisionObject::Empty(..) => ParrySimpleCollisionObject::Empty,
            SimpleCollisionObject::FullSpace(..) => ParrySimpleCollisionObject::FullSpace,
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
    let support = half_space.offset * half_space.outward_normal;
    ParrySimpleCollisionObject::Shape {
        shape: Box::new(ParryHalfSpace::new(half_space.outward_normal)),
        position: DPose2::translation(support.x, support.y),
    }
}

fn convert_circle(circle: Circle) -> ParrySimpleCollisionObject {
    ParrySimpleCollisionObject::Shape {
        shape: Box::new(Ball::new(circle.radius())),
        position: make_pose(circle.center(), 0.0),
    }
}

fn convert_rectangle(rectangle: Rectangle) -> ParrySimpleCollisionObject {
    let half_extents = DVec2::new(rectangle.width() / 2.0, rectangle.height() / 2.0);
    ParrySimpleCollisionObject::Shape {
        shape: Box::new(Cuboid::new(half_extents)),
        position: make_pose(rectangle.center(), rectangle.orientation()),
    }
}

fn convert_triangle(triangle: Triangle) -> ParrySimpleCollisionObject {
    ParrySimpleCollisionObject::Shape {
        shape: Box::new(ParryTriangle::new(
            DVec2::new(triangle.0.x, triangle.0.y),
            DVec2::new(triangle.1.x, triangle.1.y),
            DVec2::new(triangle.2.x, triangle.2.y),
        )),
        position: DPose2::IDENTITY,
    }
}

fn convert_convex_polygon(convex_polygon: ConvexPolygon) -> ParrySimpleCollisionObject {
    let parry_convex = ParryConvexPolygon::from_convex_polyline(geo_line_string_to_parry_polyline(
        convex_polygon.exterior(),
    ))
    .expect("Convex polygon should be a valid convex polygon");
    ParrySimpleCollisionObject::Shape {
        shape: Box::new(parry_convex),
        position: DPose2::IDENTITY,
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
            .tuples::<(_, _)>()
            .map_into()
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

fn geo_line_string_to_parry_polyline(line_string: &LineString) -> Vec<DVec2> {
    line_string
        .points_ccw()
        .skip(1) // Skip duplicate first point
        .map(|point| DVec2::new(point.x(), point.y()))
        .collect()
}

fn make_pose(translation: impl Into<DVec2>, rotation: f64) -> DPose2 {
    DPose2::new(translation.into(), rotation)
}
