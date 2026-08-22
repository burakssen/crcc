use crate::collision_object::simple::{
    Circle, ConvexPolygon, HalfSpace, NonConvexPolygon, PolygonWithHoles, Rectangle,
    SimpleCollisionObject, Triangle,
};
use geo::{LineString, TriangulateEarcut, Winding};
use glamx::{DPose2, DVec2};
use parry2d_f64::shape::{
    Ball, ConvexPolygon as ParryConvexPolygon, Cuboid, HalfSpace as ParryHalfSpace, Shape, TriMesh,
    Triangle as ParryTriangle,
};
use std::ops::{Div, Mul};

pub enum ParrySimpleCollisionObject {
    Empty,
    FullSpace,
    Invalid,
    TriMesh(Box<TriMesh>),
    Shape {
        shape: Box<dyn Shape>,
        position: DPose2,
    },
}

impl From<SimpleCollisionObject> for ParrySimpleCollisionObject {
    fn from(collision_object: SimpleCollisionObject) -> Self {
        match &collision_object {
            SimpleCollisionObject::Empty(..) => Self::Empty,
            SimpleCollisionObject::FullSpace(..) => Self::FullSpace,
            SimpleCollisionObject::HalfSpace(half_space) => convert_half_space(half_space),
            SimpleCollisionObject::Circle(circle) => convert_circle(circle),
            SimpleCollisionObject::Rectangle(rectangle) => convert_rectangle(rectangle),
            SimpleCollisionObject::Triangle(triangle) => convert_triangle(triangle),
            SimpleCollisionObject::ConvexPolygon(polygon) => convert_convex_polygon(polygon),
            SimpleCollisionObject::NonConvexPolygon(polygon) => convert_non_convex_polygon(polygon),
            SimpleCollisionObject::PolygonWithHoles(polygon) => convert_polygon_with_holes(polygon),
        }
    }
}

fn convert_half_space(half_space: &HalfSpace) -> ParrySimpleCollisionObject {
    let support = half_space.outward_normal.mul(half_space.offset);

    ParrySimpleCollisionObject::Shape {
        shape: Box::new(ParryHalfSpace::new(half_space.outward_normal)),
        position: DPose2::translation(support.x, support.y),
    }
}

fn convert_circle(circle: &Circle) -> ParrySimpleCollisionObject {
    ParrySimpleCollisionObject::Shape {
        shape: Box::new(Ball::new(circle.radius())),
        position: make_pose(circle.center(), 0.0),
    }
}

fn convert_rectangle(rectangle: &Rectangle) -> ParrySimpleCollisionObject {
    let half_extents = DVec2::new(rectangle.width().div(2.0), rectangle.height().div(2.0));

    ParrySimpleCollisionObject::Shape {
        shape: Box::new(Cuboid::new(half_extents)),
        position: make_pose(rectangle.center(), rectangle.orientation()),
    }
}

fn convert_triangle(triangle: &Triangle) -> ParrySimpleCollisionObject {
    ParrySimpleCollisionObject::Shape {
        shape: Box::new(ParryTriangle::new(
            DVec2::new(triangle.0.x, triangle.0.y),
            DVec2::new(triangle.1.x, triangle.1.y),
            DVec2::new(triangle.2.x, triangle.2.y),
        )),
        position: DPose2::IDENTITY,
    }
}

fn convert_convex_polygon(convex_polygon: &ConvexPolygon) -> ParrySimpleCollisionObject {
    let vertices = geo_line_string_to_parry_polyline(convex_polygon.exterior());

    ParryConvexPolygon::from_convex_polyline(vertices.clone())
        // parry prunes needle-thin convex polygons below its collinearity
        // epsilon to <3 points; keep the unpruned polygon instead of poisoning the scene.
        .or_else(|| ParryConvexPolygon::from_convex_polyline_unmodified(vertices))
        .map_or_else(
            || ParrySimpleCollisionObject::Invalid,
            |polygon| ParrySimpleCollisionObject::Shape {
                shape: Box::new(polygon),
                position: DPose2::IDENTITY,
            },
        )
}

fn convert_non_convex_polygon(non_convex_polygon: &NonConvexPolygon) -> ParrySimpleCollisionObject {
    let vertices = geo_line_string_to_parry_polyline(non_convex_polygon.exterior());

    TriMesh::from_polygon(vertices).map_or_else(
        || ParrySimpleCollisionObject::Invalid,
        trimesh_collision_object,
    )
}

fn convert_polygon_with_holes(polygon_with_holes: &PolygonWithHoles) -> ParrySimpleCollisionObject {
    let triangulation = polygon_with_holes.earcut_triangles_raw();

    let Some(vertices) = triangulation_vertices(&triangulation.vertices) else {
        return ParrySimpleCollisionObject::Invalid;
    };

    let Some(indices) = triangulation_indices(&triangulation.triangle_indices) else {
        return ParrySimpleCollisionObject::Invalid;
    };

    TriMesh::new(vertices, indices).map_or_else(
        |_| ParrySimpleCollisionObject::Invalid,
        trimesh_collision_object,
    )
}

fn trimesh_collision_object(mesh: TriMesh) -> ParrySimpleCollisionObject {
    ParrySimpleCollisionObject::TriMesh(Box::new(mesh))
}

fn triangulation_vertices(coordinates: &[f64]) -> Option<Vec<DVec2>> {
    let mut coordinate_pairs = coordinates.chunks_exact(2);

    let vertices = coordinate_pairs
        .by_ref()
        .map(|coordinate| {
            let [x, y] = coordinate else {
                return None;
            };

            Some(DVec2::new(*x, *y))
        })
        .collect::<Option<Vec<_>>>()?;

    coordinate_pairs.remainder().is_empty().then_some(vertices)
}

fn triangulation_indices(indices: &[usize]) -> Option<Vec<[u32; 3]>> {
    let mut index_triples = indices.chunks_exact(3);

    let triangles = index_triples
        .by_ref()
        .map(|triangle| {
            let [a, b, c] = triangle else {
                return None;
            };

            Some([
                u32::try_from(*a).ok()?,
                u32::try_from(*b).ok()?,
                u32::try_from(*c).ok()?,
            ])
        })
        .collect::<Option<Vec<_>>>()?;

    index_triples.remainder().is_empty().then_some(triangles)
}

fn geo_line_string_to_parry_polyline(line_string: &LineString) -> Vec<DVec2> {
    line_string
        .points_ccw()
        .skip(1)
        .map(|point| DVec2::new(point.x(), point.y()))
        .collect()
}

fn make_pose(translation: impl Into<DVec2>, rotation: f64) -> DPose2 {
    DPose2::new(translation.into(), rotation)
}
