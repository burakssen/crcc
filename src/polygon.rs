use geo::{HasDimensions, IsConvex, LineString, Polygon, TriangulateEarcut, Winding};
use itertools::Itertools;
use nalgebra::Point2;
use parry2d_f64::shape::{ConvexPolygon, SharedShape, TriMesh};

pub enum PolygonCollisionObject {
    ConvexPolygon(ConvexPolygon),
    TriMesh(TriMesh),
}

impl PolygonCollisionObject {
    pub fn new(polygon: &Polygon) -> Option<Self> {
        if polygon.is_empty() {
            return None;
        }

        let has_holes = !polygon.interiors().is_empty();
        let is_convex = polygon.exterior().is_convex();

        match (has_holes, is_convex) {
            (false, true) => {
                let convex_poly = ConvexPolygon::from_convex_polyline(
                    geo_line_string_to_parry_polyline(polygon.exterior()),
                )
                .expect("Polygon should be a valid convex polygon");
                Some(Self::ConvexPolygon(convex_poly))
            }
            (false, false) => {
                let trimesh =
                    TriMesh::from_polygon(geo_line_string_to_parry_polyline(polygon.exterior()))
                        .expect("Polygon should be a valid polygon");
                Some(Self::TriMesh(trimesh))
            }
            _ => Some(Self::TriMesh(triangulate_polygon_with_holes(polygon))),
        }
    }

    pub fn into_shared(self) -> SharedShape {
        match self {
            PolygonCollisionObject::ConvexPolygon(poly) => SharedShape::new(poly),
            PolygonCollisionObject::TriMesh(mesh) => SharedShape::new(mesh),
        }
    }
}

fn triangulate_polygon_with_holes(polygon: &Polygon) -> TriMesh {
    let triangulation = polygon.earcut_triangles_raw();
    TriMesh::new(
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
    .expect("Triangulated polygon should be a valid trimesh")
}

fn geo_line_string_to_parry_polyline(line_string: &LineString) -> Vec<Point2<f64>> {
    line_string
        .points_ccw()
        .skip(1) // Skip duplicate first point
        .map(|point| Point2::new(point.x(), point.y()))
        .collect()
}
