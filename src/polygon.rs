use geo::{HasDimensions, IsConvex, LineString, Polygon, TriangulateEarcut, Winding};
use itertools::Itertools;
use nalgebra::Point2;
use parry2d_f64::shape::{Shape, TriMesh};

pub fn polygon_to_collision_shape(polygon: &Polygon) -> Option<Box<dyn Shape>> {
    if polygon.is_empty() {
        return None;
    }

    let has_holes = !polygon.interiors().is_empty();
    let is_convex = polygon.exterior().is_convex();

    match (has_holes, is_convex) {
        (false, true) => {
            let convex_poly = parry2d_f64::shape::ConvexPolygon::from_convex_polyline(
                geo_line_string_to_parry_polyline(polygon.exterior()),
            )
            .expect("Polygon should be a valid convex polygon");
            Some(Box::new(convex_poly))
        }
        (false, false) => {
            let trimesh =
                TriMesh::from_polygon(geo_line_string_to_parry_polyline(polygon.exterior()))
                    .expect("Polygon should be a valid polygon");
            Some(Box::new(trimesh))
        }
        _ => Some(Box::new(triangulate_polygon_with_holes(polygon))),
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
