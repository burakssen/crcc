use geo::{
    Area, BooleanOps, ConvexHull, IsConvex, Orient, Polygon, Winding, orient::Direction,
    unary_union,
};
use itertools::Itertools;
use nalgebra::{Isometry2, Point2};
use parry2d_f64::shape::{Compound, ConvexPolygon, HalfSpace, SharedShape, TriMesh};

pub fn create_road_boundary_obstacle(lanelets: &[Polygon]) -> Vec<Compound> {
    let road = unary_union(lanelets);
    let road_convex_hull = road.convex_hull();
    let mut holes = road_convex_hull
        .difference(&road)
        .0
        .into_iter()
        .filter(|hole| hole.unsigned_area() > 0.001) // Ignore holes smaller than 10 cm² as these are most likely artifacts
        .collect_vec();
    let first_non_convex_idx = itertools::partition(holes.iter_mut(), |hole: &Polygon| {
        if !hole.interiors().is_empty() {
            panic!("Holes with interiors are not supported yet");
        }
        hole.exterior().is_convex()
    });
    let (convex_holes, non_convex_holes) = holes.split_at(first_non_convex_idx);

    // Construct outer half-spaces from convex hull
    let road_convex_hull =
        ConvexPolygon::from_convex_polyline(geo_poly_to_parry_polyline(&road_convex_hull))
            .expect("Convex hull is a valid convex polygon");
    let outer_halfspaces = itertools::izip!(road_convex_hull.normals(), road_convex_hull.points())
        .map(|(n, p)| (Isometry2::translation(p.x, p.y), HalfSpace::new(-*n)));

    // Construct convex polygons from convex holes
    let convex_hole_polygons = convex_holes.iter().map(|hole| {
        ConvexPolygon::from_convex_polyline(geo_poly_to_parry_polyline(hole))
            .expect("Convex hole is a valid convex polygon")
    });

    // Combine all convex shapes into a single compound
    let shared_shapes = itertools::chain!(
        outer_halfspaces.map(|(iso, hs)| (iso, SharedShape::new(hs))),
        convex_hole_polygons.map(|poly| (Isometry2::identity(), SharedShape::new(poly))),
    )
    .collect();
    let convex_shapes_compound = Compound::new(shared_shapes);

    // Construct compound from non-convex holes based on trimeshes
    let non_convex_hole_mesh = non_convex_holes
        .iter()
        .map(|hole| {
            TriMesh::from_polygon(geo_poly_to_parry_polyline(hole))
                .expect("Non-convex hole is a valid polygon")
        })
        .reduce(|mut acc, mesh| {
            acc.append(&mesh);
            acc
        })
        .and_then(|mesh| Compound::decompose_trimesh(&mesh));

    if let Some(non_convex_hole_mesh) = non_convex_hole_mesh {
        vec![convex_shapes_compound, non_convex_hole_mesh]
    } else {
        vec![convex_shapes_compound]
    }
}

fn geo_poly_to_parry_polyline(polygon: &Polygon) -> Vec<Point2<f64>> {
    let polygon = if polygon.exterior().is_ccw() {
        polygon
    } else {
        &polygon.orient(Direction::Default)
    };
    // dbg!(polygon);
    polygon
        .exterior()
        .points()
        .skip(1) // Skip duplicate first point
        .map(|point| Point2::new(point.x(), point.y()))
        .collect()
}
