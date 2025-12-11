use geo::{Area, BooleanOps, ConvexHull, HasDimensions, Polygon, Simplify, unary_union};
use itertools::Itertools;
use nalgebra::Isometry2;
use parry2d_f64::shape::{SharedShape, TriMesh};

use crate::polygon::PolygonCollisionObject;

pub fn create_road_boundary_obstacle(
    lanelets: &[Polygon],
) -> (Vec<(Isometry2<f64>, SharedShape)>, Vec<TriMesh>) {
    let road = unary_union(lanelets).simplify(0.01); // Simplify with 1 cm tolerance to reduce artifacts
    if road.is_empty() {
        return (Vec::new(), Vec::new());
    }
    let road_convex_hull = road.convex_hull();

    // Construct outer half-spaces from convex hull
    let road_convex_hull_parry = match PolygonCollisionObject::new(&road_convex_hull) {
        Some(PolygonCollisionObject::ConvexPolygon(poly)) => poly,
        _ => unreachable!("Convex hull should be a valid convex polygon"),
    };
    let outer_halfspaces = itertools::izip!(
        road_convex_hull_parry.normals(),
        road_convex_hull_parry.points()
    )
    .map(|(n, p)| {
        (
            Isometry2::translation(p.x, p.y),
            SharedShape::halfspace(-*n),
        )
    })
    .collect_vec();

    // Determine holes in the convex hull of the road
    let difference = road_convex_hull.difference(&road);
    let mut shapes = outer_halfspaces;
    let mut meshes = Vec::new();
    for hole in difference
        .into_iter()
        .filter(|hole| hole.unsigned_area() > 0.1)
    // Ignore holes smaller than 10 cm² as these are most likely artifacts
    {
        match PolygonCollisionObject::new(&hole) {
            Some(PolygonCollisionObject::TriMesh(mesh)) => {
                meshes.push(*mesh);
            }
            Some(PolygonCollisionObject::ConvexPolygon(poly)) => {
                shapes.push((Isometry2::identity(), SharedShape::new(poly)));
            }
            None => {}
        }
    }

    (shapes, meshes)
}
