use geo::{Area, BooleanOps, ConvexHull, Polygon, Winding, unary_union};
use itertools::Itertools;
use nalgebra::{Isometry2, Point2};
use parry2d_f64::shape::{ConvexPolygon, SharedShape, TriMesh};

use crate::polygon::PolygonCollisionObject;

pub fn create_road_boundary_obstacle(
    lanelets: &[Polygon],
) -> (Vec<(Isometry2<f64>, SharedShape)>, Vec<TriMesh>) {
    let road = unary_union(lanelets);
    let road_convex_hull = road.convex_hull();

    // Construct outer half-spaces from convex hull
    let road_convex_hull_parry = ConvexPolygon::from_convex_polyline(
        road_convex_hull
            .exterior()
            .points_ccw()
            .skip(1)
            .map(|point| Point2::new(point.x(), point.y()))
            .collect(),
    )
    .expect("Convex hull is a valid convex polygon");
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
                meshes.push(mesh);
            }
            Some(PolygonCollisionObject::ConvexPolygon(poly)) => {
                shapes.push((Isometry2::identity(), SharedShape::new(poly)));
            }
            None => {}
        }
    }

    (shapes, meshes)
}
