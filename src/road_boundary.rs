use crate::collision_object::CollisionObject;
use geo::{Area, BooleanOps, ConvexHull, HasDimensions, Polygon, Simplify, Winding, unary_union};
use itertools::{Itertools, chain};

pub fn create_road_boundary_obstacle(lanelets: &[Polygon]) -> Vec<CollisionObject> {
    let road = unary_union(lanelets).simplify(0.01); // Simplify with 1 cm tolerance to reduce artifacts
    if road.is_empty() {
        return Vec::new();
    }
    let road_convex_hull = road.convex_hull();

    // Construct outer half-spaces from convex hull
    let outer_halfspaces = road_convex_hull
        .exterior()
        .points_ccw()
        .tuples()
        .map(|(p1, p2)| CollisionObject::half_space_from_points(p1.into(), p2.into()));

    // Determine holes in the convex hull of the road
    let holes = road_convex_hull
        .difference(&road)
        .into_iter()
        // Ignore holes smaller than 10 cm² as these are most likely artifacts
        .filter(|hole| hole.unsigned_area() > 0.1)
        .map(CollisionObject::polygon);

    chain!(outer_halfspaces, holes).collect_vec()
}
