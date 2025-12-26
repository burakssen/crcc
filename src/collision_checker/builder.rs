use crate::collision_checker::CollisionChecker;
use crate::collision_checker::engine::parry::ParryEngine;
use crate::collision_object::StaticCollisionObject;
use crate::collision_object::simple::SimpleCollisionObject;
use geo::{Area, BooleanOps, ConvexHull, HasDimensions, Polygon, Simplify, Winding, unary_union};
use itertools::{Itertools, chain};

#[derive(Clone, Debug, Default)]
pub struct CollisionCheckerBuilder {
    static_obstacles: Vec<StaticCollisionObject>,
}

impl CollisionCheckerBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_static_obstacle(
        mut self,
        collision_object: impl Into<StaticCollisionObject>,
    ) -> Self {
        self.static_obstacles.push(collision_object.into());
        self
    }

    pub fn with_road_boundary_obstacle(self, lanelets: &[Polygon]) -> Self {
        let road_boundary = create_road_boundary_obstacle(lanelets);
        self.with_static_obstacle(road_boundary)
    }

    pub fn build_parry(self) -> CollisionChecker<ParryEngine> {
        CollisionChecker {
            static_obstacle: StaticCollisionObject::merge_all(self.static_obstacles).into(),
        }
    }
}

fn create_road_boundary_obstacle(lanelets: &[Polygon]) -> StaticCollisionObject {
    let road = unary_union(lanelets).simplify(0.01); // Simplify with 1 cm tolerance to reduce artifacts
    if road.is_empty() {
        return StaticCollisionObject::empty();
    }
    let road_convex_hull = road.convex_hull();

    // Construct outer half-spaces from convex hull
    let outer_halfspaces = road_convex_hull
        .exterior()
        .points_ccw()
        .tuples()
        .map(|(p1, p2)| SimpleCollisionObject::half_space_from_points(p1.into(), p2.into()));

    // Determine holes in the convex hull of the road
    let holes = road_convex_hull
        .difference(&road)
        .into_iter()
        // Ignore holes smaller than 10 cm² as these are most likely artifacts
        .filter(|hole| hole.unsigned_area() > 0.001)
        .map(SimpleCollisionObject::polygon);

    chain!(outer_halfspaces, holes).collect()
}
