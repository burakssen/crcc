use crate::collision_checker::CollisionChecker;
use crate::collision_checker::CollisionCheckerError;
use crate::collision_checker::engine::CollisionEngine;
use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_checker::selected::SelectedCollisionChecker;
use crate::collision_object::CollisionObject;
use crate::collision_object::simple::SimpleCollisionObject;
use crate::dynamic_obstacle::DynamicObstacle;
use crate::time::TimeStepSet;
use geo::{Area, BooleanOps, ConvexHull, HasDimensions, Polygon, Simplify, Winding, unary_union};
use itertools::{Itertools, chain};

#[derive(Clone, Debug, Default)]
pub struct CollisionCheckerBuilder {
    static_obstacles: Vec<CollisionObject>,
    dynamic_obstacles: Vec<DynamicObstacle>,
}

impl CollisionCheckerBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_static_obstacle(mut self, collision_object: impl Into<CollisionObject>) -> Self {
        self.static_obstacles.push(collision_object.into());
        self
    }

    pub fn with_road_boundary_obstacle(self, lanelets: &[Polygon]) -> Self {
        let road_boundary = create_road_boundary_obstacle(lanelets);
        self.with_static_obstacle(road_boundary)
    }

    pub fn with_dynamic_obstacle(mut self, dynamic_obstacle: DynamicObstacle) -> Self {
        self.dynamic_obstacles.push(dynamic_obstacle);
        self
    }

    pub fn build<E: EngineCollisionObject>(self) -> CollisionChecker<E> {
        let active_times = self.active_times();
        CollisionChecker {
            static_obstacle: CollisionObject::merge_all(self.static_obstacles).into(),
            dynamic_obstacles: self
                .dynamic_obstacles
                .into_iter()
                .map(DynamicObstacle::convert_repr)
                .collect(),
            active_times,
        }
    }

    pub fn build_with_engine(
        self,
        engine: CollisionEngine,
    ) -> Result<SelectedCollisionChecker, CollisionCheckerError> {
        match engine {
            #[cfg(feature = "parry")]
            CollisionEngine::Parry => Ok(SelectedCollisionChecker::Parry(self.build())),
            #[cfg(not(feature = "parry"))]
            CollisionEngine::Parry => Err(CollisionCheckerError::Unsupported),
            #[cfg(feature = "rhusics")]
            CollisionEngine::Rhusics => Ok(SelectedCollisionChecker::Rhusics(self.build())),
            #[cfg(not(feature = "rhusics"))]
            CollisionEngine::Rhusics => Err(CollisionCheckerError::Unsupported),
        }
    }

    fn active_times(&self) -> TimeStepSet {
        let mut active_times = TimeStepSet::new();
        for obs in &self.dynamic_obstacles {
            active_times.add(obs.active_times());
        }
        active_times
    }
}

fn create_road_boundary_obstacle(lanelets: &[Polygon]) -> CollisionObject {
    let road = unary_union(lanelets).simplify(0.01); // Simplify with 1 cm tolerance to reduce artifacts
    if road.is_empty() {
        return CollisionObject::empty();
    }
    let road_convex_hull = road.convex_hull();

    // Construct outer half-spaces from convex hull
    let outer_halfspaces = road_convex_hull
        .exterior()
        .points_ccw()
        .tuple_windows()
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
