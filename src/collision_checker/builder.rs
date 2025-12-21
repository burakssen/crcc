use crate::collision_checker::CollisionChecker;
use crate::collision_checker::engine::CollisionEngine;
use crate::collision_checker::engine::parry::ParryEngine;
use crate::collision_object::CollisionObject;
use crate::road_boundary::create_road_boundary_obstacle;
use geo::Polygon;

#[derive(Clone, Debug, Default)]
pub struct CollisionCheckerBuilder {
    static_obstacles: Vec<CollisionObject>,
}

impl CollisionCheckerBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_static_obstacle(mut self, collision_object: CollisionObject) -> Self {
        self.static_obstacles.push(collision_object);
        self
    }

    pub fn with_road_boundary_obstacle(mut self, lanelets: &[Polygon]) -> Self {
        let road_boundary = create_road_boundary_obstacle(lanelets);
        self.static_obstacles.extend(road_boundary);
        self
    }

    pub fn build_parry(self) -> CollisionChecker<ParryEngine> {
        CollisionChecker {
            engine: ParryEngine::from_collision_objects(self.static_obstacles),
        }
    }
}
