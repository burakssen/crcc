use crate::collision_checker::engine::CollisionEngine;
use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_checker::{
    CollisionChecker, SelectedCollisionChecker, SelectedCollisionCheckerInner,
};
use crate::collision_object::CollisionObject;
use crate::collision_object::DynamicObstacle;
use crate::collision_object::simple::SimpleCollisionObject;
use crate::error::CrccError;
use crate::time::TimeStepSet;
use geo::{Area, BooleanOps, ConvexHull, HasDimensions, Polygon, Simplify, Winding, unary_union};
use itertools::{Itertools, chain};

#[derive(Clone, Debug)]
/// Builds an immutable [`CollisionChecker`] with runtime engine selection.
pub struct CollisionCheckerBuilder {
    static_obstacles: Vec<CollisionObject>,
    dynamic_obstacles: Vec<DynamicObstacle>,
}

impl CollisionCheckerBuilder {
    /// Creates an empty builder using [`CollisionEngine::default`].
    pub fn new() -> Self {
        Self {
            static_obstacles: Vec::new(),
            dynamic_obstacles: Vec::new(),
        }
    }

    /// Adds geometry to the checker's merged static obstacle.
    pub fn with_static_obstacle(mut self, collision_object: impl Into<CollisionObject>) -> Self {
        self.static_obstacles.push(collision_object.into());
        self
    }

    /// Adds geometry representing the space outside `lanelets`.
    pub fn with_road_boundary(self, lanelets: &[Polygon]) -> Self {
        self.with_static_obstacle(road_boundary(lanelets))
    }

    /// Adds a dynamic trajectory to the checker.
    pub fn with_dynamic_obstacle(mut self, dynamic_obstacle: DynamicObstacle) -> Self {
        self.dynamic_obstacles.push(dynamic_obstacle);
        self
    }

    /// Builds a checker whose backend representation is selected by `E`.
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

    /// Builds a runtime-selected checker.
    pub fn build_with_engine(
        self,
        engine: CollisionEngine,
    ) -> Result<SelectedCollisionChecker, CrccError> {
        match engine {
            #[cfg(feature = "parry")]
            CollisionEngine::Parry => Ok(SelectedCollisionChecker::new(
                SelectedCollisionCheckerInner::Parry(Box::new(self.build())),
            )),
            #[cfg(not(feature = "parry"))]
            CollisionEngine::Parry => Err(CrccError::Unsupported),
            #[cfg(feature = "rhusics")]
            CollisionEngine::Rhusics => Ok(SelectedCollisionChecker::new(
                SelectedCollisionCheckerInner::Rhusics(Box::new(self.build())),
            )),
            #[cfg(not(feature = "rhusics"))]
            CollisionEngine::Rhusics => Err(CrccError::Unsupported),
            #[cfg(feature = "collide")]
            CollisionEngine::Collide => Ok(SelectedCollisionChecker::new(
                SelectedCollisionCheckerInner::Collide(Box::new(self.build())),
            )),
            #[cfg(not(feature = "collide"))]
            CollisionEngine::Collide => Err(CrccError::Unsupported),
        }
    }

    fn active_times(&self) -> TimeStepSet {
        let mut active_times = TimeStepSet::new();
        for dynamic_obstacle in &self.dynamic_obstacles {
            active_times.union(&dynamic_obstacle.active_times());
        }
        active_times
    }
}

impl Default for CollisionCheckerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

pub(crate) fn road_boundary(lanelets: &[Polygon]) -> CollisionObject {
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
        .map(|(start_point, end_point)| {
            SimpleCollisionObject::half_space_from_points(start_point.into(), end_point.into())
                .expect("lanelet boundary segments contain distinct finite points")
        });

    // Determine holes in the convex hull of the road
    let holes = road_convex_hull
        .difference(&road)
        .into_iter()
        // Ignore holes smaller than 10 cm² as these are most likely artifacts
        .filter(|hole| hole.unsigned_area() > 0.001)
        .filter_map(|hole| SimpleCollisionObject::polygon(hole).ok());

    chain!(outer_halfspaces, holes).collect()
}
