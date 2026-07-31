use crate::collision_checker::engine::{CollisionEngine, EngineCollisionObject};
use crate::collision_checker::{
    CollisionChecker, SelectedCollisionChecker, SelectedCollisionCheckerInner,
};
use crate::collision_object::simple::SimpleCollisionObject;
use crate::collision_object::{CollisionObject, DynamicObstacle};
use crate::error::CrccError;
use crate::time::TimeStepSet;
use geo::{Area, BooleanOps, ConvexHull, HasDimensions, Polygon, Simplify, Winding, unary_union};
use itertools::Itertools;

/// Builds an immutable [`CollisionChecker`] with runtime engine selection.
#[derive(Clone, Debug)]
pub struct CollisionCheckerBuilder {
    static_obstacles: Vec<CollisionObject>,
    dynamic_obstacles: Vec<DynamicObstacle>,
}

impl CollisionCheckerBuilder {
    /// Creates an empty builder using [`CollisionEngine::default`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            static_obstacles: Vec::new(),
            dynamic_obstacles: Vec::new(),
        }
    }

    /// Adds geometry to the checker's merged static obstacle.
    #[must_use]
    pub fn with_static_obstacle(mut self, collision_object: impl Into<CollisionObject>) -> Self {
        self.static_obstacles.push(collision_object.into());
        self
    }

    /// Adds geometry representing the space outside `lanelets`.
    #[must_use]
    pub fn with_road_boundary(self, lanelets: &[Polygon]) -> Self {
        self.with_static_obstacle(road_boundary(lanelets))
    }

    /// Adds a dynamic trajectory to the checker.
    #[must_use]
    pub fn with_dynamic_obstacle(mut self, dynamic_obstacle: DynamicObstacle) -> Self {
        self.dynamic_obstacles.push(dynamic_obstacle);
        self
    }

    /// Builds a checker whose backend representation is selected by `E`.
    #[must_use]
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

    /// Builds a runtime-selected collision checker.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] when the selected engine's Cargo
    /// feature is not enabled.
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
            active_times.extend(dynamic_obstacle.active_times());
        }

        active_times
    }
}

impl Default for CollisionCheckerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

pub fn road_boundary(lanelets: &[Polygon]) -> CollisionObject {
    // Simplify with a 1 cm tolerance to reduce geometric artifacts.
    let road = unary_union(lanelets).simplify(0.01);

    if road.is_empty() {
        return CollisionObject::empty();
    }

    let road_convex_hull = road.convex_hull();

    let outer_half_spaces = road_convex_hull
        .exterior()
        .points_ccw()
        .tuple_windows()
        .filter_map(|(start_point, end_point)| {
            SimpleCollisionObject::half_space_from_points(start_point.into(), end_point.into()).ok()
        });

    let holes = road_convex_hull
        .difference(&road)
        .into_iter()
        // Ignore holes smaller than 10 cm² because they are likely artifacts.
        .filter(|hole| hole.unsigned_area() > 0.001)
        .filter_map(|hole| SimpleCollisionObject::polygon(hole).ok());

    outer_half_spaces.chain(holes).collect()
}
