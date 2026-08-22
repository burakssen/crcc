#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
use crate::collision_checker::SelectedCollisionCheckerInner;
use crate::collision_checker::engine::{CollisionEngine, EngineCollisionObject};
use crate::collision_checker::{CollisionChecker, SelectedCollisionChecker};
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
        return CollisionObject::full_space();
    }

    // ponytail: geometry we cannot represent must fail toward over-approximation,
    // so an unrepresentable edge or hole turns the whole boundary into full space.
    let road_convex_hull = road.convex_hull();
    let mut objects = Vec::new();

    for (start_point, end_point) in road_convex_hull.exterior().points_ccw().tuple_windows() {
        match SimpleCollisionObject::half_space_from_points(start_point.into(), end_point.into()) {
            Ok(half_space) => objects.push(half_space),
            Err(_) => return CollisionObject::full_space(),
        }
    }

    for hole in road_convex_hull.difference(&road) {
        // Ignore holes smaller than 10 cm² because they are likely artifacts.
        if hole.unsigned_area() <= 0.001 {
            continue;
        }
        match SimpleCollisionObject::polygon(hole) {
            Ok(polygon) => objects.push(polygon),
            Err(_) => return CollisionObject::full_space(),
        }
    }

    objects.into_iter().collect()
}

#[cfg(test)]
#[allow(clippy::panic_in_result_fn)]
mod tests {
    use super::road_boundary;
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    use crate::collision_checker::engine::CollisionEngine;
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    use crate::collision_object::CollisionObject;
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    use crate::error::CrccResult;
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    use geo::Polygon;
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    use glamx::DPose2;

    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    fn assert_road_points(
        lanelets: &[Polygon],
        inside: &[(f64, f64)],
        outside: &[(f64, f64)],
    ) -> CrccResult<()> {
        let boundary = road_boundary(lanelets);
        // Small circles exercise existing cross-engine APIs without point-shape support.
        let probe = CollisionObject::circle((0.0, 0.0), 0.1)?;

        for point in inside {
            assert!(!boundary.collides(
                &probe,
                DPose2::IDENTITY,
                DPose2::translation(point.0, point.1),
                CollisionEngine::default(),
            )?);
        }
        for point in outside {
            assert!(boundary.collides(
                &probe,
                DPose2::IDENTITY,
                DPose2::translation(point.0, point.1),
                CollisionEngine::default(),
            )?);
        }
        Ok(())
    }

    #[test]
    fn empty_road_has_full_space_boundary() {
        assert!(road_boundary(&[]).is_full_space());
    }

    #[test]
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    fn rectangular_road_boundary_separates_inside_from_outside() -> CrccResult<()> {
        let road = Polygon::new(
            vec![(0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0), (0.0, 0.0)].into(),
            Vec::new(),
        );

        assert_road_points(
            &[road],
            &[(1.0, 1.0), (5.0, 3.0)],
            &[(-1.0, 2.0), (3.0, 5.0)],
        )
    }

    #[test]
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    fn concave_road_boundary_rejects_convex_hull_notch() -> CrccResult<()> {
        let road = Polygon::new(
            vec![
                (0.0, 0.0),
                (6.0, 0.0),
                (6.0, 2.0),
                (2.0, 2.0),
                (2.0, 6.0),
                (0.0, 6.0),
                (0.0, 0.0),
            ]
            .into(),
            Vec::new(),
        );

        assert_road_points(
            &[road],
            &[(1.0, 5.0), (5.0, 1.0)],
            &[(4.0, 4.0), (7.0, 1.0)],
        )
    }

    #[test]
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    fn disconnected_road_boundary_rejects_gap() -> CrccResult<()> {
        let left = Polygon::new(
            vec![(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)].into(),
            Vec::new(),
        );
        let right = Polygon::new(
            vec![(5.0, 0.0), (7.0, 0.0), (7.0, 2.0), (5.0, 2.0), (5.0, 0.0)].into(),
            Vec::new(),
        );

        assert_road_points(
            &[left, right],
            &[(1.0, 1.0), (6.0, 1.0)],
            &[(3.5, 1.0), (8.0, 1.0)],
        )
    }
}
