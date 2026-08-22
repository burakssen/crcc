use crate::collision_checker::ccd_collider::{CCDCollider, CCDColliderAt};
use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_object::dynamic::GenericDynamicObstacle;
use crate::collision_object::{CollisionObject, DynamicObstacle};
use crate::error::CrccError;
use crate::time::{TimeStep, TimeStepSet};
use glamx::DPose2;
use std::ops::RangeBounds;

mod builder;
mod ccd_collider;
pub mod engine;
#[cfg(feature = "rayon")]
pub mod parallel;

pub use builder::CollisionCheckerBuilder;
#[cfg(feature = "python_bindings")]
pub(crate) use builder::road_boundary;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// The first collision found by a checker query.
pub enum CollisionStatus {
    /// No static or dynamic obstacle collides in the requested window.
    NoCollision,
    /// The query collides with merged static geometry.
    CollidesStatic,
    /// The query collides with dynamic geometry at the contained time step.
    CollidesDynamic(TimeStep),
}

impl CollisionStatus {
    /// Returns whether this status represents any collision.
    #[must_use]
    pub const fn collides(self) -> bool {
        !matches!(self, Self::NoCollision)
    }
}

/// The result of a checker query.
pub type CollisionResult = Result<CollisionStatus, CrccError>;

pub struct CollisionChecker<E: EngineCollisionObject> {
    static_obstacle: E,
    dynamic_obstacles: Vec<GenericDynamicObstacle<E>>,
    active_times: TimeStepSet,
}

impl<E: EngineCollisionObject> CollisionChecker<E> {
    /// Checks a static obstacle against the scene geometry across all active times.
    ///
    /// # Errors
    ///
    /// Returns an error if an underlying collision-engine query fails.
    pub fn collides_static(&self, static_obstacle: &E) -> CollisionResult {
        self.collides_static_range(static_obstacle, DPose2::IDENTITY, ..)
    }

    /// Checks a dynamic obstacle against the scene geometry across all active times.
    ///
    /// # Errors
    ///
    /// Returns an error if an underlying collision-engine query fails.
    pub fn collides_dynamic(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E>,
    ) -> CollisionResult {
        self.collides_dynamic_range(dynamic_obstacle, ..)
    }

    /// Checks a static obstacle against the scene geometry at a specific time step.
    ///
    /// # Errors
    ///
    /// Returns an error if an underlying collision-engine query fails.
    pub fn collides_static_at(&self, static_obstacle: &E, time_step: TimeStep) -> CollisionResult {
        self.collides_static_range(static_obstacle, DPose2::IDENTITY, time_step..=time_step)
    }

    /// Checks a dynamic obstacle against the scene geometry at a specific time step.
    ///
    /// # Errors
    ///
    /// Returns an error if an underlying collision-engine query fails.
    pub fn collides_dynamic_at(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E>,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_dynamic_range(dynamic_obstacle, time_step..=time_step)
    }

    /// Checks a positioned static obstacle against the scene geometry across all active times.
    ///
    /// # Errors
    ///
    /// Returns an error if an underlying collision-engine query fails.
    pub fn collides_static_pos(&self, static_obstacle: &E, position: DPose2) -> CollisionResult {
        self.collides_static_range(static_obstacle, position, ..)
    }

    /// Checks a positioned static obstacle against the scene geometry at a specific time step.
    ///
    /// # Errors
    ///
    /// Returns an error if an underlying collision-engine query fails.
    pub fn collides_static_pos_at(
        &self,
        static_obstacle: &E,
        position: DPose2,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_static_range(static_obstacle, position, time_step..=time_step)
    }

    /// Checks a positioned static obstacle against the scene geometry within a specific time range.
    ///
    /// # Errors
    ///
    /// Returns an error if an underlying collision-engine query fails.
    pub fn collides_static_range(
        &self,
        static_obstacle: &E,
        position: DPose2,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        if self.check_collision_static_static(static_obstacle, position)? {
            return Ok(CollisionStatus::CollidesStatic);
        }

        // ponytail: BTreeSet::range panics on inverted ranges, so filter instead.
        let active_times: TimeStepSet = self
            .active_times
            .iter()
            .copied()
            .filter(|t| time_range.contains(t))
            .collect();
        for &time_step in &active_times {
            let ccd_collider = time_step
                .checked_succ()
                .is_some_and(|next| active_times.contains(&next))
                .then_some(Self::stationary_ccd_collider(static_obstacle, position));
            if self.static_query_collides_dynamic_at(
                static_obstacle,
                position,
                time_step,
                ccd_collider.as_ref(),
            )? {
                return Ok(CollisionStatus::CollidesDynamic(time_step));
            }
        }
        Ok(CollisionStatus::NoCollision)
    }

    /// Checks a dynamic obstacle against the scene geometry within a specific time range.
    ///
    /// # Errors
    ///
    /// Returns an error if an underlying collision-engine query fails.
    pub fn collides_dynamic_range(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E>,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        let obstacle_active_times = dynamic_obstacle.active_times();
        let active_times: TimeStepSet = obstacle_active_times
            .iter()
            .copied()
            .filter(|t| time_range.contains(t))
            .collect();
        for &time_step in &active_times {
            if self.dynamic_query_collides_at(
                dynamic_obstacle,
                time_step,
                time_step
                    .checked_succ()
                    .is_some_and(|next| active_times.contains(&next)),
            )? {
                return Ok(CollisionStatus::CollidesDynamic(time_step));
            }
        }
        Ok(CollisionStatus::NoCollision)
    }

    const fn stationary_ccd_collider(static_obstacle: &E, position: DPose2) -> CCDCollider<'_, E> {
        CCDCollider {
            shape: static_obstacle,
            position,
            next_position: position,
            convex_hull: static_obstacle,
            convex_hull_position: position,
        }
    }

    fn static_query_collides_dynamic_at(
        &self,
        static_obstacle: &E,
        position: DPose2,
        time_step: TimeStep,
        ccd_collider: Option<&CCDCollider<E>>,
    ) -> Result<bool, CrccError> {
        if let Some(ccd_collider) = ccd_collider
            && self.check_collision_dynamic_ccd(ccd_collider, time_step)?
        {
            return Ok(true);
        }
        self.check_collision_dynamic_static(static_obstacle, position, time_step)
    }

    fn dynamic_query_collides_at(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E>,
        time_step: TimeStep,
        next_step_active: bool,
    ) -> Result<bool, CrccError> {
        if next_step_active
            && let Some(ccd_collider) = dynamic_obstacle.ccd_collider_at(time_step)
            && (self.check_collision_static_ccd(&ccd_collider)?
                || (self.active_times.contains(&time_step)
                    && self.check_collision_dynamic_ccd(&ccd_collider, time_step)?))
        {
            return Ok(true);
        }

        let Some((shape, position)) = dynamic_obstacle.obstacle_at(time_step) else {
            return Ok(false);
        };
        Ok(self.check_collision_static_static(shape, position)?
            || (self.active_times.contains(&time_step)
                && self.check_collision_dynamic_static(shape, position, time_step)?))
    }

    fn check_collision_static_static(
        &self,
        static_obstacle: &E,
        position: DPose2,
    ) -> Result<bool, CrccError> {
        E::collides_at(
            &self.static_obstacle,
            DPose2::IDENTITY,
            static_obstacle,
            position,
        )
    }

    fn check_collision_dynamic_static(
        &self,
        static_obstacle: &E,
        position: DPose2,
        time_step: TimeStep,
    ) -> Result<bool, CrccError> {
        for obstacle in &self.dynamic_obstacles {
            let Some((obstacle_shape, obstacle_position)) = obstacle.obstacle_at(time_step) else {
                continue;
            };
            if E::collides_at(obstacle_shape, obstacle_position, static_obstacle, position)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn check_collision_static_ccd(&self, ccd_collider: &CCDCollider<E>) -> Result<bool, CrccError> {
        Ok(E::collides_at(
            &self.static_obstacle,
            DPose2::IDENTITY,
            ccd_collider.convex_hull,
            ccd_collider.convex_hull_position,
        )? && E::collides_continuous(
            &self.static_obstacle,
            DPose2::IDENTITY,
            DPose2::IDENTITY,
            ccd_collider.shape,
            ccd_collider.position,
            ccd_collider.next_position,
        )?)
    }

    fn check_collision_dynamic_ccd(
        &self,
        ccd_collider: &CCDCollider<E>,
        time_step: TimeStep,
    ) -> Result<bool, CrccError> {
        for obstacle in &self.dynamic_obstacles {
            if let Some(obstacle_ccd_collider) = obstacle.ccd_collider_at(time_step) {
                if E::collides_at(
                    // Broad-phase check with convex hull
                    obstacle_ccd_collider.convex_hull,
                    obstacle_ccd_collider.convex_hull_position,
                    ccd_collider.convex_hull,
                    ccd_collider.convex_hull_position,
                )? && E::collides_continuous(
                    // Narrow-phase check with CCD
                    obstacle_ccd_collider.shape,
                    obstacle_ccd_collider.position,
                    obstacle_ccd_collider.next_position,
                    ccd_collider.shape,
                    ccd_collider.position,
                    ccd_collider.next_position,
                )? {
                    return Ok(true);
                }
            } else if let Some((obstacle_shape, obstacle_position)) =
                obstacle.obstacle_at(time_step)
                && E::collides_at(
                    obstacle_shape,
                    obstacle_position,
                    ccd_collider.shape,
                    ccd_collider.position,
                )?
            {
                return Ok(true);
            }
        }
        Ok(false)
    }
}

#[cfg(all(test, any(feature = "parry", feature = "rhusics", feature = "collide")))]
#[allow(clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::collision_object::CollisionObject;
    use crate::collision_object::DynamicObstacle;
    use crate::collision_object::simple::SimpleCollisionObject;
    use geo::{Polygon, Rect};
    use std::f64::consts::FRAC_PI_2;

    fn engines() -> Vec<CollisionEngine> {
        vec![
            #[cfg(feature = "parry")]
            CollisionEngine::Parry,
            #[cfg(feature = "rhusics")]
            CollisionEngine::Rhusics,
            #[cfg(feature = "collide")]
            CollisionEngine::Collide,
        ]
    }

    #[test]
    fn duplicate_vertex_polygon_matches_clean_polygon_on_all_engines() {
        // Regression: a duplicated consecutive vertex once made the parry
        // backend reject every query against a scene containing such geometry.
        let clean = Polygon::new(
            vec![(0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)].into(),
            Vec::new(),
        );
        let duplicated = Polygon::new(
            vec![(0.0, 0.0), (4.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)].into(),
            Vec::new(),
        );
        let clean = CollisionObject::from(SimpleCollisionObject::polygon(clean).unwrap());
        let duplicated = CollisionObject::from(SimpleCollisionObject::polygon(duplicated).unwrap());
        let probe = CollisionObject::from(
            SimpleCollisionObject::rectangle(Rect::new((-1.0, -1.0), (1.0, 1.0)), 0.0).unwrap(),
        );

        for engine in engines() {
            let clean_checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(clean.clone())
                .build_with_engine(engine)
                .unwrap();
            let duplicated_checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(duplicated.clone())
                .build_with_engine(engine)
                .unwrap();

            for position in [DPose2::IDENTITY, DPose2::translation(100.0, 100.0)] {
                assert_eq!(
                    duplicated_checker
                        .collides_static_range(&probe, position, ..)
                        .unwrap(),
                    clean_checker
                        .collides_static_range(&probe, position, ..)
                        .unwrap(),
                    "{engine:?}, {position:?}"
                );
            }
        }
    }

    #[test]
    fn static_queries_respect_dynamic_obstacle_time_windows() {
        let dynamic_obstacle = DynamicObstacle::new(
            SimpleCollisionObject::circle((0.0, 0.0), 1.0)
                .unwrap()
                .into(),
            vec![
                DPose2::translation(10.0, 10.0),
                DPose2::translation(9.0, 9.0),
                DPose2::translation(10.0, 10.0),
                DPose2::translation(0.0, 0.0),
            ],
            TimeStep(0),
        )
        .unwrap();
        let query = CollisionObject::from(SimpleCollisionObject::circle((8.0, 8.0), 1.0).unwrap());

        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .with_dynamic_obstacle(dynamic_obstacle.clone())
                .build_with_engine(engine)
                .unwrap();

            for (time, expected) in [
                (0, CollisionStatus::NoCollision),
                (1, CollisionStatus::CollidesDynamic(TimeStep(1))),
                (2, CollisionStatus::NoCollision),
                (3, CollisionStatus::NoCollision),
                (4, CollisionStatus::NoCollision),
            ] {
                assert_eq!(
                    checker
                        .collides_static_range(
                            &query,
                            DPose2::IDENTITY,
                            TimeStep(time)..=TimeStep(time),
                        )
                        .unwrap(),
                    expected,
                    "{engine:?}, t={time}"
                );
            }
            assert_eq!(
                checker
                    .collides_static_range(&query, DPose2::IDENTITY, ..)
                    .unwrap(),
                CollisionStatus::CollidesDynamic(TimeStep(0)),
                "{engine:?}"
            );
            assert_eq!(
                checker
                    .collides_static_range(&query, DPose2::IDENTITY, TimeStep(2)..=TimeStep(4))
                    .unwrap(),
                CollisionStatus::CollidesDynamic(TimeStep(2)),
                "{engine:?}"
            );
        }
    }

    #[test]
    fn continuous_dynamic_checks_use_shape_casting() {
        let dynamic_obstacle = DynamicObstacle::new(
            SimpleCollisionObject::circle((0.0, 0.0), 1.0)
                .unwrap()
                .into(),
            vec![
                DPose2::translation(10.0, 10.0),
                DPose2::translation(9.0, 9.0),
                DPose2::translation(10.0, 10.0),
                DPose2::translation(0.0, 0.0),
            ],
            TimeStep(0),
        )
        .unwrap();
        let moving_query = DynamicObstacle::new(
            SimpleCollisionObject::circle((0.0, 0.0), 1.0)
                .unwrap()
                .into(),
            vec![
                DPose2::translation(5.0, 5.0),
                DPose2::translation(15.0, -5.0),
            ],
            TimeStep(2),
        )
        .unwrap();

        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .with_dynamic_obstacle(dynamic_obstacle.clone())
                .build_with_engine(engine)
                .unwrap();
            assert_eq!(
                checker.collides_dynamic(&moving_query).unwrap(),
                CollisionStatus::NoCollision,
                "{engine:?}"
            );
        }
    }

    #[test]
    fn dynamic_query_uses_static_ccd_narrow_phase() {
        let static_obstacle =
            CollisionObject::from(SimpleCollisionObject::circle((-1.9, 0.3), 0.1).unwrap());
        let moving_shape = CollisionObject::from(
            SimpleCollisionObject::rectangle(Rect::new((-2.0, -0.1), (2.0, 0.1)), 0.0).unwrap(),
        );
        let start = DPose2::IDENTITY;
        let end = DPose2::new((0.0, 0.0).into(), FRAC_PI_2);
        let moving_query =
            DynamicObstacle::new(moving_shape, vec![start, end], TimeStep(0)).unwrap();

        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(static_obstacle.clone())
                .build_with_engine(engine)
                .unwrap();
            let expected = match engine {
                CollisionEngine::Parry => CollisionStatus::NoCollision,
                CollisionEngine::Rhusics | CollisionEngine::Collide => {
                    CollisionStatus::CollidesDynamic(TimeStep(0))
                }
            };
            assert_eq!(
                checker.collides_dynamic(&moving_query).unwrap(),
                expected,
                "{engine:?}"
            );
        }
    }

    #[test]
    fn varying_shape_gaps_preserve_occupancy_without_phantom_motion() {
        let moving_shape = CollisionObject::circle((0.0, 0.0), 0.25).unwrap();
        let separated_query = DynamicObstacle::time_variant(
            vec![moving_shape.clone(), CollisionObject::empty()],
            vec![DPose2::translation(-2.0, 0.0), DPose2::IDENTITY],
            TimeStep::ZERO,
        )
        .unwrap();
        let colliding_query = DynamicObstacle::time_variant(
            vec![moving_shape, CollisionObject::empty()],
            vec![DPose2::IDENTITY; 2],
            TimeStep::ZERO,
        )
        .unwrap();

        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 0.25).unwrap())
                .build_with_engine(engine)
                .unwrap();

            assert_eq!(
                checker.collides_dynamic(&separated_query).unwrap(),
                CollisionStatus::NoCollision,
                "{engine:?}",
            );
            assert_eq!(
                checker.collides_dynamic(&colliding_query).unwrap(),
                CollisionStatus::CollidesDynamic(TimeStep::ZERO),
                "{engine:?}",
            );
        }
    }

    #[test]
    fn dynamic_compound_query_matches_expanded_children() {
        let static_compound = CollisionObject::merge_all([
            CollisionObject::from(SimpleCollisionObject::circle((0.0, 0.0), 0.5).unwrap()),
            CollisionObject::from(
                SimpleCollisionObject::rectangle(Rect::new((4.0, -0.5), (5.0, 0.5)), 0.0).unwrap(),
            ),
        ]);
        let query_parts = vec![
            CollisionObject::from(SimpleCollisionObject::circle((-0.75, 0.0), 0.25).unwrap()),
            CollisionObject::from(
                SimpleCollisionObject::rectangle(Rect::new((0.5, -0.2), (1.0, 0.2)), 0.0).unwrap(),
            ),
        ];
        let positions = vec![
            DPose2::translation(-3.0, 0.0),
            DPose2::translation(1.0, 0.0),
            DPose2::translation(8.0, 0.0),
        ];
        let compound = DynamicObstacle::new(
            CollisionObject::merge_all(query_parts.clone()),
            positions.clone(),
            TimeStep(0),
        )
        .unwrap();

        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(static_compound.clone())
                .build_with_engine(engine)
                .unwrap();
            let expanded = query_parts
                .iter()
                .cloned()
                .map(|part| DynamicObstacle::new(part, positions.clone(), TimeStep(0)).unwrap())
                .map(|obstacle| checker.collides_dynamic(&obstacle).unwrap())
                .find(|status| status.collides())
                .unwrap_or(CollisionStatus::NoCollision);

            assert_eq!(
                expanded,
                CollisionStatus::CollidesDynamic(TimeStep(0)),
                "{engine:?}"
            );
            assert_eq!(
                checker.collides_dynamic(&compound).unwrap(),
                CollisionStatus::CollidesDynamic(TimeStep(0)),
                "{engine:?}"
            );
        }
    }

    #[test]
    fn rotated_rectangles_collide_at_expected_extents() {
        let rect1 =
            SimpleCollisionObject::rectangle(Rect::new((0.0, 0.0), (2.0, 1.0)), 0.0).unwrap();
        let rect2 =
            SimpleCollisionObject::rectangle(Rect::new((0.0, 1.1), (2.0, 2.1)), FRAC_PI_2).unwrap();
        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(rect1.clone())
                .build_with_engine(engine)
                .unwrap();
            assert_eq!(
                checker
                    .collides_static(&CollisionObject::from(rect2.clone()))
                    .unwrap(),
                CollisionStatus::CollidesStatic,
                "{engine:?}"
            );
        }
    }

    #[test]
    fn full_space_query_always_collides() {
        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();
            assert_eq!(
                checker
                    .collides_static(&CollisionObject::from(SimpleCollisionObject::full_space()))
                    .unwrap(),
                CollisionStatus::CollidesStatic,
                "{engine:?}"
            );
        }
    }
}

use crate::collision_checker::engine::CollisionEngine;

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
const PARALLEL_QUERY_THRESHOLD: usize = 32;

pub(crate) enum SelectedCollisionCheckerInner {
    #[cfg(feature = "parry")]
    Parry(Box<CollisionChecker<crate::collision_checker::engine::parry::ParryCollisionObject>>),
    #[cfg(feature = "rhusics")]
    Rhusics(
        Box<
            CollisionChecker<crate::collision_checker::engine::rhusics::RhusicsCoreCollisionObject>,
        >,
    ),
    #[cfg(feature = "collide")]
    Collide(
        Box<CollisionChecker<crate::collision_checker::engine::collide::CollideCollisionObject>>,
    ),
}

#[derive(Clone)]
enum PreparedStaticQueryInner {
    #[cfg(feature = "parry")]
    Parry(Box<crate::collision_checker::engine::parry::ParryCollisionObject>),
    #[cfg(feature = "rhusics")]
    Rhusics(Box<crate::collision_checker::engine::rhusics::RhusicsCoreCollisionObject>),
    #[cfg(feature = "collide")]
    Collide(Box<crate::collision_checker::engine::collide::CollideCollisionObject>),
}

/// Geometry converted once for repeated queries against a selected checker.
#[derive(Clone)]
pub struct PreparedStaticQuery(PreparedStaticQueryInner);

impl PreparedStaticQuery {
    /// Returns the backend representation stored by this query.
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    #[must_use]
    pub const fn engine(&self) -> CollisionEngine {
        match &self.0 {
            #[cfg(feature = "parry")]
            PreparedStaticQueryInner::Parry(_) => CollisionEngine::Parry,
            #[cfg(feature = "rhusics")]
            PreparedStaticQueryInner::Rhusics(_) => CollisionEngine::Rhusics,
            #[cfg(feature = "collide")]
            PreparedStaticQueryInner::Collide(_) => CollisionEngine::Collide,
        }
    }

    /// Returns the default backend when no collision backend is compiled in.
    #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
    #[must_use]
    pub fn engine(&self) -> CollisionEngine {
        let _ = &self.0;
        CollisionEngine::default()
    }
}

#[derive(Clone)]
enum PreparedDynamicQueryInner {
    #[cfg(feature = "parry")]
    Parry(
        Box<GenericDynamicObstacle<crate::collision_checker::engine::parry::ParryCollisionObject>>,
    ),
    #[cfg(feature = "rhusics")]
    Rhusics(
        Box<
            GenericDynamicObstacle<
                crate::collision_checker::engine::rhusics::RhusicsCoreCollisionObject,
            >,
        >,
    ),
    #[cfg(feature = "collide")]
    Collide(
        Box<
            GenericDynamicObstacle<
                crate::collision_checker::engine::collide::CollideCollisionObject,
            >,
        >,
    ),
}

/// A dynamic trajectory converted once for repeated selected-checker queries.
#[derive(Clone)]
pub struct PreparedDynamicQuery(PreparedDynamicQueryInner);

impl PreparedDynamicQuery {
    /// Returns the backend representation stored by this query.
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    #[must_use]
    pub const fn engine(&self) -> CollisionEngine {
        match &self.0 {
            #[cfg(feature = "parry")]
            PreparedDynamicQueryInner::Parry(_) => CollisionEngine::Parry,
            #[cfg(feature = "rhusics")]
            PreparedDynamicQueryInner::Rhusics(_) => CollisionEngine::Rhusics,
            #[cfg(feature = "collide")]
            PreparedDynamicQueryInner::Collide(_) => CollisionEngine::Collide,
        }
    }

    /// Returns the default backend when no collision backend is compiled in.
    #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
    #[must_use]
    pub fn engine(&self) -> CollisionEngine {
        let _ = &self.0;
        CollisionEngine::default()
    }
}

/// An immutable collision scene using one runtime-selected backend.
pub struct SelectedCollisionChecker(SelectedCollisionCheckerInner);

impl SelectedCollisionChecker {
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    pub(crate) const fn new(inner: SelectedCollisionCheckerInner) -> Self {
        Self(inner)
    }

    /// Converts fixed geometry to this checker's backend representation.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] when no collision backend is available.
    pub fn prepare_static(
        &self,
        query: &CollisionObject,
    ) -> Result<PreparedStaticQuery, CrccError> {
        #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
        let _ = query;
        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(_) => Ok(PreparedStaticQuery(
                PreparedStaticQueryInner::Parry(Box::new(query.clone().into())),
            )),
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(_) => Ok(PreparedStaticQuery(
                PreparedStaticQueryInner::Rhusics(Box::new(query.clone().into())),
            )),
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(_) => Ok(PreparedStaticQuery(
                PreparedStaticQueryInner::Collide(Box::new(query.clone().into())),
            )),
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => Err(CrccError::Unsupported),
        }
    }

    /// Converts a dynamic trajectory to this checker's backend representation.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] when no collision backend is available.
    pub fn prepare_dynamic(
        &self,
        query: &DynamicObstacle,
    ) -> Result<PreparedDynamicQuery, CrccError> {
        #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
        let _ = query;
        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(_) => Ok(PreparedDynamicQuery(
                PreparedDynamicQueryInner::Parry(Box::new(query.clone().convert_repr())),
            )),
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(_) => Ok(PreparedDynamicQuery(
                PreparedDynamicQueryInner::Rhusics(Box::new(query.clone().convert_repr())),
            )),
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(_) => Ok(PreparedDynamicQuery(
                PreparedDynamicQueryInner::Collide(Box::new(query.clone().convert_repr())),
            )),
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => Err(CrccError::Unsupported),
        }
    }

    /// Checks a static obstacle against the scene geometry across all active times.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] if no collision backend is available,
    /// or propagates an error from the selected backend.
    pub fn collides_static(&self, static_obstacle: &CollisionObject) -> CollisionResult {
        self.collides_static_range(static_obstacle, DPose2::IDENTITY, ..)
    }

    /// Checks a dynamic obstacle against the scene geometry across all active times.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] if no collision backend is available,
    /// or propagates an error from the selected backend.
    pub fn collides_dynamic(&self, dynamic_obstacle: &DynamicObstacle) -> CollisionResult {
        self.collides_dynamic_range(dynamic_obstacle, ..)
    }

    /// Checks a static obstacle against the scene geometry at a specific time step.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] if no collision backend is available,
    /// or propagates an error from the selected backend.
    pub fn collides_static_at(
        &self,
        static_obstacle: &CollisionObject,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_static_range(static_obstacle, DPose2::IDENTITY, time_step..=time_step)
    }

    /// Checks a dynamic obstacle against the scene geometry at a specific time step.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] if no collision backend is available,
    /// or propagates an error from the selected backend.
    pub fn collides_dynamic_at(
        &self,
        dynamic_obstacle: &DynamicObstacle,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_dynamic_range(dynamic_obstacle, time_step..=time_step)
    }

    /// Checks a positioned static obstacle against the scene geometry across all active times.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] if no collision backend is available,
    /// or propagates an error from the selected backend.
    pub fn collides_static_pos(
        &self,
        static_obstacle: &CollisionObject,
        position: DPose2,
    ) -> CollisionResult {
        self.collides_static_range(static_obstacle, position, ..)
    }

    /// Checks a positioned static obstacle against the scene geometry at a specific time step.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] if no collision backend is available,
    /// or propagates an error from the selected backend.
    pub fn collides_static_pos_at(
        &self,
        static_obstacle: &CollisionObject,
        position: DPose2,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_static_range(static_obstacle, position, time_step..=time_step)
    }

    /// Checks a positioned fixed shape against static and dynamic scene geometry.
    ///
    /// `time_range` limits dynamic-obstacle checks. Static geometry is always
    /// checked. The first dynamic collision time is returned.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] if no collision backend is available,
    /// or propagates an error from the selected backend.
    pub fn collides_static_range(
        &self,
        static_obstacle: &CollisionObject,
        position: DPose2,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        let prepared = self.prepare_static(static_obstacle)?;
        self.collides_static_prepared_range(&prepared, position, time_range)
    }

    /// Checks prepared fixed geometry across all active times.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] when the query was prepared for a
    /// different backend, or propagates a backend query error.
    pub fn collides_static_prepared(&self, query: &PreparedStaticQuery) -> CollisionResult {
        self.collides_static_prepared_range(query, DPose2::IDENTITY, ..)
    }

    /// Checks prepared fixed geometry at a pose within a time range.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] when the query was prepared for a
    /// different backend, or propagates a backend query error.
    pub fn collides_static_prepared_range(
        &self,
        query: &PreparedStaticQuery,
        position: DPose2,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
        let _ = (position, &time_range);
        match (&self.0, &query.0) {
            #[cfg(feature = "parry")]
            (
                SelectedCollisionCheckerInner::Parry(checker),
                PreparedStaticQueryInner::Parry(query),
            ) => checker.collides_static_range(query, position, time_range),
            #[cfg(feature = "rhusics")]
            (
                SelectedCollisionCheckerInner::Rhusics(checker),
                PreparedStaticQueryInner::Rhusics(query),
            ) => checker.collides_static_range(query, position, time_range),
            #[cfg(feature = "collide")]
            (
                SelectedCollisionCheckerInner::Collide(checker),
                PreparedStaticQueryInner::Collide(query),
            ) => checker.collides_static_range(query, position, time_range),
            #[cfg(any(
                not(any(feature = "parry", feature = "rhusics", feature = "collide")),
                all(feature = "parry", feature = "rhusics"),
                all(feature = "parry", feature = "collide"),
                all(feature = "rhusics", feature = "collide"),
            ))]
            _ => Err(CrccError::Unsupported),
        }
    }

    /// Checks a moving obstacle against static and dynamic scene geometry.
    ///
    /// Continuous motion between adjacent active trajectory steps is included.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] if no collision backend is available,
    /// or propagates an error from the selected backend.
    pub fn collides_dynamic_range(
        &self,
        dynamic_obstacle: &DynamicObstacle,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        let prepared = self.prepare_dynamic(dynamic_obstacle)?;
        self.collides_dynamic_prepared_range(&prepared, time_range)
    }

    /// Checks a prepared dynamic trajectory across all active times.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] when the query was prepared for a
    /// different backend, or propagates a backend query error.
    pub fn collides_dynamic_prepared(&self, query: &PreparedDynamicQuery) -> CollisionResult {
        self.collides_dynamic_prepared_range(query, ..)
    }

    /// Checks a prepared dynamic trajectory within a time range.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] when the query was prepared for a
    /// different backend, or propagates a backend query error.
    pub fn collides_dynamic_prepared_range(
        &self,
        query: &PreparedDynamicQuery,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
        let _ = &time_range;
        match (&self.0, &query.0) {
            #[cfg(feature = "parry")]
            (
                SelectedCollisionCheckerInner::Parry(checker),
                PreparedDynamicQueryInner::Parry(query),
            ) => checker.collides_dynamic_range(query, time_range),
            #[cfg(feature = "rhusics")]
            (
                SelectedCollisionCheckerInner::Rhusics(checker),
                PreparedDynamicQueryInner::Rhusics(query),
            ) => checker.collides_dynamic_range(query, time_range),
            #[cfg(feature = "collide")]
            (
                SelectedCollisionCheckerInner::Collide(checker),
                PreparedDynamicQueryInner::Collide(query),
            ) => checker.collides_dynamic_range(query, time_range),
            #[cfg(any(
                not(any(feature = "parry", feature = "rhusics", feature = "collide")),
                all(feature = "parry", feature = "rhusics"),
                all(feature = "parry", feature = "collide"),
                all(feature = "rhusics", feature = "collide"),
            ))]
            _ => Err(CrccError::Unsupported),
        }
    }

    /// Returns the backend selected when this checker was built.
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    #[must_use]
    pub const fn engine(&self) -> CollisionEngine {
        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(_) => CollisionEngine::Parry,
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(_) => CollisionEngine::Rhusics,
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(_) => CollisionEngine::Collide,
        }
    }

    /// Returns the default backend when no collision backend is compiled in.
    #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
    #[must_use]
    pub fn engine(&self) -> CollisionEngine {
        CollisionEngine::default()
    }

    #[cfg(feature = "rayon")]
    /// Checks fixed-shape queries in a batch, preserving input order.
    ///
    /// Small batches run sequentially; larger batches use Rayon's active pool.
    #[must_use]
    pub fn collides_static_batch(
        &self,
        positioned_static_obstacles: &[(CollisionObject, DPose2)],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult> {
        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(checker) => {
                collides_static_batch(checker, positioned_static_obstacles, time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(checker) => {
                collides_static_batch(checker, positioned_static_obstacles, time_range)
            }
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(checker) => {
                collides_static_batch(checker, positioned_static_obstacles, time_range)
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => positioned_static_obstacles
                .iter()
                .map(|_| Err(CrccError::Unsupported))
                .collect(),
        }
    }

    #[cfg(feature = "rayon")]
    /// Checks dynamic queries in a batch, preserving input order.
    ///
    /// Small batches run sequentially; larger batches use Rayon's active pool.
    #[must_use]
    pub fn collides_dynamic_batch(
        &self,
        dynamic_obstacles: &[DynamicObstacle],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult> {
        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(checker) => {
                collides_dynamic_batch(checker, dynamic_obstacles, time_range)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(checker) => {
                collides_dynamic_batch(checker, dynamic_obstacles, time_range)
            }
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(checker) => {
                collides_dynamic_batch(checker, dynamic_obstacles, time_range)
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => dynamic_obstacles
                .iter()
                .map(|_| Err(CrccError::Unsupported))
                .collect(),
        }
    }

    #[cfg(feature = "rayon")]
    /// Checks multiple positioned static obstacles in parallel using Rayon.
    #[must_use]
    pub fn par_static(
        &self,
        positioned_static_obstacles: &[(CollisionObject, DPose2)],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult> {
        self.collides_static_batch(positioned_static_obstacles, time_range)
    }

    #[cfg(feature = "rayon")]
    /// Checks multiple dynamic obstacles in parallel using Rayon.
    #[must_use]
    pub fn par_dynamic(
        &self,
        dynamic_obstacles: &[DynamicObstacle],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult> {
        self.collides_dynamic_batch(dynamic_obstacles, time_range)
    }
}

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
fn collides_static_batch<E: EngineCollisionObject + Send + Sync>(
    checker: &CollisionChecker<E>,
    positioned_static_obstacles: &[(CollisionObject, DPose2)],
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
) -> Vec<CollisionResult> {
    use crate::collision_checker::parallel::ParallelCollisionChecker;
    use rayon::prelude::*;

    let converted = positioned_static_obstacles
        .iter()
        .map(|(obstacle, position)| (E::from(obstacle.clone()), *position))
        .collect::<Vec<_>>();

    if converted.len() < PARALLEL_QUERY_THRESHOLD {
        return converted
            .iter()
            .map(|(obstacle, position)| {
                checker.collides_static_range(obstacle, *position, time_range.clone())
            })
            .collect();
    }

    checker.collides_static_batch(
        converted
            .par_iter()
            .map(|(obstacle, position)| (obstacle, *position)),
        time_range,
    )
}

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
fn collides_dynamic_batch<E: EngineCollisionObject + Send + Sync>(
    checker: &CollisionChecker<E>,
    dynamic_obstacles: &[DynamicObstacle],
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
) -> Vec<CollisionResult> {
    use crate::collision_checker::parallel::ParallelCollisionChecker;
    use rayon::prelude::*;

    let converted = dynamic_obstacles
        .iter()
        .cloned()
        .map(DynamicObstacle::convert_repr)
        .collect::<Vec<GenericDynamicObstacle<E>>>();

    if converted.len() < PARALLEL_QUERY_THRESHOLD {
        return converted
            .iter()
            .map(|obstacle| checker.collides_dynamic_range(obstacle, time_range.clone()))
            .collect();
    }

    checker.collides_dynamic_batch(converted.par_iter(), time_range)
}

#[cfg(all(
    test,
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
#[allow(clippy::panic, clippy::unwrap_used)]
mod selected_tests {
    use super::*;
    use crate::collision_checker::CollisionCheckerBuilder;
    use crate::collision_object::simple::SimpleCollisionObject;

    fn engines() -> Vec<CollisionEngine> {
        vec![
            #[cfg(feature = "parry")]
            CollisionEngine::Parry,
            #[cfg(feature = "rhusics")]
            CollisionEngine::Rhusics,
            #[cfg(feature = "collide")]
            CollisionEngine::Collide,
        ]
    }

    const fn assert_send_sync<T: Send + Sync>() {}

    #[test]
    fn backend_objects_are_send_and_sync() {
        #[cfg(feature = "parry")]
        assert_send_sync::<crate::collision_checker::engine::parry::ParryCollisionObject>();
        #[cfg(feature = "rhusics")]
        assert_send_sync::<crate::collision_checker::engine::rhusics::RhusicsCoreCollisionObject>();
        #[cfg(feature = "collide")]
        assert_send_sync::<crate::collision_checker::engine::collide::CollideCollisionObject>();
    }

    #[test]
    fn prepared_queries_match_regular_queries() {
        let query = CollisionObject::circle((0.0, 0.0), 0.25).unwrap();
        let dynamic = DynamicObstacle::new(
            query.clone(),
            vec![DPose2::translation(4.0, 0.0), DPose2::translation(0.5, 0.0)],
            TimeStep(5),
        )
        .unwrap();

        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();
            let prepared_static = checker.prepare_static(&query).unwrap();
            let prepared_dynamic = checker.prepare_dynamic(&dynamic).unwrap();

            assert_eq!(prepared_static.engine(), engine);
            assert_eq!(prepared_dynamic.engine(), engine);
            assert_eq!(
                checker
                    .collides_static_prepared_range(
                        &prepared_static,
                        DPose2::translation(0.5, 0.0),
                        ..,
                    )
                    .unwrap(),
                CollisionStatus::CollidesStatic,
            );
            assert_eq!(
                checker
                    .collides_dynamic_prepared(&prepared_dynamic)
                    .unwrap(),
                CollisionStatus::CollidesDynamic(TimeStep(5)),
            );
            assert_eq!(
                checker
                    .collides_static_pos(&query, DPose2::translation(0.5, 0.0))
                    .unwrap(),
                CollisionStatus::CollidesStatic,
            );
            assert_eq!(
                checker.collides_dynamic(&dynamic).unwrap(),
                CollisionStatus::CollidesDynamic(TimeStep(5)),
            );
        }
    }

    #[cfg(all(feature = "parry", feature = "rhusics"))]
    #[test]
    fn prepared_queries_reject_engine_mismatch() {
        let query = CollisionObject::circle((0.0, 0.0), 0.25).unwrap();
        let parry = CollisionCheckerBuilder::new()
            .build_with_engine(CollisionEngine::Parry)
            .unwrap();
        let rhusics = CollisionCheckerBuilder::new()
            .build_with_engine(CollisionEngine::Rhusics)
            .unwrap();
        let dynamic = DynamicObstacle::new(
            query.clone(),
            vec![DPose2::translation(2.0, 0.0), DPose2::IDENTITY],
            TimeStep::ZERO,
        )
        .unwrap();
        let prepared_static = parry.prepare_static(&query).unwrap();
        let prepared_dynamic = parry.prepare_dynamic(&dynamic).unwrap();

        assert_eq!(
            rhusics.collides_static_prepared(&prepared_static),
            Err(CrccError::Unsupported),
        );
        assert_eq!(
            rhusics.collides_dynamic_prepared(&prepared_dynamic),
            Err(CrccError::Unsupported),
        );
    }

    #[test]
    fn parallel_static_matches_sequential_around_threshold() {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(2)
            .build()
            .unwrap();
        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();

            for count in [
                PARALLEL_QUERY_THRESHOLD - 1,
                PARALLEL_QUERY_THRESHOLD,
                PARALLEL_QUERY_THRESHOLD + 1,
            ] {
                let queries = (0..count)
                    .map(|index| {
                        let x = if index % 2 == 0 { 0.5 } else { 10.0 };
                        (
                            CollisionObject::circle((0.0, 0.0), 0.25).unwrap(),
                            DPose2::translation(x, 0.0),
                        )
                    })
                    .collect::<Vec<_>>();
                let sequential = queries
                    .iter()
                    .map(|(query, position)| checker.collides_static_range(query, *position, ..))
                    .collect::<Vec<_>>();
                let expected = (0..count)
                    .map(|index| {
                        Ok(if index % 2 == 0 {
                            CollisionStatus::CollidesStatic
                        } else {
                            CollisionStatus::NoCollision
                        })
                    })
                    .collect::<Vec<_>>();

                assert_eq!(sequential, expected, "{engine:?}, {count}");
                assert_eq!(
                    pool.install(|| checker.collides_static_batch(&queries, ..)),
                    expected,
                    "{engine:?}, {count}"
                );
            }
        }
    }

    #[test]
    fn parallel_dynamic_matches_sequential_around_threshold() {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(2)
            .build()
            .unwrap();
        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();

            for count in [
                PARALLEL_QUERY_THRESHOLD - 1,
                PARALLEL_QUERY_THRESHOLD,
                PARALLEL_QUERY_THRESHOLD + 1,
            ] {
                let queries = (0..count)
                    .map(|index| {
                        let x = if index % 2 == 0 { 4.0 } else { 10.0 };
                        DynamicObstacle::new(
                            CollisionObject::circle((0.0, 0.0), 0.25).unwrap(),
                            vec![DPose2::translation(x, 0.0), DPose2::translation(0.5, 0.0)],
                            TimeStep(5),
                        )
                        .unwrap()
                    })
                    .collect::<Vec<_>>();
                let sequential = queries
                    .iter()
                    .map(|query| checker.collides_dynamic_range(query, TimeStep(5)..=TimeStep(6)))
                    .collect::<Vec<_>>();
                let expected = vec![Ok(CollisionStatus::CollidesDynamic(TimeStep(5))); count];

                assert_eq!(sequential, expected, "{engine:?}, {count}");
                assert_eq!(
                    pool.install(
                        || checker.collides_dynamic_batch(&queries, TimeStep(5)..=TimeStep(6))
                    ),
                    expected,
                    "{engine:?}, {count}"
                );
            }
        }
    }

    #[test]
    fn inverted_time_ranges_are_empty_instead_of_panicking() {
        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();

            let query = CollisionObject::circle((10.0, 0.0), 0.25).unwrap();
            let dynamic = DynamicObstacle::new(
                query.clone(),
                vec![DPose2::translation(10.0, 0.0)],
                TimeStep(5),
            )
            .unwrap();

            assert_eq!(
                checker.collides_static_range(&query, DPose2::IDENTITY, TimeStep(3)..TimeStep(1)),
                Ok(CollisionStatus::NoCollision),
                "{engine:?}"
            );
            assert_eq!(
                checker.collides_dynamic_range(&dynamic, TimeStep(3)..=TimeStep(1)),
                Ok(CollisionStatus::NoCollision),
                "{engine:?}"
            );
        }
    }
}
