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

        // Iterate the cached scene union directly; rebuilding a filtered set per query
        // made batch bookkeeping compete with the collision kernel.
        for &time_step in self.active_times.iter().filter(|t| time_range.contains(t)) {
            let ccd_collider = time_step
                .checked_succ()
                .is_some_and(|next| self.active_times.contains(&next) && time_range.contains(&next))
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

    #[cfg(feature = "rayon")]
    fn collides_static_active_times(
        &self,
        static_obstacle: &E,
        position: DPose2,
        active_times: &[TimeStep],
    ) -> CollisionResult {
        if self.check_collision_static_static(static_obstacle, position)? {
            return Ok(CollisionStatus::CollidesStatic);
        }

        for (index, &time_step) in active_times.iter().enumerate() {
            let ccd_collider = index
                .checked_add(1)
                .and_then(|next_index| active_times.get(next_index))
                .and_then(|next| time_step.checked_succ().filter(|step| step == next))
                .map(|_| Self::stationary_ccd_collider(static_obstacle, position));
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
        let Some((active_start, active_end)) = dynamic_obstacle.active_time_bounds() else {
            return Ok(CollisionStatus::NoCollision);
        };
        for time_step in TimeStep::iter_intersection(time_range, active_start, active_end) {
            if self.dynamic_query_collides_at(
                dynamic_obstacle,
                time_step,
                time_step
                    .checked_succ()
                    .is_some_and(|next| next <= active_end),
            )? {
                return Ok(CollisionStatus::CollidesDynamic(time_step));
            }
        }
        Ok(CollisionStatus::NoCollision)
    }

    #[cfg(feature = "rayon")]
    fn active_times_in(&self, time_range: impl RangeBounds<TimeStep>) -> Vec<TimeStep> {
        self.active_times
            .iter()
            .copied()
            .filter(|time_step| time_range.contains(time_step))
            .collect()
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
///
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

    #[cfg(feature = "rayon")]
    /// Checks one prepared fixed query at multiple poses, preserving pose order.
    /// The caller selects sequential or Rayon execution with `parallel`.
    #[must_use]
    pub fn collides_static_prepared_batch(
        &self,
        query: &PreparedStaticQuery,
        positions: &[DPose2],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
        parallel: bool,
    ) -> Vec<CollisionResult> {
        match (&self.0, &query.0) {
            #[cfg(feature = "parry")]
            (
                SelectedCollisionCheckerInner::Parry(checker),
                PreparedStaticQueryInner::Parry(query),
            ) => collides_prepared_static_batch(checker, query, positions, time_range, parallel),
            #[cfg(feature = "rhusics")]
            (
                SelectedCollisionCheckerInner::Rhusics(checker),
                PreparedStaticQueryInner::Rhusics(query),
            ) => collides_prepared_static_batch(checker, query, positions, time_range, parallel),
            #[cfg(feature = "collide")]
            (
                SelectedCollisionCheckerInner::Collide(checker),
                PreparedStaticQueryInner::Collide(query),
            ) => collides_prepared_static_batch(checker, query, positions, time_range, parallel),
            _ => positions
                .iter()
                .map(|_| Err(CrccError::Unsupported))
                .collect(),
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

    #[cfg(feature = "rayon")]
    /// Checks prepared dynamic queries in input order.
    /// The caller selects sequential or Rayon execution with `parallel`.
    #[must_use]
    pub fn collides_dynamic_prepared_batch(
        &self,
        queries: &[PreparedDynamicQuery],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
        parallel: bool,
    ) -> Vec<CollisionResult> {
        use rayon::prelude::*;

        let run = |query: &PreparedDynamicQuery| match (&self.0, &query.0) {
            #[cfg(feature = "parry")]
            (
                SelectedCollisionCheckerInner::Parry(checker),
                PreparedDynamicQueryInner::Parry(query),
            ) => checker.collides_dynamic_range(query, time_range.clone()),
            #[cfg(feature = "rhusics")]
            (
                SelectedCollisionCheckerInner::Rhusics(checker),
                PreparedDynamicQueryInner::Rhusics(query),
            ) => checker.collides_dynamic_range(query, time_range.clone()),
            #[cfg(feature = "collide")]
            (
                SelectedCollisionCheckerInner::Collide(checker),
                PreparedDynamicQueryInner::Collide(query),
            ) => checker.collides_dynamic_range(query, time_range.clone()),
            _ => Err(CrccError::Unsupported),
        };

        if parallel {
            queries.par_iter().map(run).collect()
        } else {
            queries.iter().map(run).collect()
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
    /// The caller selects sequential or Rayon execution with `parallel`.
    #[must_use]
    pub fn collides_static_batch(
        &self,
        positioned_static_obstacles: &[(CollisionObject, DPose2)],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
        parallel: bool,
    ) -> Vec<CollisionResult> {
        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(checker) => {
                collides_static_batch(checker, positioned_static_obstacles, time_range, parallel)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(checker) => {
                collides_static_batch(checker, positioned_static_obstacles, time_range, parallel)
            }
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(checker) => {
                collides_static_batch(checker, positioned_static_obstacles, time_range, parallel)
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
    /// The caller selects sequential or Rayon execution with `parallel`.
    #[must_use]
    pub fn collides_dynamic_batch(
        &self,
        dynamic_obstacles: &[DynamicObstacle],
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
        parallel: bool,
    ) -> Vec<CollisionResult> {
        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(checker) => {
                collides_dynamic_batch(checker, dynamic_obstacles, time_range, parallel)
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(checker) => {
                collides_dynamic_batch(checker, dynamic_obstacles, time_range, parallel)
            }
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(checker) => {
                collides_dynamic_batch(checker, dynamic_obstacles, time_range, parallel)
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => dynamic_obstacles
                .iter()
                .map(|_| Err(CrccError::Unsupported))
                .collect(),
        }
    }

    #[cfg(feature = "rayon")]
    /// Checks static queries that mix raw objects with prepared geometry.
    /// The caller selects sequential or Rayon execution with `parallel`.
    ///
    /// Prepared queries built for a different backend make every slot return
    /// [`CrccError::Unsupported`]; otherwise results preserve input order.
    #[must_use]
    pub fn collides_static_heterogeneous_batch<'a, I>(
        &self,
        sources: I,
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
        parallel: bool,
    ) -> Vec<CollisionResult>
    where
        I: IntoIterator<Item = (StaticBatchQuery<'a>, DPose2)>,
    {
        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(checker) =>
            {
                #[allow(unreachable_patterns)]
                collides_heterogeneous_static_batch(
                    checker,
                    sources,
                    time_range,
                    parallel,
                    |query| match &query.0 {
                        PreparedStaticQueryInner::Parry(inner) => Some(inner.as_ref()),
                        _ => None,
                    },
                )
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(checker) =>
            {
                #[allow(unreachable_patterns)]
                collides_heterogeneous_static_batch(
                    checker,
                    sources,
                    time_range,
                    parallel,
                    |query| match &query.0 {
                        PreparedStaticQueryInner::Rhusics(inner) => Some(inner.as_ref()),
                        _ => None,
                    },
                )
            }
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(checker) =>
            {
                #[allow(unreachable_patterns)]
                collides_heterogeneous_static_batch(
                    checker,
                    sources,
                    time_range,
                    parallel,
                    |query| match &query.0 {
                        PreparedStaticQueryInner::Collide(inner) => Some(inner.as_ref()),
                        _ => None,
                    },
                )
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => {
                let count = sources.into_iter().count();
                (0..count).map(|_| Err(CrccError::Unsupported)).collect()
            }
        }
    }

    #[cfg(feature = "rayon")]
    /// Checks dynamic queries that mix raw obstacles with prepared trajectories.
    /// The caller selects sequential or Rayon execution with `parallel`.
    ///
    /// Prepared queries built for a different backend make every slot return
    /// [`CrccError::Unsupported`]; otherwise results preserve input order.
    #[must_use]
    pub fn collides_dynamic_heterogeneous_batch<'a, I>(
        &self,
        sources: I,
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
        parallel: bool,
    ) -> Vec<CollisionResult>
    where
        I: IntoIterator<Item = DynamicBatchQuery<'a>>,
    {
        match &self.0 {
            #[cfg(feature = "parry")]
            SelectedCollisionCheckerInner::Parry(checker) =>
            {
                #[allow(unreachable_patterns)]
                collides_heterogeneous_dynamic_batch(
                    checker,
                    sources,
                    time_range,
                    parallel,
                    |query| match &query.0 {
                        PreparedDynamicQueryInner::Parry(inner) => Some(inner.as_ref()),
                        _ => None,
                    },
                )
            }
            #[cfg(feature = "rhusics")]
            SelectedCollisionCheckerInner::Rhusics(checker) =>
            {
                #[allow(unreachable_patterns)]
                collides_heterogeneous_dynamic_batch(
                    checker,
                    sources,
                    time_range,
                    parallel,
                    |query| match &query.0 {
                        PreparedDynamicQueryInner::Rhusics(inner) => Some(inner.as_ref()),
                        _ => None,
                    },
                )
            }
            #[cfg(feature = "collide")]
            SelectedCollisionCheckerInner::Collide(checker) =>
            {
                #[allow(unreachable_patterns)]
                collides_heterogeneous_dynamic_batch(
                    checker,
                    sources,
                    time_range,
                    parallel,
                    |query| match &query.0 {
                        PreparedDynamicQueryInner::Collide(inner) => Some(inner.as_ref()),
                        _ => None,
                    },
                )
            }
            #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
            _ => {
                let count = sources.into_iter().count();
                (0..count).map(|_| Err(CrccError::Unsupported)).collect()
            }
        }
    }
}

/// A static batch entry referencing either a raw object or prepared geometry.
#[cfg(feature = "rayon")]
pub enum StaticBatchQuery<'a> {
    /// A domain-level fixed shape converted per execution.
    Raw(&'a CollisionObject),
    /// Geometry already converted for the checker's backend.
    Prepared(&'a PreparedStaticQuery),
}

/// A dynamic batch entry referencing either a raw obstacle or a prepared trajectory.
#[cfg(feature = "rayon")]
pub enum DynamicBatchQuery<'a> {
    /// A domain-level moving obstacle converted per execution.
    Raw(&'a DynamicObstacle),
    /// A trajectory already converted for the checker's backend.
    Prepared(&'a PreparedDynamicQuery),
}

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
fn collides_static_batch<E: EngineCollisionObject + Send + Sync>(
    checker: &CollisionChecker<E>,
    positioned_static_obstacles: &[(CollisionObject, DPose2)],
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    parallel: bool,
) -> Vec<CollisionResult> {
    use rayon::prelude::*;

    if positioned_static_obstacles.is_empty() {
        return Vec::new();
    }
    let active_times = checker.active_times_in(time_range);
    if parallel {
        positioned_static_obstacles
            .par_iter()
            .map(|(obstacle, position)| {
                let converted = E::from(obstacle.clone());
                checker.collides_static_active_times(&converted, *position, &active_times)
            })
            .collect()
    } else {
        positioned_static_obstacles
            .iter()
            .map(|(obstacle, position)| {
                let converted = E::from(obstacle.clone());
                checker.collides_static_active_times(&converted, *position, &active_times)
            })
            .collect()
    }
}

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
fn collides_dynamic_batch<E: EngineCollisionObject + Send + Sync>(
    checker: &CollisionChecker<E>,
    dynamic_obstacles: &[DynamicObstacle],
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    parallel: bool,
) -> Vec<CollisionResult> {
    use rayon::prelude::*;

    if parallel {
        dynamic_obstacles
            .par_iter()
            .map(|obstacle| {
                let converted = obstacle.clone().convert_repr::<E>();
                checker.collides_dynamic_range(&converted, time_range.clone())
            })
            .collect()
    } else {
        dynamic_obstacles
            .iter()
            .map(|obstacle| {
                let converted = obstacle.clone().convert_repr::<E>();
                checker.collides_dynamic_range(&converted, time_range.clone())
            })
            .collect()
    }
}

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
fn collides_prepared_static_batch<E: EngineCollisionObject + Send + Sync>(
    checker: &CollisionChecker<E>,
    query: &E,
    positions: &[DPose2],
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    parallel: bool,
) -> Vec<CollisionResult> {
    use rayon::prelude::*;

    if positions.is_empty() {
        return Vec::new();
    }
    let active_times = checker.active_times_in(time_range);
    if parallel {
        positions
            .par_iter()
            .map(|position| checker.collides_static_active_times(query, *position, &active_times))
            .collect()
    } else {
        positions
            .iter()
            .map(|position| checker.collides_static_active_times(query, *position, &active_times))
            .collect()
    }
}

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
enum HeterogeneousStaticEntry<'a, E> {
    Raw(&'a CollisionObject),
    Prepared { repr: &'a E },
}

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
enum HeterogeneousDynamicEntry<'a, E> {
    Raw(&'a DynamicObstacle),
    Prepared(&'a GenericDynamicObstacle<E>),
}

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
fn collides_heterogeneous_static_batch<'a, E, F, I>(
    checker: &CollisionChecker<E>,
    sources: I,
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    parallel: bool,
    prepared_repr: F,
) -> Vec<CollisionResult>
where
    E: EngineCollisionObject + Send + Sync,
    F: Fn(&PreparedStaticQuery) -> Option<&E>,
    I: IntoIterator<Item = (StaticBatchQuery<'a>, DPose2)>,
{
    use rayon::prelude::*;

    let sources: Vec<(StaticBatchQuery<'a>, DPose2)> = sources.into_iter().collect();
    let total = sources.len();
    let mut entries: Vec<(HeterogeneousStaticEntry<'_, E>, DPose2)> = Vec::with_capacity(total);
    let mut mismatched = false;
    for (source, position) in sources {
        let entry = match source {
            StaticBatchQuery::Raw(object) => HeterogeneousStaticEntry::Raw(object),
            StaticBatchQuery::Prepared(query) => {
                let Some(repr) = prepared_repr(query) else {
                    mismatched = true;
                    break;
                };
                HeterogeneousStaticEntry::Prepared { repr }
            }
        };
        entries.push((entry, position));
    }
    if mismatched {
        return (0..total).map(|_| Err(CrccError::Unsupported)).collect();
    }

    let active_times = checker.active_times_in(time_range);
    let run = |(entry, position): &(HeterogeneousStaticEntry<'_, E>, DPose2)| match entry {
        HeterogeneousStaticEntry::Raw(object) => {
            let converted = E::from((*object).clone());
            checker.collides_static_active_times(&converted, *position, &active_times)
        }
        HeterogeneousStaticEntry::Prepared { repr } => {
            checker.collides_static_active_times(repr, *position, &active_times)
        }
    };

    if parallel {
        entries.par_iter().map(run).collect()
    } else {
        entries.iter().map(run).collect()
    }
}

#[cfg(all(
    feature = "rayon",
    any(feature = "parry", feature = "rhusics", feature = "collide")
))]
fn collides_heterogeneous_dynamic_batch<'a, E, F, I>(
    checker: &CollisionChecker<E>,
    sources: I,
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    parallel: bool,
    prepared_repr: F,
) -> Vec<CollisionResult>
where
    E: EngineCollisionObject + Send + Sync,
    F: Fn(&PreparedDynamicQuery) -> Option<&GenericDynamicObstacle<E>>,
    I: IntoIterator<Item = DynamicBatchQuery<'a>>,
{
    use rayon::prelude::*;

    let sources: Vec<DynamicBatchQuery<'a>> = sources.into_iter().collect();
    let total = sources.len();
    let mut entries: Vec<HeterogeneousDynamicEntry<'_, E>> = Vec::with_capacity(total);
    let mut mismatched = false;
    for source in sources {
        let entry = match source {
            DynamicBatchQuery::Raw(obstacle) => HeterogeneousDynamicEntry::Raw(obstacle),
            DynamicBatchQuery::Prepared(query) => {
                let Some(repr) = prepared_repr(query) else {
                    mismatched = true;
                    break;
                };
                HeterogeneousDynamicEntry::Prepared(repr)
            }
        };
        entries.push(entry);
    }
    if mismatched {
        return (0..total).map(|_| Err(CrccError::Unsupported)).collect();
    }

    let run = |entry: &HeterogeneousDynamicEntry<'_, E>| match entry {
        HeterogeneousDynamicEntry::Raw(obstacle) => {
            let converted = (**obstacle).clone().convert_repr::<E>();
            checker.collides_dynamic_range(&converted, time_range.clone())
        }
        HeterogeneousDynamicEntry::Prepared(query) => {
            checker.collides_dynamic_range(query, time_range.clone())
        }
    };

    if parallel {
        entries.par_iter().map(run).collect()
    } else {
        entries.iter().map(run).collect()
    }
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

    #[test]
    fn prepared_static_batch_matches_scalar_queries() {
        let query = CollisionObject::circle((0.0, 0.0), 0.25).unwrap();
        let positions = vec![
            DPose2::translation(0.5, 0.0),
            DPose2::translation(4.0, 0.0),
            DPose2::translation(0.0, 0.0),
        ];

        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();
            let prepared = checker.prepare_static(&query).unwrap();
            let expected = positions
                .iter()
                .map(|position| checker.collides_static_pos(&query, *position))
                .collect::<Vec<_>>();

            assert_eq!(
                checker.collides_static_prepared_batch(&prepared, &positions, .., true),
                expected,
                "{engine:?}",
            );
        }
    }

    #[test]
    fn prepared_dynamic_batch_matches_scalar_queries() {
        let query = CollisionObject::circle((0.0, 0.0), 0.25).unwrap();
        let dynamic = DynamicObstacle::new(
            query,
            vec![DPose2::translation(4.0, 0.0), DPose2::translation(0.5, 0.0)],
            TimeStep(5),
        )
        .unwrap();

        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();
            let prepared = checker.prepare_dynamic(&dynamic).unwrap();
            let prepared_queries = vec![prepared.clone(), prepared.clone()];
            let expected = prepared_queries
                .iter()
                .map(|query| {
                    checker.collides_dynamic_prepared_range(query, TimeStep(5)..=TimeStep(6))
                })
                .collect::<Vec<_>>();

            assert_eq!(
                checker.collides_dynamic_prepared_batch(
                    &prepared_queries,
                    TimeStep(5)..=TimeStep(6),
                    true,
                ),
                expected,
                "{engine:?}",
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
    fn explicit_static_modes_match() {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(2)
            .build()
            .unwrap();
        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();

            for count in [1, 512] {
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
                    pool.install(|| checker.collides_static_batch(&queries, .., true)),
                    expected,
                    "{engine:?}, {count}"
                );
            }
        }
    }

    #[test]
    fn explicit_dynamic_modes_match() {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(2)
            .build()
            .unwrap();
        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();

            for count in [1, 256] {
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
                    pool.install(|| checker.collides_dynamic_batch(
                        &queries,
                        TimeStep(5)..=TimeStep(6),
                        true
                    )),
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

    #[cfg(feature = "rayon")]
    #[test]
    fn heterogeneous_batches_match_scalar_results() {
        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .with_dynamic_obstacle(
                    DynamicObstacle::new(
                        CollisionObject::circle((0.0, 0.0), 1.0).unwrap(),
                        vec![DPose2::translation(5.0, 0.0), DPose2::IDENTITY],
                        TimeStep(0),
                    )
                    .unwrap(),
                )
                .build_with_engine(engine)
                .unwrap();

            let raw_static = CollisionObject::circle((0.0, 0.0), 0.25).unwrap();
            let prepared_static = checker.prepare_static(&raw_static).unwrap();
            let raw_dynamic = DynamicObstacle::new(
                CollisionObject::circle((0.0, 0.0), 0.25).unwrap(),
                vec![DPose2::translation(5.25, 0.0)],
                TimeStep(0),
            )
            .unwrap();
            let prepared_dynamic = checker.prepare_dynamic(&raw_dynamic).unwrap();

            let static_sources = vec![
                (StaticBatchQuery::Raw(&raw_static), DPose2::IDENTITY),
                (
                    StaticBatchQuery::Prepared(&prepared_static),
                    DPose2::IDENTITY,
                ),
                (
                    StaticBatchQuery::Raw(&raw_static),
                    DPose2::translation(4.0, 0.0),
                ),
                (
                    StaticBatchQuery::Prepared(&prepared_static),
                    DPose2::translation(4.0, 0.0),
                ),
            ];
            let static_expected = vec![
                checker.collides_static_pos(&raw_static, DPose2::IDENTITY),
                checker.collides_static_prepared(&prepared_static),
                checker.collides_static_pos(&raw_static, DPose2::translation(4.0, 0.0)),
                checker.collides_static_prepared_range(
                    &prepared_static,
                    DPose2::translation(4.0, 0.0),
                    ..,
                ),
            ];
            assert_eq!(
                checker.collides_static_heterogeneous_batch(static_sources, .., true),
                static_expected,
                "{engine:?}"
            );

            let dynamic_sources = vec![
                DynamicBatchQuery::Raw(&raw_dynamic),
                DynamicBatchQuery::Prepared(&prepared_dynamic),
            ];
            let expected = vec![
                checker.collides_dynamic_range(&raw_dynamic, TimeStep(0)..=TimeStep(0)),
                checker
                    .collides_dynamic_prepared_range(&prepared_dynamic, TimeStep(0)..=TimeStep(0)),
            ];
            assert_eq!(
                checker.collides_dynamic_heterogeneous_batch(
                    dynamic_sources,
                    TimeStep(0)..=TimeStep(0),
                    true,
                ),
                expected,
                "{engine:?}"
            );
        }
    }

    #[cfg(feature = "rayon")]
    #[test]
    fn heterogeneous_batches_reject_mismatched_backends_in_order() {
        for engine in engines() {
            let Some(other) = for_other_engine(engine) else {
                continue;
            };
            let owner = CollisionCheckerBuilder::new()
                .build_with_engine(engine)
                .unwrap();
            let prepared = owner
                .prepare_static(&CollisionObject::circle((0.0, 0.0), 0.25).unwrap())
                .unwrap();
            let raw = CollisionObject::circle((10.0, 0.0), 0.25).unwrap();
            let sources = vec![
                (StaticBatchQuery::Raw(&raw), DPose2::IDENTITY),
                (StaticBatchQuery::Prepared(&prepared), DPose2::IDENTITY),
            ];

            assert_eq!(
                other.collides_static_heterogeneous_batch(sources, .., true),
                vec![Err(CrccError::Unsupported), Err(CrccError::Unsupported)],
                "{engine:?}"
            );
        }
    }

    /// Returns a checker on a different backend, or `None` when only one backend is compiled.
    #[cfg(feature = "rayon")]
    fn for_other_engine(engine: CollisionEngine) -> Option<SelectedCollisionChecker> {
        let alternatives = [
            #[cfg(feature = "parry")]
            CollisionEngine::Parry,
            #[cfg(feature = "rhusics")]
            CollisionEngine::Rhusics,
            #[cfg(feature = "collide")]
            CollisionEngine::Collide,
        ];
        alternatives
            .into_iter()
            .find(|candidate| *candidate != engine)
            .map(|candidate| {
                CollisionCheckerBuilder::new()
                    .build_with_engine(candidate)
                    .unwrap()
            })
    }
}
