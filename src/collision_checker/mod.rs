use crate::collision_checker::ccd_collider::{CCDCollider, CCDColliderAt};
use crate::collision_checker::engine::EngineCollisionObject;
use crate::dynamic_obstacle::GenericDynamicObstacle;
use crate::time::{TimeStep, TimeStepSet};
use glamx::DPose2;
use std::ops::RangeBounds;

use crate::error::CrccError;

mod builder;
mod ccd_collider;
pub mod engine;
#[cfg(feature = "rayon")]
pub mod parallel;
mod selected;

pub use builder::CollisionCheckerBuilder;
#[cfg(feature = "python_bindings")]
pub(crate) use builder::road_boundary;
pub use selected::SelectedCollisionChecker;

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
    pub fn collides(&self) -> bool {
        match self {
            CollisionStatus::NoCollision => false,
            CollisionStatus::CollidesStatic | CollisionStatus::CollidesDynamic(_) => true,
        }
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
    pub fn collides_static(&self, static_obstacle: &E) -> CollisionResult {
        self.collides_static_range(static_obstacle, DPose2::IDENTITY, ..)
    }

    pub fn collides_dynamic(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E>,
    ) -> CollisionResult {
        self.collides_dynamic_range(dynamic_obstacle, ..)
    }

    pub fn collides_static_at(&self, static_obstacle: &E, time_step: TimeStep) -> CollisionResult {
        self.collides_static_range(static_obstacle, DPose2::IDENTITY, time_step..=time_step)
    }

    pub fn collides_dynamic_at(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E>,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_dynamic_range(dynamic_obstacle, time_step..=time_step)
    }

    pub fn collides_static_pos(&self, static_obstacle: &E, position: DPose2) -> CollisionResult {
        self.collides_static_range(static_obstacle, position, ..)
    }

    pub fn collides_static_pos_at(
        &self,
        static_obstacle: &E,
        position: DPose2,
        time_step: TimeStep,
    ) -> CollisionResult {
        self.collides_static_range(static_obstacle, position, time_step..=time_step)
    }

    pub fn collides_static_range(
        &self,
        static_obstacle: &E,
        position: DPose2,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        if self.check_collision_static_static(static_obstacle, position)? {
            return Ok(CollisionStatus::CollidesStatic);
        }

        let mut active_times = TimeStepSet::from(time_range);
        active_times.intersect(&self.active_times);
        for time_step in active_times.iter() {
            let ccd_collider = active_times
                .contains(time_step.succ())
                .then(|| Self::stationary_ccd_collider(static_obstacle, position));
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

    pub fn collides_dynamic_range(
        &self,
        dynamic_obstacle: &GenericDynamicObstacle<E>,
        time_range: impl RangeBounds<TimeStep>,
    ) -> CollisionResult {
        let mut active_times = TimeStepSet::from(time_range);
        active_times.intersect(&dynamic_obstacle.active_times());
        for time_step in active_times.iter() {
            if self.dynamic_query_collides_at(
                dynamic_obstacle,
                time_step,
                active_times.contains(time_step.succ()),
            )? {
                return Ok(CollisionStatus::CollidesDynamic(time_step));
            }
        }
        Ok(CollisionStatus::NoCollision)
    }

    fn stationary_ccd_collider(static_obstacle: &E, position: DPose2) -> CCDCollider<'_, E> {
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
        if next_step_active && let Some(ccd_collider) = dynamic_obstacle.ccd_collider_at(time_step)
        {
            return Ok(self.check_collision_static_ccd(&ccd_collider)?
                || (self.active_times.contains(time_step)
                    && self.check_collision_dynamic_ccd(&ccd_collider, time_step)?));
        }

        let Some((shape, position)) = dynamic_obstacle.obstacle_at(time_step) else {
            return Ok(false);
        };
        Ok(self.check_collision_static_static(shape, position)?
            || (self.active_times.contains(time_step)
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

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "parry")]
    use crate::collision_checker::engine::parry::ParryCollisionObject;
    use crate::collision_object::CollisionObject;
    use crate::collision_object::simple::SimpleCollisionObject;
    use crate::dynamic_obstacle::DynamicObstacle;
    use geo::Rect;
    use std::f64::consts::FRAC_PI_2;

    #[cfg(feature = "parry")]
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
        );
        let checker = CollisionCheckerBuilder::new()
            .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
            .with_dynamic_obstacle(dynamic_obstacle)
            .build::<ParryCollisionObject>();

        let query =
            CollisionObject::from(SimpleCollisionObject::circle((8.0, 8.0), 1.0).unwrap()).into();

        assert!(
            !checker
                .collides_static_range(&query, DPose2::IDENTITY, TimeStep(0)..=TimeStep(0))
                .unwrap()
                .collides()
        );
        assert!(
            checker
                .collides_static_range(&query, DPose2::IDENTITY, TimeStep(1)..=TimeStep(1))
                .unwrap()
                .collides()
        );
        assert!(
            !checker
                .collides_static_range(&query, DPose2::IDENTITY, TimeStep(2)..=TimeStep(2))
                .unwrap()
                .collides()
        );
        assert!(
            !checker
                .collides_static_range(&query, DPose2::IDENTITY, TimeStep(3)..=TimeStep(3))
                .unwrap()
                .collides()
        );
        assert!(
            !checker
                .collides_static_range(&query, DPose2::IDENTITY, TimeStep(4)..=TimeStep(4))
                .unwrap()
                .collides()
        );
        assert_eq!(
            checker
                .collides_static_range(&query, DPose2::IDENTITY, ..)
                .unwrap(),
            CollisionStatus::CollidesDynamic(TimeStep(0))
        );
        assert_eq!(
            checker
                .collides_static_range(&query, DPose2::IDENTITY, TimeStep(2)..=TimeStep(4))
                .unwrap(),
            CollisionStatus::CollidesDynamic(TimeStep(2))
        );
    }

    #[cfg(feature = "parry")]
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
        );
        let checker = CollisionCheckerBuilder::new()
            .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
            .with_dynamic_obstacle(dynamic_obstacle)
            .build::<ParryCollisionObject>();
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
        .convert_repr();

        assert_eq!(
            checker.collides_dynamic(&moving_query).unwrap(),
            CollisionStatus::NoCollision
        );
    }

    #[cfg(feature = "parry")]
    #[test]
    fn dynamic_query_uses_static_ccd_narrow_phase() {
        let checker = CollisionCheckerBuilder::new()
            .with_static_obstacle(SimpleCollisionObject::circle((-1.9, 0.3), 0.1).unwrap())
            .build::<ParryCollisionObject>();
        let moving_query = DynamicObstacle::new(
            SimpleCollisionObject::rectangle(Rect::new((-2.0, -0.1), (2.0, 0.1)), 0.0)
                .unwrap()
                .into(),
            vec![DPose2::IDENTITY, DPose2::new((0.0, 0.0).into(), FRAC_PI_2)],
            TimeStep(0),
        )
        .convert_repr();

        assert_eq!(
            checker.collides_dynamic(&moving_query).unwrap(),
            CollisionStatus::NoCollision
        );
    }

    #[cfg(feature = "parry")]
    #[test]
    fn dynamic_compound_query_matches_expanded_children() {
        let static_compound = CollisionObject::merge_all([
            CollisionObject::from(SimpleCollisionObject::circle((0.0, 0.0), 0.5).unwrap()),
            CollisionObject::from(
                SimpleCollisionObject::rectangle(Rect::new((4.0, -0.5), (5.0, 0.5)), 0.0).unwrap(),
            ),
        ]);
        let checker = CollisionCheckerBuilder::new()
            .with_static_obstacle(static_compound)
            .build::<ParryCollisionObject>();
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
        .convert_repr();
        let expanded = query_parts
            .into_iter()
            .map(|part| {
                let obstacle =
                    DynamicObstacle::new(part, positions.clone(), TimeStep(0)).convert_repr();
                checker.collides_dynamic(&obstacle).unwrap()
            })
            .find(|status| status.collides())
            .unwrap_or(CollisionStatus::NoCollision);

        assert_eq!(checker.collides_dynamic(&compound).unwrap(), expanded);
    }

    #[cfg(feature = "parry")]
    #[test]
    fn rotated_rectangles_collide_at_expected_extents() {
        let rect1 =
            SimpleCollisionObject::rectangle(Rect::new((0.0, 0.0), (2.0, 1.0)), 0.0).unwrap();
        let rect2 =
            SimpleCollisionObject::rectangle(Rect::new((0.0, 1.1), (2.0, 2.1)), FRAC_PI_2).unwrap();
        let checker = CollisionCheckerBuilder::new()
            .with_static_obstacle(rect1)
            .build::<ParryCollisionObject>();

        assert_ne!(
            checker
                .collides_static(&CollisionObject::from(rect2).into())
                .unwrap(),
            CollisionStatus::NoCollision
        );
    }

    #[cfg(feature = "parry")]
    #[test]
    fn full_space_query_always_collides() {
        let checker = CollisionCheckerBuilder::new()
            .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
            .build::<ParryCollisionObject>();

        assert_ne!(
            checker
                .collides_static(&CollisionObject::from(SimpleCollisionObject::full_space()).into())
                .unwrap(),
            CollisionStatus::NoCollision
        );
    }
}
