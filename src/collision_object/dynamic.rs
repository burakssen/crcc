use crate::collision_object::CollisionObject;
use crate::error::{CrccError, CrccResult};
use crate::time::{TimeStep, TimeStepSet};
use glamx::DPose2;

#[derive(Debug, Clone)]
pub struct GenericDynamicObstacle<E> {
    pub(crate) trajectory: DynamicObstacleTrajectory<E>,
    pub(crate) time_offset: TimeStep,
}

#[derive(Debug, Clone)]
pub(crate) enum DynamicObstacleTrajectory<E> {
    FixedShape {
        shape: E,
        positions: Vec<DPose2>,
        convex_hulls: Vec<E>,
    },
    VaryingShape {
        obstacles: Vec<E>,
        positions: Vec<DPose2>,
        convex_hulls: Vec<E>,
    },
}

#[derive(Debug, Clone)]
/// A discrete moving obstacle used by [`crate::CollisionChecker`].
pub struct DynamicObstacle(GenericDynamicObstacle<CollisionObject>);

impl DynamicObstacle {
    /// Creates a fixed-shape trajectory.
    ///
    /// `positions[0]` is active at `time_offset`; later poses advance one time
    /// step each. Motion between adjacent poses is checked conservatively.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidGeometry`] when a pose is non-finite or the
    /// trajectory extends beyond [`TimeStep::MAX`].
    pub fn new(
        shape: CollisionObject,
        positions: Vec<DPose2>,
        time_offset: TimeStep,
    ) -> CrccResult<Self> {
        validate_trajectory(&positions, time_offset)?;
        let convex_hulls = shape.swept_areas(&positions);
        Ok(Self(GenericDynamicObstacle {
            trajectory: DynamicObstacleTrajectory::FixedShape {
                shape,
                positions,
                convex_hulls,
            },
            time_offset,
        }))
    }

    /// Creates a trajectory whose shape may change at each step.
    ///
    /// `obstacles` and `positions` must have equal lengths.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::InvalidGeometry`] when the input lengths differ, a
    /// pose is non-finite, or the trajectory extends beyond [`TimeStep::MAX`].
    ///
    pub fn time_variant(
        obstacles: Vec<CollisionObject>,
        positions: Vec<DPose2>,
        time_offset: TimeStep,
    ) -> CrccResult<Self> {
        if obstacles.len() != positions.len() {
            return Err(CrccError::InvalidGeometry(
                "time-variant obstacle shape and pose counts must match",
            ));
        }
        validate_trajectory(&positions, time_offset)?;
        let convex_hulls = obstacles
            .windows(2)
            .zip(positions.windows(2))
            .filter_map(|(shapes, poses)| {
                let [shape_t, shape_t1] = shapes else {
                    return None;
                };
                let [pos_t, pos_t1] = poses else {
                    return None;
                };

                if shape_t.is_empty() || shape_t1.is_empty() {
                    // Missing occupancy means no motion across this interval.
                    return Some(CollisionObject::empty());
                }

                shape_t
                    .swept_area(*pos_t, *pos_t1)
                    .zip(shape_t1.swept_area(*pos_t, *pos_t1))
                    .map(|(swept_t, swept_t1)| swept_t.merge(swept_t1))
            })
            .collect();
        Ok(Self(GenericDynamicObstacle {
            trajectory: DynamicObstacleTrajectory::VaryingShape {
                obstacles,
                positions,
                convex_hulls,
            },
            time_offset,
        }))
    }

    #[must_use]
    pub fn convert_repr<E: From<CollisionObject>>(self) -> GenericDynamicObstacle<E> {
        GenericDynamicObstacle {
            trajectory: self.0.trajectory.convert_repr(),
            time_offset: self.0.time_offset,
        }
    }

    #[cfg(test)]
    pub(crate) fn obstacle_at(&self, time_step: TimeStep) -> Option<(&CollisionObject, DPose2)> {
        self.0.obstacle_at(time_step)
    }

    pub(crate) fn active_times(&self) -> TimeStepSet {
        self.0.active_times()
    }
}

fn validate_trajectory(positions: &[DPose2], time_offset: TimeStep) -> CrccResult<()> {
    if positions
        .iter()
        .any(|pose| !pose.translation.is_finite() || !pose.rotation.angle().is_finite())
    {
        return Err(CrccError::InvalidGeometry(
            "trajectory poses must contain only finite values",
        ));
    }

    if let Some(last_index) = positions.len().checked_sub(1)
        && time_offset.checked_add_steps(last_index).is_none()
    {
        return Err(CrccError::InvalidGeometry(
            "trajectory exceeds the representable time-step range",
        ));
    }

    Ok(())
}

impl DynamicObstacleTrajectory<CollisionObject> {
    fn convert_repr<E: From<CollisionObject>>(self) -> DynamicObstacleTrajectory<E> {
        match self {
            Self::FixedShape {
                shape,
                positions,
                convex_hulls,
            } => DynamicObstacleTrajectory::FixedShape {
                shape: shape.into(),
                positions,
                convex_hulls: convex_hulls.into_iter().map(E::from).collect(),
            },
            Self::VaryingShape {
                obstacles,
                positions,
                convex_hulls,
            } => DynamicObstacleTrajectory::VaryingShape {
                obstacles: obstacles.into_iter().map(E::from).collect(),
                positions,
                convex_hulls: convex_hulls.into_iter().map(E::from).collect(),
            },
        }
    }
}

impl<E> DynamicObstacleTrajectory<E> {
    fn obstacle_at(&self, index: usize) -> Option<(&E, DPose2)> {
        match self {
            Self::FixedShape {
                shape, positions, ..
            } => positions.get(index).map(|position| (shape, *position)),
            Self::VaryingShape {
                obstacles,
                positions,
                ..
            } => obstacles
                .get(index)
                .zip(positions.get(index))
                .map(|(obstacle, position)| (obstacle, *position)),
        }
    }

    fn positions(&self) -> &[DPose2] {
        match self {
            Self::FixedShape { positions, .. } | Self::VaryingShape { positions, .. } => positions,
        }
    }

    fn len(&self) -> usize {
        self.positions().len()
    }
}

impl<E> GenericDynamicObstacle<E> {
    pub(crate) fn obstacle_at(&self, time_step: TimeStep) -> Option<(&E, DPose2)> {
        let elapsed_steps = time_step.0.checked_sub(self.time_offset.0)?;
        let index = usize::try_from(elapsed_steps).ok()?;
        self.trajectory.obstacle_at(index)
    }

    pub(crate) fn active_times(&self) -> TimeStepSet {
        let Some(last_index) = self.len().checked_sub(1) else {
            return TimeStepSet::new();
        };
        let Some(end) = self.time_offset.checked_add_steps(last_index) else {
            return TimeStepSet::new();
        };
        TimeStep::iter_range(self.time_offset..=end).collect()
    }

    fn len(&self) -> usize {
        self.trajectory.len()
    }
}
#[cfg(test)]
#[allow(clippy::panic_in_result_fn)]
mod tests {
    use super::*;
    use geo::Rect;
    use glamx::approx::assert_relative_eq;

    fn dynamic_obstacle() -> CrccResult<DynamicObstacle> {
        let shape = CollisionObject::rectangle(Rect::new((-2.0, -0.5), (2.0, 0.5)), 0.0)?;

        let positions = vec![
            DPose2::new((0.0, 0.0).into(), 0.0),
            DPose2::new((1.0, 1.0).into(), std::f64::consts::FRAC_PI_4),
            DPose2::new((2.0, 2.0).into(), std::f64::consts::FRAC_PI_2),
        ];

        DynamicObstacle::new(shape, positions, TimeStep(5))
    }

    #[test]
    fn obstacle_at_returns_pose_for_active_time() -> CrccResult<()> {
        let dynamic_obstacle = dynamic_obstacle()?;

        assert_eq!(
            dynamic_obstacle
                .obstacle_at(TimeStep(5))
                .map(|(_, position)| position),
            Some(DPose2::new((0.0, 0.0).into(), 0.0)),
        );

        assert_eq!(
            dynamic_obstacle
                .obstacle_at(TimeStep(6))
                .map(|(_, position)| position),
            Some(DPose2::new((1.0, 1.0).into(), std::f64::consts::FRAC_PI_4,)),
        );

        assert_eq!(
            dynamic_obstacle
                .obstacle_at(TimeStep(7))
                .map(|(_, position)| position),
            Some(DPose2::new((2.0, 2.0).into(), std::f64::consts::FRAC_PI_2,)),
        );

        assert!(dynamic_obstacle.obstacle_at(TimeStep(8)).is_none(),);
        Ok(())
    }

    #[test]
    fn active_times_cover_trajectory_range() -> CrccResult<()> {
        let dynamic_obstacle = dynamic_obstacle()?;

        let active_times = dynamic_obstacle.active_times();

        assert_eq!(
            active_times,
            TimeStep::iter_range(TimeStep(5)..=TimeStep(7),).collect(),
        );
        Ok(())
    }

    #[test]
    fn trajectories_report_exact_unrepresentable_end_time_error() -> CrccResult<()> {
        let shape = CollisionObject::circle((0.0, 0.0), 1.0)?;

        assert!(matches!(
            DynamicObstacle::new(
                shape.clone(),
                vec![DPose2::IDENTITY, DPose2::IDENTITY],
                TimeStep::MAX,
            ),
            Err(CrccError::InvalidGeometry(
                "trajectory exceeds the representable time-step range"
            ))
        ));
        let obstacle = DynamicObstacle::new(shape, vec![DPose2::IDENTITY], TimeStep::MAX)?;
        assert_eq!(
            obstacle.active_times(),
            std::iter::once(TimeStep::MAX).collect()
        );
        Ok(())
    }

    #[test]
    fn trajectories_report_exact_non_finite_pose_error() -> CrccResult<()> {
        let shape = CollisionObject::circle((0.0, 0.0), 1.0)?;

        assert!(matches!(
            DynamicObstacle::new(
                shape,
                vec![DPose2::translation(f64::NAN, 0.0)],
                TimeStep::ZERO,
            ),
            Err(CrccError::InvalidGeometry(
                "trajectory poses must contain only finite values"
            ))
        ));
        Ok(())
    }

    #[test]
    fn time_variant_reports_exact_shape_pose_count_error() {
        assert!(matches!(
            DynamicObstacle::time_variant(
                vec![CollisionObject::empty()],
                Vec::new(),
                TimeStep::ZERO,
            ),
            Err(CrccError::InvalidGeometry(
                "time-variant obstacle shape and pose counts must match"
            ))
        ));
    }

    #[test]
    fn time_variant_selects_shapes_at_extreme_times() -> CrccResult<()> {
        let small = CollisionObject::circle((0.0, 0.0), 1.0)?;
        let large = CollisionObject::circle((0.0, 0.0), 2.0)?;
        let obstacle = DynamicObstacle::time_variant(
            vec![small, large],
            vec![DPose2::IDENTITY, DPose2::translation(1.0, 0.0)],
            TimeStep::MAX.pred(),
        )?;

        let Some((shape, pose)) = obstacle.obstacle_at(TimeStep::MAX) else {
            return Err(CrccError::InvalidGeometry(
                "test expected an obstacle at TimeStep::MAX",
            ));
        };
        let [crate::collision_object::simple::SimpleCollisionObject::Circle(circle)] =
            shape.collision_objects()
        else {
            return Err(CrccError::InvalidGeometry(
                "test expected the second circle shape",
            ));
        };

        assert_relative_eq!(circle.radius(), 2.0);
        assert_eq!(pose, DPose2::translation(1.0, 0.0));
        assert_eq!(
            obstacle.active_times(),
            [TimeStep::MAX.pred(), TimeStep::MAX].into_iter().collect(),
        );
        Ok(())
    }

    #[test]
    fn time_variant_empty_endpoint_has_empty_interval_sweep() -> CrccResult<()> {
        let obstacle = DynamicObstacle::time_variant(
            vec![
                CollisionObject::empty(),
                CollisionObject::circle((0.0, 0.0), 1.0)?,
            ],
            vec![DPose2::IDENTITY, DPose2::translation(1.0, 0.0)],
            TimeStep::MIN,
        )?;
        let DynamicObstacleTrajectory::VaryingShape { convex_hulls, .. } = &obstacle.0.trajectory
        else {
            return Err(CrccError::InvalidGeometry(
                "test expected a time-variant trajectory",
            ));
        };
        let [convex_hull] = convex_hulls.as_slice() else {
            return Err(CrccError::InvalidGeometry(
                "test expected exactly one swept hull",
            ));
        };

        assert!(convex_hull.is_empty());
        Ok(())
    }

    #[test]
    fn len_fixture_matches() -> CrccResult<()> {
        let dynamic_obstacle = dynamic_obstacle()?;

        assert_eq!(dynamic_obstacle.0.len(), 3);
        Ok(())
    }

    #[test]
    fn convex_hull_uses_motion_radius_and_translation_extrema() -> CrccResult<()> {
        let dynamic_obstacle = dynamic_obstacle()?;
        let DynamicObstacleTrajectory::FixedShape {
            positions,
            convex_hulls,
            ..
        } = &dynamic_obstacle.0.trajectory
        else {
            return Err(CrccError::InvalidGeometry(
                "test expected a fixed-shape trajectory",
            ));
        };
        let Some(convex_hull) = convex_hulls.first() else {
            return Err(CrccError::InvalidGeometry("test expected a swept hull"));
        };
        let [crate::collision_object::simple::SimpleCollisionObject::Rectangle(bound)] =
            convex_hull.collision_objects()
        else {
            return Err(CrccError::InvalidGeometry(
                "test expected a rectangular swept bound",
            ));
        };
        let [start, end, ..] = positions.as_slice() else {
            return Err(CrccError::InvalidGeometry(
                "test expected at least two trajectory poses",
            ));
        };
        let radius = 2.0_f64.hypot(0.5);

        assert_relative_eq!(bound.rect().min().x, start.translation.x - radius);
        assert_relative_eq!(bound.rect().min().y, start.translation.y - radius);
        assert_relative_eq!(bound.rect().max().x, end.translation.x + radius);
        assert_relative_eq!(bound.rect().max().y, end.translation.y + radius);
        Ok(())
    }

    #[test]
    fn convex_hulls_exist_between_positions_only() -> CrccResult<()> {
        let dynamic_obstacle = dynamic_obstacle()?;
        let DynamicObstacleTrajectory::FixedShape { convex_hulls, .. } =
            &dynamic_obstacle.0.trajectory
        else {
            return Err(CrccError::InvalidGeometry(
                "test expected a fixed-shape trajectory",
            ));
        };

        assert_eq!(convex_hulls.len(), 2);
        Ok(())
    }

    #[test]
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide",))]
    fn convert_repr_preserves_trajectory_metadata() -> CrccResult<()> {
        fn check<E: crate::collision_checker::engine::EngineCollisionObject>(
            dynamic_obstacle: &DynamicObstacle,
        ) -> CrccResult<()> {
            let converted = dynamic_obstacle.clone().convert_repr::<E>();

            assert_eq!(
                converted.trajectory.positions(),
                dynamic_obstacle.0.trajectory.positions(),
            );

            assert_eq!(converted.time_offset, dynamic_obstacle.0.time_offset,);

            let (converted_hull_count, original_hull_count) =
                match (&converted.trajectory, &dynamic_obstacle.0.trajectory) {
                    (
                        DynamicObstacleTrajectory::FixedShape {
                            convex_hulls: converted_hulls,
                            ..
                        },
                        DynamicObstacleTrajectory::FixedShape {
                            convex_hulls: original_hulls,
                            ..
                        },
                    ) => (converted_hulls.len(), original_hulls.len()),
                    _ => {
                        return Err(CrccError::InvalidGeometry(
                            "test expected matching fixed-shape trajectories",
                        ));
                    }
                };

            assert_eq!(converted_hull_count, original_hull_count,);
            Ok(())
        }

        let dynamic_obstacle = dynamic_obstacle()?;

        #[cfg(feature = "parry")]
        check::<crate::collision_checker::engine::parry::ParryCollisionObject>(&dynamic_obstacle)?;

        #[cfg(feature = "rhusics")]
        check::<crate::collision_checker::engine::rhusics::RhusicsCoreCollisionObject>(
            &dynamic_obstacle,
        )?;

        #[cfg(feature = "collide")]
        check::<crate::collision_checker::engine::collide::CollideCollisionObject>(
            &dynamic_obstacle,
        )?;
        Ok(())
    }
}
