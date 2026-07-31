use crate::collision_object::CollisionObject;
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
    #[must_use]
    pub fn new(shape: CollisionObject, positions: Vec<DPose2>, time_offset: TimeStep) -> Self {
        let convex_hulls = shape.swept_areas(&positions);
        Self(GenericDynamicObstacle {
            trajectory: DynamicObstacleTrajectory::FixedShape {
                shape,
                positions,
                convex_hulls,
            },
            time_offset,
        })
    }

    /// Creates a trajectory whose shape may change at each step.
    ///
    /// `obstacles` and `positions` must have equal lengths.
    ///
    /// # Panics
    ///
    /// Panics if `obstacles` and `positions` have different lengths.
    #[must_use]
    pub fn time_variant(
        obstacles: Vec<CollisionObject>,
        positions: Vec<DPose2>,
        time_offset: TimeStep,
    ) -> Self {
        assert_eq!(
            obstacles.len(),
            positions.len(),
            "time-variant obstacle shape and pose counts must match"
        );
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

                shape_t
                    .swept_area(*pos_t, *pos_t1)
                    .zip(shape_t1.swept_area(*pos_t, *pos_t1))
                    .map(|(swept_t, swept_t1)| swept_t.merge(swept_t1))
            })
            .collect();
        Self(GenericDynamicObstacle {
            trajectory: DynamicObstacleTrajectory::VaryingShape {
                obstacles,
                positions,
                convex_hulls,
            },
            time_offset,
        })
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
        self.len()
            .checked_sub(1)
            .map_or_else(TimeStepSet::new, |last_index| {
                TimeStep::iter_range(self.time_offset..=self.time_offset.add_steps(last_index))
                    .collect()
            })
    }

    fn len(&self) -> usize {
        self.trajectory.len()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use geo::Rect;
    use std::ops::Div;

    fn dynamic_obstacle() -> Option<DynamicObstacle> {
        let shape = CollisionObject::rectangle(Rect::new((-2.0, -0.5), (2.0, 0.5)), 0.0).ok()?;

        let positions = vec![
            DPose2::new((0.0, 0.0).into(), 0.0),
            DPose2::new((1.0, 1.0).into(), std::f64::consts::FRAC_PI_4),
            DPose2::new((2.0, 2.0).into(), std::f64::consts::FRAC_PI_2),
        ];

        Some(DynamicObstacle::new(shape, positions, TimeStep(5)))
    }

    #[test]
    fn obstacle_at_returns_pose_for_active_time() {
        let dynamic_obstacle = dynamic_obstacle();

        assert!(
            dynamic_obstacle.is_some(),
            "failed to create dynamic-obstacle fixture",
        );

        let Some(dynamic_obstacle) = dynamic_obstacle else {
            return;
        };

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
    }

    #[test]
    fn active_times_cover_trajectory_range() {
        let dynamic_obstacle = dynamic_obstacle();

        assert!(
            dynamic_obstacle.is_some(),
            "failed to create dynamic-obstacle fixture",
        );

        let Some(dynamic_obstacle) = dynamic_obstacle else {
            return;
        };

        let active_times = dynamic_obstacle.active_times();

        assert_eq!(
            active_times,
            TimeStep::iter_range(TimeStep(5)..=TimeStep(7),).collect(),
        );
    }

    #[test]
    fn len_fixture_matches() {
        let dynamic_obstacle = dynamic_obstacle();

        assert!(
            dynamic_obstacle.is_some(),
            "failed to create dynamic-obstacle fixture",
        );

        let Some(dynamic_obstacle) = dynamic_obstacle else {
            return;
        };

        assert_eq!(dynamic_obstacle.0.len(), 3);
    }

    #[test]
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide",))]
    fn convex_hull_covers_interpolated_motion() {
        let dynamic_obstacle = dynamic_obstacle();

        assert!(
            dynamic_obstacle.is_some(),
            "failed to create dynamic-obstacle fixture",
        );

        let Some(dynamic_obstacle) = dynamic_obstacle else {
            return;
        };

        assert!(
            matches!(
                dynamic_obstacle.0.trajectory,
                DynamicObstacleTrajectory::FixedShape { .. }
            ),
            "fixture should be a fixed-shape dynamic obstacle",
        );

        let DynamicObstacleTrajectory::FixedShape {
            shape,
            positions,
            convex_hulls,
        } = &dynamic_obstacle.0.trajectory
        else {
            return;
        };

        let motion_data = convex_hulls
            .first()
            .zip(positions.first())
            .zip(positions.get(1));

        assert!(
            motion_data.is_some(),
            "fixture should contain one swept hull and two poses",
        );

        let Some(((convex_hull, start_position), end_position)) = motion_data else {
            return;
        };

        for sample_index in 0..=5 {
            let interpolation = f64::from(sample_index).div(5.0);

            let interpolated_position = DPose2::from_parts(
                start_position
                    .translation
                    .lerp(end_position.translation, interpolation),
                start_position
                    .rotation
                    .slerp(&end_position.rotation, interpolation),
            );

            let collision_result = crate::collision_checker::engine::collides(
                convex_hull,
                DPose2::IDENTITY,
                shape,
                interpolated_position,
                crate::collision_checker::engine::CollisionEngine::default(),
            );

            assert!(
                collision_result.is_ok(),
                "collision query failed: {collision_result:?}",
            );

            let Ok(collides) = collision_result else {
                return;
            };

            assert!(
                collides,
                "convex hull did not contain the shape at \
                 interpolation {interpolation}",
            );
        }
    }

    #[test]
    fn convex_hulls_exist_between_positions_only() {
        let dynamic_obstacle = dynamic_obstacle();

        assert!(
            dynamic_obstacle.is_some(),
            "failed to create dynamic-obstacle fixture",
        );

        let Some(dynamic_obstacle) = dynamic_obstacle else {
            return;
        };

        assert!(
            matches!(
                dynamic_obstacle.0.trajectory,
                DynamicObstacleTrajectory::FixedShape { .. }
            ),
            "fixture should be a fixed-shape dynamic obstacle",
        );

        let DynamicObstacleTrajectory::FixedShape { convex_hulls, .. } =
            &dynamic_obstacle.0.trajectory
        else {
            return;
        };

        assert_eq!(convex_hulls.len(), 2);
    }

    #[test]
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide",))]
    fn convert_repr_preserves_trajectory_metadata() {
        fn check<E: crate::collision_checker::engine::EngineCollisionObject>(
            dynamic_obstacle: &DynamicObstacle,
        ) {
            let converted = dynamic_obstacle.clone().convert_repr::<E>();

            assert_eq!(
                converted.trajectory.positions(),
                dynamic_obstacle.0.trajectory.positions(),
            );

            assert_eq!(converted.time_offset, dynamic_obstacle.0.time_offset,);

            let hull_lengths = match (&converted.trajectory, &dynamic_obstacle.0.trajectory) {
                (
                    DynamicObstacleTrajectory::FixedShape {
                        convex_hulls: converted_hulls,
                        ..
                    },
                    DynamicObstacleTrajectory::FixedShape {
                        convex_hulls: original_hulls,
                        ..
                    },
                ) => Some((converted_hulls.len(), original_hulls.len())),
                _ => None,
            };

            assert!(
                hull_lengths.is_some(),
                "fixture and converted obstacle should be fixed-shape",
            );

            let Some((converted_hull_count, original_hull_count)) = hull_lengths else {
                return;
            };

            assert_eq!(converted_hull_count, original_hull_count,);
        }

        let dynamic_obstacle = dynamic_obstacle();

        assert!(
            dynamic_obstacle.is_some(),
            "failed to create dynamic-obstacle fixture",
        );

        let Some(dynamic_obstacle) = dynamic_obstacle else {
            return;
        };

        #[cfg(feature = "parry")]
        check::<crate::collision_checker::engine::parry::ParryCollisionObject>(&dynamic_obstacle);

        #[cfg(feature = "rhusics")]
        check::<crate::collision_checker::engine::rhusics::RhusicsCoreCollisionObject>(
            &dynamic_obstacle,
        );

        #[cfg(feature = "collide")]
        check::<crate::collision_checker::engine::collide::CollideCollisionObject>(
            &dynamic_obstacle,
        );
    }
}
