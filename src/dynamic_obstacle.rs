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
    /// `obstacles` and `positions` must have equal lengths. This constructor
    /// panics when they differ.
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
        let mut convex_hulls = Vec::new();
        for i in 0..obstacles.len().saturating_sub(1) {
            let shape_t = &obstacles[i];
            let shape_t1 = &obstacles[i + 1];
            let pos_t = positions[i];
            let pos_t1 = positions[i + 1];
            let swept_t = shape_t.swept_area(pos_t, pos_t1);
            let swept_t1 = shape_t1.swept_area(pos_t, pos_t1);
            convex_hulls.push(swept_t.merge(swept_t1));
        }
        Self(GenericDynamicObstacle {
            trajectory: DynamicObstacleTrajectory::VaryingShape {
                obstacles,
                positions,
                convex_hulls,
            },
            time_offset,
        })
    }

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
            DynamicObstacleTrajectory::FixedShape {
                shape,
                positions,
                convex_hulls,
            } => DynamicObstacleTrajectory::FixedShape {
                shape: shape.into(),
                positions,
                convex_hulls: convex_hulls.into_iter().map(E::from).collect(),
            },
            DynamicObstacleTrajectory::VaryingShape {
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
            DynamicObstacleTrajectory::FixedShape {
                shape, positions, ..
            } => positions.get(index).map(|position| (shape, *position)),
            DynamicObstacleTrajectory::VaryingShape {
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
            DynamicObstacleTrajectory::FixedShape { positions, .. } => positions,
            DynamicObstacleTrajectory::VaryingShape { positions, .. } => positions,
        }
    }

    fn len(&self) -> usize {
        self.positions().len()
    }
}

impl<E> GenericDynamicObstacle<E> {
    pub(crate) fn obstacle_at(&self, time_step: TimeStep) -> Option<(&E, DPose2)> {
        let index = time_step.0.checked_sub(self.time_offset.0)? as usize;
        self.trajectory.obstacle_at(index)
    }

    pub(crate) fn active_times(&self) -> TimeStepSet {
        match self.len().checked_sub(1) {
            Some(last_index) => {
                TimeStepSet::from(self.time_offset..=self.time_offset.add_steps(last_index))
            }
            None => TimeStepSet::default(),
        }
    }

    fn len(&self) -> usize {
        self.trajectory.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::TimeStepSet;
    use geo::Rect;
    use rstest::{fixture, rstest};

    #[fixture]
    fn dynamic_obstacle() -> DynamicObstacle {
        let shape = CollisionObject::rectangle(Rect::new((-2.0, -0.5), (2.0, 0.5)), 0.0).unwrap();
        let positions = vec![
            DPose2::new((0.0, 0.0).into(), 0.0),
            DPose2::new((1.0, 1.0).into(), std::f64::consts::FRAC_PI_4),
            DPose2::new((2.0, 2.0).into(), std::f64::consts::FRAC_PI_2),
        ];
        DynamicObstacle::new(shape, positions, TimeStep(5))
    }

    #[rstest]
    fn obstacle_at_returns_pose_for_active_time(dynamic_obstacle: DynamicObstacle) {
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
            Some(DPose2::new((1.0, 1.0).into(), std::f64::consts::FRAC_PI_4)),
        );
        assert_eq!(
            dynamic_obstacle
                .obstacle_at(TimeStep(7))
                .map(|(_, position)| position),
            Some(DPose2::new((2.0, 2.0).into(), std::f64::consts::FRAC_PI_2)),
        );
        assert!(dynamic_obstacle.obstacle_at(TimeStep(8)).is_none());
    }

    #[rstest]
    fn active_times_cover_trajectory_range(dynamic_obstacle: DynamicObstacle) {
        let active_times = dynamic_obstacle.active_times();
        assert_eq!(active_times, TimeStepSet::from(TimeStep(5)..=TimeStep(7)));
    }

    #[rstest]
    #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
    fn convex_hull_covers_interpolated_motion(dynamic_obstacle: DynamicObstacle) {
        let DynamicObstacleTrajectory::FixedShape {
            shape,
            positions,
            convex_hulls,
        } = &dynamic_obstacle.0.trajectory
        else {
            panic!("fixture should be a fixed-shape dynamic obstacle");
        };
        let convex_hull = &convex_hulls[0];
        let start_pos = positions[0];
        let end_pos = positions[1];
        // Interpolate 5 points between start_pos and end_pos
        for i in 0..=5 {
            let t = i as f64 / 5.0;
            let interpolated_position = DPose2::from_parts(
                start_pos.translation.lerp(end_pos.translation, t),
                start_pos.rotation.slerp(&end_pos.rotation, t),
            );
            assert!(
                crate::collision_checker::engine::collides(
                    convex_hull,
                    DPose2::IDENTITY,
                    shape,
                    interpolated_position,
                    crate::collision_checker::engine::CollisionEngine::default()
                )
                .unwrap()
            );
        }
    }

    #[rstest]
    fn convex_hulls_exist_between_positions_only(dynamic_obstacle: DynamicObstacle) {
        let DynamicObstacleTrajectory::FixedShape { convex_hulls, .. } =
            &dynamic_obstacle.0.trajectory
        else {
            panic!("fixture should be a fixed-shape dynamic obstacle");
        };
        assert_eq!(convex_hulls.len(), 2);
    }

    #[cfg(feature = "parry")]
    #[rstest]
    fn convert_repr_preserves_trajectory_metadata(dynamic_obstacle: DynamicObstacle) {
        use crate::collision_checker::engine::parry::ParryCollisionObject;

        let converted = dynamic_obstacle
            .clone()
            .convert_repr::<ParryCollisionObject>();
        assert_eq!(
            converted.trajectory.positions(),
            dynamic_obstacle.0.trajectory.positions()
        );
        assert_eq!(converted.time_offset, dynamic_obstacle.0.time_offset);
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
            ) => assert_eq!(converted_hulls.len(), original_hulls.len()),
            _ => panic!("fixture should be fixed-shape"),
        }
    }
}
