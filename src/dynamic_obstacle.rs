use crate::collision_object::CollisionObject;
use crate::time::{TimeStep, TimeStepSet};
use glamx::DPose2;

#[derive(Debug, Clone)]
pub struct GenericDynamicObstacle<E> {
    pub(crate) shape: E,
    pub(crate) positions: Vec<DPose2>,
    pub(crate) time_offset: TimeStep,
    pub(crate) convex_hulls: Vec<E>,
}

pub type DynamicObstacle = GenericDynamicObstacle<CollisionObject>;

impl DynamicObstacle {
    pub fn new(shape: CollisionObject, positions: Vec<DPose2>, time_offset: TimeStep) -> Self {
        let convex_hulls = shape.swept_areas(&positions);
        Self {
            shape,
            positions,
            time_offset,
            convex_hulls,
        }
    }

    pub fn convert_repr<E: From<CollisionObject>>(self) -> GenericDynamicObstacle<E> {
        GenericDynamicObstacle {
            shape: self.shape.into(),
            positions: self.positions,
            time_offset: self.time_offset,
            convex_hulls: self.convex_hulls.into_iter().map(E::from).collect(),
        }
    }
}

impl<E> GenericDynamicObstacle<E> {
    pub fn shape(&self) -> &E {
        &self.shape
    }

    pub fn position_at(&self, time_step: TimeStep) -> Option<DPose2> {
        let index = time_step.0.checked_sub(self.time_offset.0)?;
        self.positions.get(index as usize).copied()
    }

    pub fn active_times(&self) -> TimeStepSet {
        TimeStepSet::from(self.time_offset..self.time_offset.add_steps(self.positions.len()))
    }

    pub fn convex_hull_after(&self, time_step: TimeStep) -> Option<&E> {
        let index = time_step.0.checked_sub(self.time_offset.0)?;
        self.convex_hulls.get(index as usize)
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
    fn test_position_at(dynamic_obstacle: DynamicObstacle) {
        assert_eq!(
            dynamic_obstacle.position_at(TimeStep(5)),
            Some(DPose2::new((0.0, 0.0).into(), 0.0))
        );
        assert_eq!(
            dynamic_obstacle.position_at(TimeStep(6)),
            Some(DPose2::new((1.0, 1.0).into(), std::f64::consts::FRAC_PI_4))
        );
        assert_eq!(
            dynamic_obstacle.position_at(TimeStep(7)),
            Some(DPose2::new((2.0, 2.0).into(), std::f64::consts::FRAC_PI_2))
        );
        assert_eq!(dynamic_obstacle.position_at(TimeStep(8)), None);
    }

    #[rstest]
    fn test_active_times(dynamic_obstacle: DynamicObstacle) {
        let active_times = dynamic_obstacle.active_times();
        assert_eq!(active_times, TimeStepSet::from(TimeStep(5)..=TimeStep(7)));
    }

    #[rstest]
    fn test_convex_hull_after(dynamic_obstacle: DynamicObstacle) {
        let convex_hull = dynamic_obstacle.convex_hull_after(TimeStep(5)).unwrap();
        let start_pos = dynamic_obstacle.position_at(TimeStep(5)).unwrap();
        let end_pos = dynamic_obstacle.position_at(TimeStep(6)).unwrap();
        // Interpolate 5 points between start_pos and end_pos
        for i in 0..=5 {
            let t = i as f64 / 5.0;
            let interp_pos = DPose2::from_parts(
                start_pos.translation.lerp(end_pos.translation, t),
                start_pos.rotation.slerp(&end_pos.rotation, t),
            );
            assert!(
                crate::collision_checker::engine::collides(
                    convex_hull,
                    DPose2::IDENTITY,
                    dynamic_obstacle.shape(),
                    interp_pos,
                    crate::collision_checker::engine::CollisionEngine::default()
                )
                .unwrap()
            );
        }
    }

    #[rstest]
    fn test_no_convex_hull_at_end(dynamic_obstacle: DynamicObstacle) {
        assert!(dynamic_obstacle.convex_hull_after(TimeStep(7)).is_none());
    }

    #[cfg(feature = "parry")]
    #[rstest]
    fn test_convert_repr(dynamic_obstacle: DynamicObstacle) {
        use crate::collision_checker::engine::parry::ParryCollisionObject;

        let converted = dynamic_obstacle
            .clone()
            .convert_repr::<ParryCollisionObject>();
        assert_eq!(converted.positions, dynamic_obstacle.positions);
        assert_eq!(converted.time_offset, dynamic_obstacle.time_offset);
        assert_eq!(
            converted.convex_hulls.len(),
            dynamic_obstacle.convex_hulls.len()
        );
    }
}
