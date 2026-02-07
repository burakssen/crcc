use crate::collision_object::CollisionObject;
use crate::time::{TimeStep, TimeStepInner};
use glamx::DPose2;
use itertools::Itertools;
use std::ops::Range;

pub type DynamicObstacle = GenericDynamicObstacle<CollisionObject>;

#[derive(Clone, Debug)]
pub struct GenericDynamicObstacle<C> {
    shape: C,
    positions: Vec<DPose2>,
    time_offset: TimeStep,
    convex_hulls: Vec<C>,
}

impl<C> GenericDynamicObstacle<C> {
    pub fn shape(&self) -> &C {
        &self.shape
    }

    pub fn position_at(&self, time_step: TimeStep) -> Option<DPose2> {
        let with_offset = time_step - self.time_offset;
        self.positions.get(with_offset.0 as usize).copied()
    }

    pub fn convex_hull_after(&self, time_step: TimeStep) -> Option<&C> {
        let with_offset = time_step - self.time_offset;
        self.convex_hulls.get(with_offset.0 as usize)
    }

    pub fn active_times(&self) -> Range<TimeStep> {
        self.time_offset..(self.time_offset + TimeStep(self.positions.len() as TimeStepInner))
    }

    pub fn convert_repr<D>(self) -> GenericDynamicObstacle<D>
    where
        C: Into<D>,
    {
        GenericDynamicObstacle {
            shape: self.shape.into(),
            positions: self.positions,
            time_offset: self.time_offset,
            convex_hulls: self.convex_hulls.into_iter().map_into().collect(),
        }
    }
}

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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision_checker::engine::EngineCollisionObject;
    use crate::time::TimeStepSet;
    use geo::Rect;
    use rstest::{fixture, rstest};

    #[fixture]
    fn dynamic_obstacle() -> DynamicObstacle {
        let shape = CollisionObject::rectangle(Rect::new((-2.0, -0.5), (2.0, 0.5)), 0.0);
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
        let active_times: TimeStepSet = dynamic_obstacle.active_times().into();
        assert_eq!(active_times, TimeStepSet::from(TimeStep(5)..=TimeStep(7)));
    }

    #[cfg(feature = "default-engine")]
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
                convex_hull
                    .collides_at(DPose2::IDENTITY, dynamic_obstacle.shape(), interp_pos)
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
