use crate::dynamic_obstacle::{DynamicObstacleTrajectory, GenericDynamicObstacle};
use crate::time::TimeStep;
use glamx::DPose2;

pub struct CCDCollider<'a, C> {
    pub shape: &'a C,
    pub position: DPose2,
    pub next_position: DPose2,
    pub convex_hull: &'a C,
    pub convex_hull_position: DPose2,
}

pub trait CCDColliderAt<C> {
    fn ccd_collider_at(&self, time_step: TimeStep) -> Option<CCDCollider<'_, C>>;
}

impl<C> CCDColliderAt<C> for GenericDynamicObstacle<C> {
    fn ccd_collider_at(&self, time_step: TimeStep) -> Option<CCDCollider<'_, C>> {
        let DynamicObstacleTrajectory::FixedShape {
            shape,
            positions,
            convex_hulls,
        } = &self.trajectory
        else {
            return None;
        };
        let index = time_step.0.checked_sub(self.time_offset.0)? as usize;
        Some(CCDCollider {
            shape,
            position: *positions.get(index)?,
            next_position: *positions.get(index + 1)?,
            convex_hull: convex_hulls.get(index)?,
            convex_hull_position: DPose2::IDENTITY,
        })
    }
}
