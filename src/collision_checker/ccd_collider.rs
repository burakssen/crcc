use crate::dynamic_obstacle::GenericDynamicObstacle;
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
        Some(CCDCollider {
            shape: self.shape(),
            position: self.position_at(time_step)?,
            next_position: self.position_at(time_step.succ())?,
            convex_hull: self.convex_hull_after(time_step)?,
            convex_hull_position: DPose2::IDENTITY,
        })
    }
}
