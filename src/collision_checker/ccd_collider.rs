use crate::collision_object::dynamic::{DynamicObstacleTrajectory, GenericDynamicObstacle};
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
        let relative_time = time_step.0.checked_sub(self.time_offset.0)?;

        let index = usize::try_from(relative_time).ok()?;
        let next_index = index.checked_add(1)?;

        match &self.trajectory {
            DynamicObstacleTrajectory::FixedShape {
                shape,
                positions,
                convex_hulls,
            } => Some(CCDCollider {
                shape,
                position: *positions.get(index)?,
                next_position: *positions.get(next_index)?,
                convex_hull: convex_hulls.get(index)?,
                convex_hull_position: DPose2::IDENTITY,
            }),

            DynamicObstacleTrajectory::VaryingShape { convex_hulls, .. } => {
                let swept_area = convex_hulls.get(index)?;

                Some(CCDCollider {
                    shape: swept_area,
                    position: DPose2::IDENTITY,
                    next_position: DPose2::IDENTITY,
                    convex_hull: swept_area,
                    convex_hull_position: DPose2::IDENTITY,
                })
            }
        }
    }
}
