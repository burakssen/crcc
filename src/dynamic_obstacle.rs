use crate::collision_object::CollisionObject;
use crate::time::TimeStep;
use glamx::DPose2;
use itertools::Itertools;
use std::ops::Range;

pub(crate) struct CCDCollider<'a, C> {
    pub shape: &'a C,
    pub position: &'a DPose2,
    pub next_position: &'a DPose2,
    pub convex_hull: &'a C,
}

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

    pub fn position_at(&self, time_step: TimeStep) -> Option<&DPose2> {
        let with_offset = time_step - self.time_offset;
        self.positions.get(with_offset.0 as usize)
    }

    pub fn convex_hull_after(&self, time_step: TimeStep) -> Option<&C> {
        let with_offset = time_step - self.time_offset;
        self.convex_hulls.get(with_offset.0 as usize)
    }

    pub fn active_times(&self) -> Range<TimeStep> {
        self.time_offset..(self.time_offset + TimeStep(self.positions.len() as i32))
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

    pub(crate) fn ccd_collider_at(&self, time_step: TimeStep) -> Option<CCDCollider<'_, C>> {
        Some(CCDCollider {
            shape: self.shape(),
            position: self.position_at(time_step)?,
            next_position: self.position_at(time_step.succ())?,
            convex_hull: self.convex_hull_after(time_step)?,
        })
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
