use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_checker::{CollisionChecker, CollisionResult};
use crate::dynamic_obstacle::GenericDynamicObstacle;
use crate::time::TimeStep;
use glamx::DPose2;
use rayon::prelude::*;
use std::ops::RangeBounds;

/// A trait for parallel collision checking using Rayon.
/// This trait is sealed.
pub trait ParallelCollisionChecker: private::Sealed {
    type ECollisionObject: EngineCollisionObject;

    fn par_collides_static<'a, I>(
        &self,
        positioned_static_obstacles: I,
        time_range: impl RangeBounds<TimeStep> + Copy + Sync,
    ) -> Vec<CollisionResult>
    where
        Self::ECollisionObject: 'a,
        I: IntoParallelIterator<Item = (&'a Self::ECollisionObject, DPose2)>;

    fn par_collides_dynamic<'a, I>(
        &self,
        dynamic_obstacles: I,
        time_range: impl RangeBounds<TimeStep> + Copy + Sync,
    ) -> Vec<CollisionResult>
    where
        Self::ECollisionObject: 'a,
        I: IntoParallelIterator<Item = &'a GenericDynamicObstacle<Self::ECollisionObject>>;
}

impl<E: EngineCollisionObject + Sync + Send> ParallelCollisionChecker for CollisionChecker<E> {
    type ECollisionObject = E;

    fn par_collides_static<'a, I>(
        &self,
        positioned_static_obstacles: I,
        time_range: impl RangeBounds<TimeStep> + Copy + Sync,
    ) -> Vec<CollisionResult>
    where
        Self::ECollisionObject: 'a,
        I: IntoParallelIterator<Item = (&'a Self::ECollisionObject, DPose2)>,
    {
        positioned_static_obstacles
            .into_par_iter()
            .map(|(obs, pos)| self.collides_static_range(obs, pos, time_range))
            .collect()
    }

    fn par_collides_dynamic<'a, I>(
        &self,
        dynamic_obstacles: I,
        time_range: impl RangeBounds<TimeStep> + Copy + Sync,
    ) -> Vec<CollisionResult>
    where
        Self::ECollisionObject: 'a,
        I: IntoParallelIterator<Item = &'a GenericDynamicObstacle<Self::ECollisionObject>>,
    {
        dynamic_obstacles
            .into_par_iter()
            .map(|obs| self.collides_dynamic_range(obs, time_range))
            .collect()
    }
}

mod private {
    use crate::collision_checker::CollisionChecker;
    use crate::collision_checker::engine::EngineCollisionObject;

    pub trait Sealed {}

    impl<E: EngineCollisionObject> Sealed for CollisionChecker<E> {}
}
