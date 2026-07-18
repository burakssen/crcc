use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_checker::{CollisionChecker, CollisionResult};
use crate::collision_object::dynamic::GenericDynamicObstacle;
use crate::time::TimeStep;
use glamx::DPose2;
use rayon::prelude::*;
use std::ops::RangeBounds;

/// A trait for parallel collision checking using Rayon.
/// This trait is sealed.
pub(crate) trait ParallelCollisionChecker: private::Sealed {
    type ECollisionObject: EngineCollisionObject;

    fn collides_static_batch<'a, I>(
        &self,
        positioned_static_obstacles: I,
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult>
    where
        Self::ECollisionObject: 'a,
        I: IntoParallelIterator<Item = (&'a Self::ECollisionObject, DPose2)>;

    fn collides_dynamic_batch<'a, I>(
        &self,
        dynamic_obstacles: I,
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult>
    where
        Self::ECollisionObject: 'a,
        I: IntoParallelIterator<Item = &'a GenericDynamicObstacle<Self::ECollisionObject>>;
}

impl<E: EngineCollisionObject + Sync + Send> ParallelCollisionChecker for CollisionChecker<E> {
    type ECollisionObject = E;

    fn collides_static_batch<'a, I>(
        &self,
        positioned_static_obstacles: I,
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult>
    where
        Self::ECollisionObject: 'a,
        I: IntoParallelIterator<Item = (&'a Self::ECollisionObject, DPose2)>,
    {
        positioned_static_obstacles
            .into_par_iter()
            .map(|(obstacle, position)| {
                self.collides_static_range(obstacle, position, time_range.clone())
            })
            .collect()
    }

    fn collides_dynamic_batch<'a, I>(
        &self,
        dynamic_obstacles: I,
        time_range: impl RangeBounds<TimeStep> + Clone + Sync,
    ) -> Vec<CollisionResult>
    where
        Self::ECollisionObject: 'a,
        I: IntoParallelIterator<Item = &'a GenericDynamicObstacle<Self::ECollisionObject>>,
    {
        dynamic_obstacles
            .into_par_iter()
            .map(|obstacle| self.collides_dynamic_range(obstacle, time_range.clone()))
            .collect()
    }
}

mod private {
    use crate::collision_checker::CollisionChecker;
    use crate::collision_checker::engine::EngineCollisionObject;

    pub trait Sealed {}

    impl<E: EngineCollisionObject> Sealed for CollisionChecker<E> {}
}
