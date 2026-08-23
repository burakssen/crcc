use crate::collision_checker::engine::EngineCollisionObject;
use crate::collision_checker::{CollisionChecker, CollisionResult};
use crate::time::TimeStep;
use glamx::DPose2;
use rayon::prelude::*;

/// A trait for parallel collision checking using Rayon.
/// This trait is sealed.
pub(crate) trait ParallelCollisionChecker: private::Sealed {
    type ECollisionObject: EngineCollisionObject;

    fn collides_static_batch<'a, I>(
        &self,
        positioned_static_obstacles: I,
        active_times: &[TimeStep],
        min_len: usize,
    ) -> Vec<CollisionResult>
    where
        Self::ECollisionObject: 'a,
        I: IntoParallelIterator<Item = (&'a Self::ECollisionObject, DPose2)>,
        I::Iter: IndexedParallelIterator;
}

impl<E: EngineCollisionObject + Sync + Send> ParallelCollisionChecker for CollisionChecker<E> {
    type ECollisionObject = E;

    fn collides_static_batch<'a, I>(
        &self,
        positioned_static_obstacles: I,
        active_times: &[TimeStep],
        min_len: usize,
    ) -> Vec<CollisionResult>
    where
        Self::ECollisionObject: 'a,
        I: IntoParallelIterator<Item = (&'a Self::ECollisionObject, DPose2)>,
        I::Iter: IndexedParallelIterator,
    {
        positioned_static_obstacles
            .into_par_iter()
            .with_min_len(min_len)
            .map(|(obstacle, position)| {
                self.collides_static_active_times(obstacle, position, active_times)
            })
            .collect()
    }
}

mod private {
    use crate::collision_checker::CollisionChecker;
    use crate::collision_checker::engine::EngineCollisionObject;

    pub trait Sealed {}

    impl<E: EngineCollisionObject> Sealed for CollisionChecker<E> {}
}
