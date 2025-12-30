use crate::collision_checker::CollisionCheckerError;
use crate::collision_checker::engine::CollisionEngine;
pub use crate::collision_checker::engine::parry::collision_object::ParryCollisionObject;
use nalgebra::Isometry2;
use parry2d_f64::query::Unsupported;

mod collision_object;

pub struct ParryEngine {}

impl CollisionEngine for ParryEngine {
    type EngineCollisionObject = ParryCollisionObject;

    fn collides(
        obj_1: &Self::EngineCollisionObject,
        pos_1: &Isometry2<f64>,
        obj_2: &Self::EngineCollisionObject,
        pos_2: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError> {
        obj_1
            .0
            .collides(pos_1, &obj_2.0, pos_2)
            .map_err(|err| match err {
                // Deliberate match with one arm to future-proof against new error variants in parry
                Unsupported => CollisionCheckerError::Unsupported,
            })
    }

    fn collides_continuous(
        obj_1: &Self::EngineCollisionObject,
        start_pos_1: &Isometry2<f64>,
        end_pos_1: &Isometry2<f64>,
        obj_2: &Self::EngineCollisionObject,
        start_pos_2: &Isometry2<f64>,
        end_pos_2: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError> {
        obj_1
            .0
            .collides_continuous(start_pos_1, end_pos_1, &obj_2.0, start_pos_2, end_pos_2)
            .map_err(|err| match err {
                // Deliberate match with one arm to future-proof against new error variants in parry
                Unsupported => CollisionCheckerError::Unsupported,
            })
    }
}
