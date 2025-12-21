use crate::collision_checker::CollisionCheckerError;
use crate::collision_checker::engine::CollisionEngine;
use crate::collision_checker::engine::parry::builder::ParryEngineBuilder;
use crate::collision_checker::engine::parry::collision_object::ParryCollisionObjectInner;
use crate::collision_object::CollisionObject;
use nalgebra::Isometry2;
use parry2d_f64::query::{Unsupported, intersection_test};
use parry2d_f64::shape::{Compound, Shape};

pub use crate::collision_checker::engine::parry::collision_object::ParryCollisionObject;

mod builder;
mod collision_object;

pub struct ParryEngine {
    tri_mesh_objects: Option<Compound>,
    generic_objects: Option<Compound>,
}

impl ParryEngine {
    fn shape_collides(
        &self,
        shape: &dyn Shape,
        position: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError> {
        let mut unsupported = false;
        for obs in [&self.generic_objects, &self.tri_mesh_objects]
            .into_iter()
            .flatten()
        {
            let collides = intersection_test(&Isometry2::identity(), obs, position, shape);
            unsupported |= matches!(collides, Err(Unsupported));
            if let Ok(true) = collides {
                return Ok(true);
            }
        }
        if unsupported {
            Err(CollisionCheckerError::Unsupported)
        } else {
            Ok(false)
        }
    }
}

impl CollisionEngine for ParryEngine {
    type EngineCollisionObject = ParryCollisionObject;

    fn from_collision_objects(collision_objects: Vec<CollisionObject>) -> Self {
        let mut builder = ParryEngineBuilder::new();
        for obj in collision_objects {
            builder = builder.with_collision_object(obj);
        }
        builder.build()
    }

    fn collides_at(
        &self,
        obj: &Self::EngineCollisionObject,
        position: &Isometry2<f64>,
    ) -> Result<bool, CollisionCheckerError> {
        match &obj.0 {
            ParryCollisionObjectInner::Empty => Ok(false),
            ParryCollisionObjectInner::TriMesh(mesh) => {
                self.shape_collides(mesh.as_ref(), position)
            }
            ParryCollisionObjectInner::Generic {
                shape,
                position: shape_position,
            } => self.shape_collides(shape.as_ref(), &(*position * *shape_position)),
        }
    }
}
