use crate::collision_object::CollisionObject;
use crate::solver::Solver;
use nalgebra::Isometry2;
use parry2d_f64::query::{Unsupported, intersection_test};
use parry2d_f64::shape::{Compound, Shape};

mod builder;
mod collision_object_repr;

use builder::ParrySolverBuilder;
use collision_object_repr::ParryCollisionObjectRepr;

pub struct ParrySolver {
    tri_mesh_objects: Option<Compound>,
    generic_objects: Option<Compound>,
}

impl ParrySolver {
    fn shape_collides(
        &self,
        shape: &dyn Shape,
        position: &Isometry2<f64>,
    ) -> Result<bool, Unsupported> {
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
            Err(Unsupported)
        } else {
            Ok(false)
        }
    }
}

impl Solver for ParrySolver {
    type CollisionObjectRepr = ParryCollisionObjectRepr;
    type CollidesError = Unsupported;

    fn from_collision_objects(collision_objects: Vec<CollisionObject>) -> Self {
        let mut builder = ParrySolverBuilder::new();
        for obj in collision_objects {
            builder.with_collision_object(obj);
        }
        builder.build()
    }

    fn collides(&self, obj: &Self::CollisionObjectRepr) -> Result<bool, Self::CollidesError> {
        match obj {
            ParryCollisionObjectRepr::Empty => Ok(false),
            ParryCollisionObjectRepr::TriMesh(mesh) => {
                self.shape_collides(mesh.as_ref(), &Isometry2::identity())
            }
            ParryCollisionObjectRepr::Generic { shape, position } => {
                self.shape_collides(shape.as_ref(), position)
            }
        }
    }
}
