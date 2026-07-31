//! Runtime-selectable 2D collision checking for Rust and Python.
//!
//! CRCC represents geometry as [`CollisionObject`] values, places it with [`Pose`],
//! and dispatches pair or scene queries through a [`CollisionEngine`]. A
//! [`SelectedCollisionChecker`] combines immutable static geometry with optional
//! [`DynamicObstacle`] trajectories.
//!
//! # Quick start
//!
//! ```
//! use crcc::collision_checker::CollisionCheckerBuilder;
//! use crcc::collision_checker::engine::CollisionEngine;
//! use crcc::collision_object::CollisionObject;
//!
//! # fn main() -> Result<(), crcc::error::CrccError> {
//! let wall = CollisionObject::rectangle(
//!     geo::Rect::new((-1.0, -1.0), (1.0, 1.0)),
//!     0.0,
//! )?;
//! let robot = CollisionObject::circle((0.0, 0.0), 0.5)?;
//! let checker = CollisionCheckerBuilder::new()
//!     .with_static_obstacle(wall)
//!     .build_with_engine(CollisionEngine::default())?;
//!
//! let status = checker.collides_static(&robot)?;
//! assert!(status.collides());
//! # Ok(())
//! # }
//! ```
//!
//! Pair-query continuous collision detection is conservative: `false` certifies
//! separation over the motion, while `true` may be a conservative positive.
//! Scene time windows use ordinary Rust ranges over [`TimeStep`]. Batch methods
//! are available with the `rayon` feature and preserve input order.

pub mod collision_checker;
pub mod collision_object;
pub mod error;
pub mod time;

pub use collision_checker::engine::CollisionEngine;
pub use collision_checker::{
    CollisionChecker, CollisionCheckerBuilder, CollisionResult, CollisionStatus,
    SelectedCollisionChecker,
};
pub use collision_object::CollisionObject;
pub use collision_object::DynamicObstacle;
pub use collision_object::simple::{Circle, Empty, FullSpace, HalfSpace, Rectangle, Triangle};
pub use error::{CrccError, CrccResult};
pub use geo::Polygon;
pub use glamx::DPose2 as Pose;
pub use time::TimeStep;

/// A semantic alias for a [`CollisionObject`] formed by merging multiple objects.
pub type Compound = CollisionObject;

#[cfg(feature = "benchmarking")]
#[doc(hidden)]
pub mod benchmark_support {
    pub use crate::collision_checker::CollisionChecker as EngineChecker;

    #[cfg(feature = "collide")]
    pub use crate::collision_checker::engine::collide::CollideCollisionObject;

    #[cfg(feature = "parry")]
    pub use crate::collision_checker::engine::parry::ParryCollisionObject;

    #[cfg(feature = "rhusics")]
    pub use crate::collision_checker::engine::rhusics::RhusicsCoreCollisionObject;

    pub use crate::collision_checker::engine::{
        EngineCollisionObject, collides, collides_continuous, distance,
    };
    pub use crate::collision_object::dynamic::GenericDynamicObstacle as EngineDynamicObstacle;
    pub use crate::collision_object::simple::SimpleCollisionObject;

    #[must_use]
    pub fn build_typed<E: EngineCollisionObject>(
        builder: crate::CollisionCheckerBuilder,
    ) -> EngineChecker<E> {
        builder.build()
    }

    #[must_use]
    pub fn convert_dynamic<E: EngineCollisionObject>(
        obstacle: crate::DynamicObstacle,
    ) -> EngineDynamicObstacle<E> {
        obstacle.convert_repr()
    }
}

#[cfg(feature = "python_bindings")]
mod python;
