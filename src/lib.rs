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
//! # #[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
//! # fn main() -> Result<(), crcc::error::CrccError> {
//! use crcc::collision_checker::CollisionCheckerBuilder;
//! use crcc::collision_checker::engine::CollisionEngine;
//! use crcc::collision_object::CollisionObject;
//!
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
//! # #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
//! # fn main() {}
//! ```
//!
//! Pair-query continuous collision detection is conservative: `false` reports
//! separation under the backend's contact/motion convention, assuming valid rigid
//! poses and representable calculations; `true` may be a conservative positive.
//! Scene windows use Rust ranges over [`TimeStep`]. Static queries require both
//! interval endpoints selected; dynamic queries select outgoing interval starts.
//! Batch methods require `rayon`, even for sequential execution, preserve input
//! order, and retain one result/error per input.
//!
//! # Features and interfaces
//!
//! Default features enable `parry`, `rhusics`, and `collide`. `python_bindings`
//! enables `PyO3` and Rayon and requires a backend. `benchmarking` exposes internal
//! measurement helpers, not an application API. Typed [`CollisionChecker`]
//! instances accept converted backend objects; [`SelectedCollisionChecker`]
//! accepts domain objects and provides backend-specific reusable prepared queries.
//!
//! [`Polygon`] is `geo::Polygon` and [`Pose`] is `glamx::DPose2`. The pose alias
//! adds no validation. Prefer [`CollisionObject`] constructors over partially
//! checked lower-level wrappers. Complex polygons are decomposed on backend
//! conversion, which can defer errors until query time. No public serialization,
//! mutable scene, simulation, or contact-manifold API is provided.
//!
//! See the [user documentation](https://burakssen.com/crcc/) for workflows,
//! `CommonRoad` conversion, numerical assumptions, and backend limitations.

pub mod collision_checker;
pub mod collision_object;
pub mod error;
pub mod time;

pub use collision_checker::engine::CollisionEngine;
pub use collision_checker::{
    CollisionChecker, CollisionCheckerBuilder, CollisionResult, CollisionStatus,
    PreparedDynamicQuery, PreparedStaticQuery, SelectedCollisionChecker,
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
