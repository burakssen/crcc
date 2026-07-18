# Rust Guide

This guide covers geometry, collision engines, scene construction, time windows, and batch queries in Rust.

For signatures and exported types, see the [Rust API reference](rust-api.md).

## Setup

For a local checkout:

```toml
[dependencies]
crcc = { path = "../crcc" }
geo = "0.32"
```

The default features enable the Parry, Rhusics, and Collide engines. Add `rayon` when batch queries are required:

```toml
crcc = { path = "../crcc", features = ["rayon"] }
```

## Pair queries

`CollisionObject` provides direct collision and distance queries:

```rust
use crcc::{CollisionEngine, CollisionObject, Pose};

# fn main() -> Result<(), crcc::CrccError> {
let left = CollisionObject::circle((0.0, 0.0), 1.0)?;
let right = CollisionObject::circle((0.0, 0.0), 1.0)?;
let right_pose = Pose::translation(3.0, 0.0);

assert!(!left.collides(&right, Pose::IDENTITY, right_pose, CollisionEngine::Parry)?);
assert_eq!(left.distance(&right, Pose::IDENTITY, right_pose, CollisionEngine::Parry)?, 1.0);
# Ok(())
# }
```

## Continuous collision detection

Use `collides_continuous` when objects move between poses:

```rust
use crcc::{CollisionEngine, CollisionObject, Pose};

# fn main() -> Result<(), crcc::CrccError> {
let moving = CollisionObject::circle((0.0, 0.0), 0.5)?;
let barrier = CollisionObject::rectangle(
    geo::Rect::new((-0.125, -1.5), (0.125, 1.5)),
    0.0,
)?;

let hit = moving.collides_continuous(
    Pose::translation(-2.0, 0.0),
    Pose::translation(2.0, 0.0),
    &barrier,
    Pose::IDENTITY,
    Pose::IDENTITY,
    CollisionEngine::Parry,
)?;
assert!(hit);
# Ok(())
# }
```

Continuous queries are conservative: `false` certifies separation, while `true` may be a conservative positive.

## Runtime-selected checker

Use `build_with_engine` when the engine is chosen at runtime:

```rust
use crcc::{CollisionCheckerBuilder, CollisionEngine, CollisionObject};

# fn main() -> Result<(), crcc::CrccError> {
let wall = CollisionObject::rectangle(
    geo::Rect::new((-1.0, -1.0), (1.0, 1.0)),
    0.0,
)?;
let query = CollisionObject::circle((0.0, 0.0), 0.5)?;

let checker = CollisionCheckerBuilder::new()
    .with_static_obstacle(wall)
    .build_with_engine(CollisionEngine::Rhusics)?;

assert!(checker.collides_static(&query)?.collides());
assert_eq!(checker.engine(), CollisionEngine::Rhusics);
# Ok(())
# }
```

## Generic checker

Use `build::<E>()` when the backend type is known at compile time. Query objects must use the same engine representation.

```rust
use crcc::collision_checker::engine::parry::ParryCollisionObject;
use crcc::{CollisionChecker, CollisionCheckerBuilder, CollisionObject};

# fn main() -> Result<(), crcc::CrccError> {
let scene = CollisionObject::circle((0.0, 0.0), 1.0)?;
let query: ParryCollisionObject = CollisionObject::circle((0.0, 0.0), 0.25)?.into();

let checker: CollisionChecker<ParryCollisionObject> = CollisionCheckerBuilder::new()
    .with_static_obstacle(scene)
    .build();
assert!(checker.collides_static(&query)?.collides());
# Ok(())
# }
```

Prefer the runtime-selected checker unless static dispatch is useful to the surrounding application.

## Dynamic obstacles and time ranges

Rust uses `TimeStep` and ordinary inclusive or exclusive ranges:

```rust
use crcc::{CollisionCheckerBuilder, CollisionEngine, CollisionObject, DynamicObstacle, Pose, TimeStep};

# fn main() -> Result<(), crcc::CrccError> {
let moving = DynamicObstacle::new(
    CollisionObject::circle((0.0, 0.0), 0.5)?,
    vec![Pose::translation(-2.0, 0.0), Pose::translation(2.0, 0.0)],
    TimeStep(10),
);
let barrier = CollisionObject::rectangle(
    geo::Rect::new((-0.125, -1.5), (0.125, 1.5)),
    0.0,
)?;
let checker = CollisionCheckerBuilder::new()
    .with_static_obstacle(barrier)
    .build_with_engine(CollisionEngine::Parry)?;

let status = checker.collides_dynamic_range(&moving, TimeStep(10)..=TimeStep(11))?;
assert!(status.collides());
# Ok(())
# }
```

Use `DynamicObstacle::time_variant` when geometry changes between steps.

## Batch queries

Batch methods require the `rayon` feature, preserve input order, and select sequential or parallel execution based on batch size.

```rust
use crcc::{CollisionCheckerBuilder, CollisionEngine, CollisionObject, Pose};

# fn main() -> Result<(), crcc::CrccError> {
let scene = CollisionObject::circle((0.0, 0.0), 1.0)?;
let query = CollisionObject::circle((0.0, 0.0), 0.25)?;
let checker = CollisionCheckerBuilder::new()
    .with_static_obstacle(scene)
    .build_with_engine(CollisionEngine::Parry)?;

let results = checker.collides_static_batch(
    &[(query.clone(), Pose::IDENTITY), (query, Pose::translation(5.0, 0.0))],
    ..,
);
assert!(results[0].as_ref().unwrap().collides());
assert!(!results[1].as_ref().unwrap().collides());
# Ok(())
# }
```

Use `collides_dynamic_batch` for multiple dynamic queries.

## Accuracy notes

- Translation is interpolated linearly.
- Rotation uses the shortest path between orientations.
- Translation-only convex sweeps are exact; rotational sweeps can be conservative.
- Smaller trajectory intervals can tighten conservative rotational bounds.
