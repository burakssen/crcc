# Rust Usage Guide

This guide covers dependency selection, validated geometry, pair and continuous queries, runtime and generic checkers, trajectories, prepared queries, batches, and errors.

See [Concepts and engines](concepts.md) for shared semantics and [Rust API reference](rust-api.md) for method lookup.

## Add the Dependency

CRCC is not currently published to crates.io. Use a Git dependency:

```toml
[dependencies]
crcc = { git = "https://github.com/burakssen/crcc", default-features = false, features = ["parry"] }
geo = "0.32"
```

Or use a local checkout:

```toml
[dependencies]
crcc = { path = "../crcc", default-features = false, features = ["parry"] }
geo = "0.32"
```

The crate uses edition 2024 and does not declare an exact MSRV. Use a recent stable Rust toolchain. Choose only the backend features required by the application.

| Features | Use |
| --- | --- |
| `parry` | Parry backend; recommended starting point. |
| `rhusics` | Rhusics backend. |
| `collide` | Collide backend. |
| `rayon` | Ordered batch methods. |

Default features enable all three engines, but not Rayon.

## Construct Validated Geometry

```rust
use crcc::CollisionObject;

fn main() -> Result<(), crcc::CrccError> {
    let circle = CollisionObject::circle((0.0, 0.0), 0.5)?;
    let rectangle = CollisionObject::rectangle(
        geo::Rect::new((-1.0, -0.5), (1.0, 0.5)),
        0.2,
    )?;
    let triangle = CollisionObject::triangle(geo::Triangle::new(
        geo::Coord { x: 0.0, y: 0.0 },
        geo::Coord { x: 1.0, y: 0.0 },
        geo::Coord { x: 0.0, y: 1.0 },
    ))?;
    let ground = CollisionObject::half_space_from_coeffs(0.0, 1.0, 0.0)?;
    let compound = CollisionObject::merge_all([circle, rectangle, triangle, ground]);

    assert!(!compound.is_empty());
    Ok(())
}
```

Constructors reject non-finite, degenerate, and invalid geometry. Prefer them over directly constructing low-level `SimpleCollisionObject` variants, which can bypass some high-level validation.

## Run Pair Queries

```rust
use crcc::{CollisionEngine, CollisionObject, Pose};

fn main() -> Result<(), crcc::CrccError> {
    let left = CollisionObject::circle((0.0, 0.0), 1.0)?;
    let right = CollisionObject::circle((0.0, 0.0), 1.0)?;
    let right_pose = Pose::translation(3.0, 0.0);

    assert!(!left.collides(
        &right,
        Pose::IDENTITY,
        right_pose,
        CollisionEngine::Parry,
    )?);
    assert_eq!(
        left.distance(
            &right,
            Pose::IDENTITY,
            right_pose,
            CollisionEngine::Parry,
        )?,
        1.0,
    );
    Ok(())
}
```

The Rust `Pose` export is `glamx::DPose2`. Application code is responsible for supplying finite pair-query poses.

## Check Continuous Motion

```rust
use crcc::{CollisionEngine, CollisionObject, Pose};

fn main() -> Result<(), crcc::CrccError> {
    let moving = CollisionObject::circle((0.0, 0.0), 0.5)?;
    let barrier = CollisionObject::rectangle(
        geo::Rect::new((-0.125, -1.5), (0.125, 1.5)),
        0.0,
    )?;

    let possible_hit = moving.collides_continuous(
        Pose::translation(-2.0, 0.0),
        Pose::translation(2.0, 0.0),
        &barrier,
        Pose::IDENTITY,
        Pose::IDENTITY,
        CollisionEngine::Parry,
    )?;
    assert!(possible_hit);
    Ok(())
}
```

`false` certifies interval separation; `true` can be conservative. Backend distinctions are documented in [Engine selection](concepts.md#engine-selection).

## Build a Runtime-Selected Checker

Use `build_with_engine` when the engine is selected by configuration or user input:

```rust
use crcc::{CollisionCheckerBuilder, CollisionEngine, CollisionObject, Pose};

fn main() -> Result<(), crcc::CrccError> {
    let wall = CollisionObject::rectangle(
        geo::Rect::new((-1.0, -1.0), (1.0, 1.0)),
        0.0,
    )?;
    let query = CollisionObject::circle((0.0, 0.0), 0.5)?;

    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(wall)
        .build_with_engine(CollisionEngine::Rhusics)?;

    let status = checker.collides_static_pos(&query, Pose::IDENTITY)?;
    assert!(status.collides());
    assert_eq!(checker.engine(), CollisionEngine::Rhusics);
    Ok(())
}
```

Selecting an engine whose feature is disabled returns `CrccError::Unsupported`.

## Build a Generic Checker

Use `CollisionChecker<E>` for static backend dispatch. Query objects must use backend representation `E`.

```rust
use crcc::collision_checker::engine::parry::ParryCollisionObject;
use crcc::{CollisionChecker, CollisionCheckerBuilder, CollisionObject};

fn main() -> Result<(), crcc::CrccError> {
    let scene = CollisionObject::circle((0.0, 0.0), 1.0)?;
    let query: ParryCollisionObject =
        CollisionObject::circle((0.0, 0.0), 0.25)?.into();

    let checker: CollisionChecker<ParryCollisionObject> =
        CollisionCheckerBuilder::new()
            .with_static_obstacle(scene)
            .build();

    assert!(checker.collides_static(&query)?.collides());
    Ok(())
}
```

Prefer `SelectedCollisionChecker` unless generic dispatch simplifies the surrounding Rust design or avoids repeated runtime selection.

## Add a Dynamic Obstacle

```rust
use crcc::{
    CollisionCheckerBuilder, CollisionEngine, CollisionObject, DynamicObstacle,
    Pose, TimeStep,
};

fn main() -> Result<(), crcc::CrccError> {
    let moving = DynamicObstacle::new(
        CollisionObject::circle((0.0, 0.0), 0.5)?,
        vec![
            Pose::translation(-2.0, 0.0),
            Pose::translation(2.0, 0.0),
        ],
        TimeStep(10),
    )?;
    let barrier = CollisionObject::rectangle(
        geo::Rect::new((-0.125, -1.5), (0.125, 1.5)),
        0.0,
    )?;
    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(barrier)
        .build_with_engine(CollisionEngine::Parry)?;

    let status = checker.collides_dynamic_range(
        &moving,
        TimeStep(10)..=TimeStep(11),
    )?;
    assert_eq!(status, crcc::CollisionStatus::CollidesDynamic(TimeStep(10)));
    Ok(())
}
```

Both endpoints must be in the range for interval `10 -> 11` to run. `collides_dynamic_at(&moving, TimeStep(10))` checks only occupancy at 10.

Use `DynamicObstacle::time_variant` for changing geometry. Shape and pose vectors must have equal length. Empty endpoint geometry suppresses occupancy for the adjacent interval.

## Reuse a Prepared Query

```rust
use crcc::{CollisionCheckerBuilder, CollisionEngine, CollisionObject, Pose};

fn main() -> Result<(), crcc::CrccError> {
    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(CollisionObject::circle((0.0, 0.0), 1.0)?)
        .build_with_engine(CollisionEngine::Parry)?;
    let prepared = checker.prepare_static(
        &CollisionObject::circle((0.0, 0.0), 0.25)?,
    )?;

    let near = checker.collides_static_prepared_range(
        &prepared,
        Pose::IDENTITY,
        ..,
    )?;
    let far = checker.collides_static_prepared_range(
        &prepared,
        Pose::translation(5.0, 0.0),
        ..,
    )?;
    assert!(near.collides() && !far.collides());
    Ok(())
}
```

Prepared queries retain their engine. Cross-engine use returns `CrccError::Unsupported`.

## Run Ordered Batches

Enable `rayon`:

```toml
crcc = { git = "https://github.com/burakssen/crcc", default-features = false, features = ["parry", "rayon"] }
```

```rust
use crcc::{CollisionCheckerBuilder, CollisionEngine, CollisionObject, Pose};

fn main() -> Result<(), crcc::CrccError> {
    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(CollisionObject::circle((0.0, 0.0), 1.0)?)
        .build_with_engine(CollisionEngine::Parry)?;
    let query = CollisionObject::circle((0.0, 0.0), 0.25)?;
    let queries = [
        (query.clone(), Pose::IDENTITY),
        (query, Pose::translation(5.0, 0.0)),
    ];

    let results = checker.collides_static_batch(&queries, ..);
    assert!(results[0].as_ref().is_ok_and(crcc::CollisionStatus::collides));
    assert!(results[1].as_ref().is_ok_and(|status| !status.collides()));
    Ok(())
}
```

Results preserve input order. Automatic execution uses estimated work, active worker count, and indexed iterator grain sizing to choose between sequential execution and the active Rayon pool.

## Handle Errors

```rust
use crcc::{CollisionObject, CrccError};

fn main() {
    match CollisionObject::circle((0.0, 0.0), 0.0) {
        Err(CrccError::InvalidRadius(radius)) => assert_eq!(radius, 0.0),
        other => panic!("unexpected result: {other:?}"),
    }
}
```

`CrccError::Unsupported` means no collision answer was produced. Do not convert it into `false`.

For arithmetic near time limits, prefer `checked_succ`, `checked_add_steps`, or the named saturating helpers. Derived arithmetic operators do not provide the same explicit checked contract.

## Road Boundaries

`with_road_boundary(&lanelets)` accepts `geo::Polygon` values describing drivable regions and adds occupied space outside their union. An empty lanelet slice produces full-space boundary geometry. The Rust crate does not parse CommonRoad XML; scenario parsing and model conversion live in the Python adapter.
