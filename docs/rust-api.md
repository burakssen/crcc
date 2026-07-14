# Rust API reference

CRCC 0.1 groups its Rust API by concept. Use `crcc::collision_object`, `crcc::collision_checker`, `crcc::dynamic_obstacle`, `crcc::time`, and `crcc::error`. Angles are radians.

## Cargo features

```toml
[dependencies]
crcc = { path = "../crcc", default-features = false, features = ["parry", "rayon"] }
geo = "0.32"
glamx = "0.1"
```

- `parry`, `rhusics`, and `collide` compile their corresponding engine representation. All are enabled by default.
- `rayon` exposes parallel batch traits and runtime-selected batch methods.
- `python_bindings` builds the PyO3 extension and enables Rayon.

The repository's benchmark feature supports research binaries and is not part of the library interface.

## Collision objects

`crcc::collision_object::CollisionObject` stores the union of zero or more simple objects.

Constructors:

- `empty()`, `full_space()`
- `circle(center, radius) -> CrccResult<CollisionObject>`
- `rectangle(rect, orientation) -> CrccResult<CollisionObject>`
- `triangle(triangle) -> CrccResult<CollisionObject>`
- `polygon(polygon) -> CrccResult<CollisionObject>`
- `half_space(normal, offset) -> CrccResult<CollisionObject>`
- `half_space_from_points(p1, p2) -> CrccResult<CollisionObject>`
- `half_space_from_coeffs(a, b, c) -> CrccResult<CollisionObject>`

Composition and inspection:

- `merge(self, other)` and `merge_all(objects)`
- `collision_objects()` and `into_collision_objects()`
- `is_empty()` and `is_full_space()`
- `swept_areas(positions)` and `swept_area(start, end)`

Discrete pair queries, distance, and pair CCD are provided by `crcc::collision_checker::engine::{collides, distance, collides_continuous}`. They accept `CollisionObject`, `glamx::DPose2`, and a `CollisionEngine`.

`crcc::collision_object::simple` exposes `Circle`, `Rectangle`, `Triangle`, `HalfSpace`, `Empty`, `FullSpace`, and the internal shape enum used by engine representations.

## Engines

`crcc::collision_checker::engine::CollisionEngine` has `Parry`, `Rhusics`, and `Collide` variants. Each enabled backend module exports its engine representation:

- `engine::parry::ParryCollisionObject`
- `engine::rhusics::RhusicsCoreCollisionObject`
- `engine::collide::CollideCollisionObject`

These types implement `EngineCollisionObject` and can be used with the generic checker.

## Generic checker

`CollisionCheckerBuilder::build::<E>() -> CollisionChecker<E>` converts the scene to one engine representation at compile time:

```rust
use crcc::collision_checker::CollisionCheckerBuilder;
use crcc::collision_checker::engine::parry::ParryCollisionObject;
use crcc::collision_object::CollisionObject;

# fn main() -> Result<(), crcc::error::CrccError> {
let scene = CollisionObject::circle((0.0, 0.0), 1.0)?;
let query: ParryCollisionObject = CollisionObject::circle((0.0, 0.0), 0.25)?.into();
let checker = CollisionCheckerBuilder::new()
    .with_static_obstacle(scene)
    .build::<ParryCollisionObject>();
assert!(checker.collides_static(&query)?.collides());
# Ok(())
# }
```

`CollisionChecker<E>` provides `collides_static`, `collides_dynamic`, `collides_static_at`, `collides_dynamic_at`, `collides_static_pos`, `collides_static_pos_at`, and the range-based variants.

With `rayon`, `crcc::collision_checker::parallel::ParallelCollisionChecker` provides generic parallel query helpers.

## Runtime-selected checker

`CollisionCheckerBuilder::build_with_engine(engine) -> CrccResult<SelectedCollisionChecker>` selects a backend at runtime. `SelectedCollisionChecker` accepts public `CollisionObject` and `DynamicObstacle` values directly:

- `engine()`
- `collides_static(&object)` and `collides_dynamic(&obstacle)`
- `collides_static_at`, `collides_dynamic_at`, `collides_static_pos`, and `collides_static_pos_at`
- `collides_static_range(&object, pose, range)` and `collides_dynamic_range(&obstacle, range)`
- with `rayon`, `par_static(...)` and `par_dynamic(...)`

Batch output order matches input order. Small batches execute sequentially; larger batches use the active Rayon pool.

## Builder

`CollisionCheckerBuilder` supports:

- `new()`
- `with_static_obstacle(object)`
- `with_dynamic_obstacle(obstacle)`
- `with_road_boundary(lanelet_polygons)`
- `build::<E>()`
- `build_with_engine(engine)`

The built scene is immutable. Rebuild it when scene objects change.

## Dynamic obstacles and time

`crcc::dynamic_obstacle::DynamicObstacle::new(shape, positions, time_offset)` creates a fixed-shape trajectory. `time_variant(objects, positions, time_offset)` permits one shape per step and requires equal vector lengths. `convert_repr::<E>()` converts it for a generic checker.

`crcc::time::TimeStep(pub i32)` provides `MIN`, `MAX`, `ZERO`, `pred`, `succ`, `add_steps`, and `iter_range`. Standard Rust range inclusion rules apply. Include both `t` and `t + 1` to check motion across that segment.

## Status and errors

`CollisionStatus` is `NoCollision`, `CollidesStatic`, or `CollidesDynamic(TimeStep)`. `collides()` returns whether either collision variant occurred. `CollisionResult` is `Result<CollisionStatus, CrccError>`.

`crcc::error::CrccError` variants are `InvalidRadius`, `NotConvex`, `HasHoles`, `EmptyShape`, `InvalidGeometry`, and `Unsupported`. `CrccResult<T>` is the common result alias.

## Generate documentation

```bash
cargo test --doc
cargo doc --no-deps
cargo doc --no-deps --all-features
```

See the [usage guide](usage.md) for Python/Rust task examples.
