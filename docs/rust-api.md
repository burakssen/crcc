# Rust API Reference

This reference summarizes CRCC's public Rust API. Start with the [Rust guide](rust-guide.md) for complete examples.

Angles are in radians. Fallible operations return `CrccResult<T>`.

## Cargo features

| Feature | Purpose |
| --- | --- |
| `parry`, `rhusics`, `collide` | Enable collision backends; all are enabled by default. |
| `rayon` | Enable ordered batch-query methods. |
| `python_bindings` | Build the PyO3 extension; also enables `rayon`. |
| `benchmarking` | Expose internal benchmark support. Not part of the library API. |

## Root exports

The crate root exports the common application-facing types:

- Geometry: `CollisionObject`, `Circle`, `Rectangle`, `Triangle`, `HalfSpace`, `Polygon`, `Compound`, `Empty`, `FullSpace`.
- Placement and time: `Pose`, `TimeStep`.
- Scenes: `CollisionChecker`, `SelectedCollisionChecker`, `CollisionCheckerBuilder`, `DynamicObstacle`.
- Results: `CollisionStatus`, `CollisionResult`, `CrccError`, `CrccResult`.
- Backend selection: `CollisionEngine`.

## `CollisionObject`

### Constructors

| Method | Result |
| --- | --- |
| `empty()` / `full_space()` | `CollisionObject` |
| `circle(center, radius)` | `CrccResult<CollisionObject>` |
| `rectangle(rect, orientation)` | `CrccResult<CollisionObject>` |
| `triangle(triangle)` | `CrccResult<CollisionObject>` |
| `polygon(polygon)` | `CrccResult<CollisionObject>` |
| `half_space(normal, offset)` | `CrccResult<CollisionObject>` |
| `half_space_from_points(p1, p2)` | `CrccResult<CollisionObject>` |
| `half_space_from_coeffs(a, b, c)` | `CrccResult<CollisionObject>` |

### Queries and composition

| Method | Purpose |
| --- | --- |
| `collides(...)` | Discrete pair collision through a selected engine. |
| `collides_continuous(...)` | Conservative pair collision over a motion interval. |
| `distance(...)` | Pair separation distance. |
| `merge(other)` / `merge_all(objects)` | Union geometry. |
| `is_empty()` / `is_full_space()` | Inspect special geometry. |
| `swept_area(start, end)` / `swept_areas(poses)` | Conservative swept geometry. |

## Collision checkers

### `CollisionCheckerBuilder`

| Method | Purpose |
| --- | --- |
| `new()` | Empty builder. |
| `with_static_obstacle(object)` | Add fixed geometry. |
| `with_dynamic_obstacle(obstacle)` | Add a trajectory. |
| `with_road_boundary(lanelets)` | Add the region outside lanelet polygons. |
| `build::<E>()` | Generic checker with static backend dispatch. |
| `build_with_engine(engine)` | Runtime-selected checker. |

### `CollisionChecker<E>`

The generic checker accepts objects converted to engine representation `E`. Its main methods are `collides_static`, `collides_dynamic`, and their `_at`, `_pos`, and `_range` variants.

### `SelectedCollisionChecker`

The runtime-selected checker accepts ordinary `CollisionObject` and `DynamicObstacle` values.

| Method | Purpose |
| --- | --- |
| `engine()` | Active `CollisionEngine`. |
| `collides_static(...)` / `collides_dynamic(...)` | Query the full supported time domain. |
| `collides_static_at(...)` / `collides_dynamic_at(...)` | Query one `TimeStep`. |
| `collides_static_range(...)` / `collides_dynamic_range(...)` | Query a Rust time range. |
| `collides_static_batch(...)` / `collides_dynamic_batch(...)` | Ordered batch query; requires `rayon`. |

## Dynamic obstacles and time

| API | Purpose |
| --- | --- |
| `DynamicObstacle::new(shape, poses, time_offset)` | Constant geometry across a trajectory. |
| `DynamicObstacle::time_variant(shapes, poses, time_offset)` | Geometry that changes by step. |
| `TimeStep::pred()` / `succ()` | Saturating adjacent step. |
| `TimeStep::add_steps(count)` | Saturating forward offset. |
| `TimeStep::iter_range(range)` | Iterate the steps selected by a Rust range. |

## Results and errors

`CollisionStatus` is `NoCollision`, `CollidesStatic`, or `CollidesDynamic(TimeStep)`. Its `collides()` method is the simplest way to inspect a result.

`CrccError` reports invalid radius, non-convex or holed geometry where unsupported, empty or invalid geometry, and unsupported engine/operation combinations.
