# Rust API reference

The crate-root exports are the primary application API. Public module-level types support backend-specific and advanced integrations. Source rustdoc remains authoritative for full generic bounds.

## Crate-root exports

```text
CollisionObject Compound Circle Rectangle Triangle HalfSpace Empty FullSpace
Polygon Pose DynamicObstacle TimeStep
CollisionChecker CollisionCheckerBuilder SelectedCollisionChecker
PreparedStaticQuery PreparedDynamicQuery CollisionEngine CollisionStatus
CollisionResult CrccError CrccResult
```

`Compound` aliases `CollisionObject`; `Polygon` is `geo::Polygon`; `Pose` is `glamx::DPose2`.

## Cargo features

| Feature | Default | Effect |
| --- | --- | --- |
| `parry` | Yes | Parry 2D backend. |
| `rhusics` | Yes | Rhusics backend and its geometry dependencies. |
| `collide` | Yes | Collide backend and its geometry dependencies. |
| `rayon` | No | Batch-query methods on runtime-selected checkers. |
| `python_bindings` | No | PyO3 bindings; also enables `rayon`. Requires at least one backend. |
| `benchmarking` | No | Benchmark-only exports and native benchmark binary. |

With `default-features = false`, enable at least one backend for collision queries. Selecting a disabled backend through `build_with_engine` returns `CrccError::Unsupported`.

## Geometry and pair queries

Validated constructors on `CollisionObject` include `empty()`, `full_space()`, `circle(center, radius)`, `rectangle(rect, orientation)`, `triangle(triangle)`, `polygon(polygon)`, `half_space(normal, offset)`, `half_space_from_points(p1, p2)`, and `half_space_from_coeffs(a, b, c)`. Except for empty/full space, constructors return `CrccResult<Self>`.

```rust
object.collides(&other, pos_self, pos_other, engine) -> CrccResult<bool>
object.collides_continuous(start_self, end_self, &other, start_other, end_other, engine)
    -> CrccResult<bool>
object.distance(&other, pos_self, pos_other, engine) -> CrccResult<f64>
```

`merge` and `merge_all` form structural unions. `collision_objects`, `into_collision_objects`, `is_empty`, `is_full_space`, `swept_area`, and `swept_areas` inspect geometry and compute conservative motion bounds.

## Checker construction and queries

`CollisionCheckerBuilder::new()` chains `with_static_obstacle`, `with_dynamic_obstacle`, and optionally `with_road_boundary`. Use `build::<E>()` for a typed `CollisionChecker<E>`, or `build_with_engine(engine)` for a `Result<SelectedCollisionChecker, CrccError>`.

Typed checkers take backend-converted objects and provide static, dynamic, positioned, single-step, and range queries. Runtime-selected checkers accept domain `CollisionObject` and `DynamicObstacle` values. They also expose `prepare_static`, `prepare_dynamic`, direct queries, prepared queries, and (with `rayon`) batch queries.

Common runtime-selected checker methods include:

```text
engine()
collides_static(object)                 collides_dynamic(obstacle)
collides_static_at(object, time_step)   collides_dynamic_at(obstacle, time_step)
collides_static_pos(object, pose)       collides_static_pos_at(object, pose, time_step)
collides_static_range(object, pose, range)
collides_dynamic_range(obstacle, range)
collides_static_prepared(query)         collides_dynamic_prepared(query)
```

`CollisionStatus::collides(self) -> bool` reports whether the returned status is a collision. `CollidesDynamic(t)` attributes a sample or between-step interval to `t`.

Rust time windows use `RangeBounds<TimeStep>`. Batches preserve input order and return one `CollisionResult` per input. They require a `parallel: bool` argument; `false` uses sequential execution and `true` requests Rayon. Heterogeneous batches can mix raw and prepared query variants via `StaticBatchQuery` and `DynamicBatchQuery`.

```rust
let results = checker.collides_static_batch(&positioned_queries, .., true);
```

## DynamicObstacle and time

`DynamicObstacle::new(shape, positions, time_offset)` creates a fixed-shape trajectory. `DynamicObstacle::time_variant(obstacles, positions, time_offset)` permits a shape per sample. Both return `CrccResult<Self>` and validate poses/time limits. The public `TimeStep` wraps signed 32-bit time; `TimeStepSet` is available in `time` as a `BTreeSet<TimeStep>` alias.

## Runtime engine trait

`collision_checker::engine::EngineCollisionObject` defines conversion from `CollisionObject`, discrete collision, continuous collision, and optional distance. Backend modules expose their concrete representation types. Generic checkers use the trait for static dispatch; normal application code can use root geometry types and `SelectedCollisionChecker`.

## Source maps

- [`lib.rs`](https://github.com/burakssen/crcc/blob/main/src/lib.rs): root exports and crate contract.
- [`collision_object`](https://github.com/burakssen/crcc/tree/main/src/collision_object): geometry and trajectories.
- [`collision_checker`](https://github.com/burakssen/crcc/tree/main/src/collision_checker): builders, scene queries, and backend interface.
- [`time`](https://github.com/burakssen/crcc/blob/main/src/time/mod.rs): discrete-time helpers.
- [`error.rs`](https://github.com/burakssen/crcc/blob/main/src/error.rs): Rust error variants.
