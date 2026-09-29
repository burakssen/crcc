# Rust API reference

The crate-root exports are the primary application API. Public module-level types support backend-specific and advanced integrations. [Generated Rustdoc](https://burakssen.com/crcc/rustdoc/crcc/) provides full signatures, bounds, implementation links, and tested examples. It is built from the same source revision as the deployed site with all backends and `rayon`, excluding Python/benchmark internals. Local generation is described in [Building and testing](../development/building-and-testing.md#build-the-documentation-site).

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

```text
object.collides(&other, pos_self, pos_other, engine) -> CrccResult<bool>
object.collides_continuous(start_self, end_self, &other, start_other, end_other, engine)
    -> CrccResult<bool>
object.distance(&other, pos_self, pos_other, engine) -> CrccResult<f64>
```

`merge` and `merge_all` form structural unions. `collision_objects`, `into_collision_objects`, `is_empty`, `is_full_space`, `swept_area`, and `swept_areas` inspect geometry and compute conservative motion bounds.

`collision_objects()` exposes classified domain components, not backend-decomposed triangles. Conversions from `SimpleCollisionObject`, vectors, and iterators normalize empty/full-space components but do not revalidate their geometry. `swept_areas(&[Pose])` returns `len.saturating_sub(1)` objects; current `swept_area(start, end)` returns `Some`, including empty geometry.

Advanced types are in `crcc::collision_object::simple`: `SimpleCollisionObject`, `ConvexPolygon`, `NonConvexPolygon`, `PolygonWithHoles`, and `SweptArea`. Lower-level polygon constructors have weaker validation than the regular constructor. See [error handling](errors-and-limits.md).

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

All query methods return `CollisionResult = Result<CollisionStatus, CrccError>`. Static inputs are `&CollisionObject`; dynamic inputs `&DynamicObstacle`; poses are passed by value. Typed checkers substitute `&E` and `&GenericDynamicObstacle<E>` and do not expose the selected checker's preparation/batch APIs.

Prepared APIs:

```text
prepare_static(&CollisionObject) -> Result<PreparedStaticQuery, CrccError>
prepare_dynamic(&DynamicObstacle) -> Result<PreparedDynamicQuery, CrccError>
collides_static_prepared(&PreparedStaticQuery) -> CollisionResult
collides_static_prepared_range(&PreparedStaticQuery, Pose, impl RangeBounds<TimeStep>)
    -> CollisionResult
collides_dynamic_prepared(&PreparedDynamicQuery) -> CollisionResult
collides_dynamic_prepared_range(&PreparedDynamicQuery, impl RangeBounds<TimeStep>)
    -> CollisionResult
```

Both prepared types expose `engine() -> CollisionEngine` and are cloneable. They can be reused across same-backend checkers; different-backend use returns `Unsupported`. Conversion is cached, not every operation's support certified. Static range queries always check static scene geometry; dynamic range queries select outgoing interval starts. See [time semantics](../concepts/scenes-and-time.md).

Rust time windows use `RangeBounds<TimeStep>`. Batches preserve input order and return one `CollisionResult` per input. They require a `parallel: bool` argument; `false` uses sequential execution and `true` requests Rayon. Heterogeneous batches can mix raw and prepared query variants via `StaticBatchQuery` and `DynamicBatchQuery`.

```text
let results = checker.collides_static_batch(&positioned_queries, .., true);
```

Exact batch input families (all on `SelectedCollisionChecker`):

| Method | Input(s) before `time_range, parallel` |
| --- | --- |
| `collides_static_batch` | `&[(CollisionObject, Pose)]` |
| `collides_dynamic_batch` | `&[DynamicObstacle]` |
| `collides_static_prepared_batch` | `&PreparedStaticQuery, &[Pose]` |
| `collides_dynamic_prepared_batch` | `&[PreparedDynamicQuery]` |
| `collides_static_heterogeneous_batch` | `IntoIterator<Item = (StaticBatchQuery<'a>, Pose)>` |
| `collides_dynamic_heterogeneous_batch` | `IntoIterator<Item = DynamicBatchQuery<'a>>` |

Every method returns `Vec<CollisionResult>`, with `time_range: impl RangeBounds<TimeStep> + Clone + Sync` and `parallel: bool`. The query enums live in `crcc::collision_checker` and offer `Raw(&...)` and `Prepared(&...)`. A heterogeneous wrong-backend entry makes every slot `Unsupported`; the homogeneous dynamic prepared batch retains per-slot mismatch errors. Empty inputs return an empty vector. See the [batch guide](../guides/prepared-and-batch-queries.md) for a runnable example.

## DynamicObstacle and time

`DynamicObstacle::new(shape, positions, time_offset)` creates a fixed-shape trajectory. `DynamicObstacle::time_variant(obstacles, positions, time_offset)` permits a shape per sample. Both return `CrccResult<Self>` and validate poses/time limits. The public `TimeStep` wraps signed 32-bit time; `TimeStepSet` is available in `time` as a `BTreeSet<TimeStep>` alias.

```text
DynamicObstacle::new(CollisionObject, Vec<Pose>, TimeStep) -> CrccResult<DynamicObstacle>
DynamicObstacle::time_variant(Vec<CollisionObject>, Vec<Pose>, TimeStep)
    -> CrccResult<DynamicObstacle>
obstacle.convert_repr::<E: From<CollisionObject>>(self) -> GenericDynamicObstacle<E>
```

`GenericDynamicObstacle<E>` lives in `crcc::collision_object::dynamic`; conversion consumes the trajectory and converts stored shapes/bounds without returning conversion errors. Empty trajectories have no samples; varying shapes and poses require equal counts. Bounds are computed at trajectory construction.

`TimeStep(pub i32)` offers `MIN`, `MAX`, `ZERO`, `pred()`, `succ()`, `add_steps(usize)`, `checked_succ()`, `checked_add_steps(usize)`, and `iter_range(impl RangeBounds<TimeStep>)`. The first arithmetic helpers saturate; checked helpers return `Option<TimeStep>`. Display is `t_<integer>`. Unbounded iteration is lazy but can span all 4,294,967,296 values.

## Runtime engine trait

`collision_checker::engine::EngineCollisionObject` defines conversion from `CollisionObject`, discrete collision, continuous collision, and optional distance. Backend modules expose their concrete representation types. Generic checkers use the trait for static dispatch; normal application code can use root geometry types and `SelectedCollisionChecker`.

Concrete representations:

- `engine::parry::ParryCollisionObject`
- `engine::rhusics::RhusicsCoreCollisionObject`
- `engine::collide::CollideCollisionObject`

`EngineCollisionObject::collides(&self, &Self)` uses identity poses. Required methods are `collides_at(&self, Pose, &Self, Pose)` and `collides_continuous(&self, start_self, end_self, &Self, start_other, end_other)`, returning `CrccResult<bool>`. `distance_at(&self, Pose, &Self, Pose) -> CrccResult<f64>` defaults to `Unsupported`; only Parry overrides it. Runtime/domain Rhusics/Collide distance instead uses the shared fallback.

```rust
use crcc::collision_checker::engine::parry::ParryCollisionObject;
use crcc::collision_checker::engine::EngineCollisionObject;
use crcc::{CollisionCheckerBuilder, CollisionObject, Pose};

fn main() -> Result<(), crcc::CrccError> {
    let obstacle = CollisionObject::circle((0.0, 0.0), 1.0)?;
    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(obstacle)
        .build::<ParryCollisionObject>();
    let query: ParryCollisionObject = CollisionObject::circle((0.0, 0.0), 0.25)?.into();
    assert!(checker.collides_static(&query)?.collides());
    assert!(query.collides_at(Pose::IDENTITY, &query, Pose::IDENTITY)?);
    Ok(())
}
```

The `engine` module also exposes free functions `collides`, `collides_continuous`, and `distance` with interleaved object/pose arguments. Prefer `CollisionObject` methods to avoid confusing their argument order.

## Source maps

- [`lib.rs`](https://github.com/burakssen/crcc/blob/main/src/lib.rs): root exports and crate contract.
- [`collision_object`](https://github.com/burakssen/crcc/tree/main/src/collision_object): geometry and trajectories.
- [`collision_checker`](https://github.com/burakssen/crcc/tree/main/src/collision_checker): builders, scene queries, and backend interface.
- [`time`](https://github.com/burakssen/crcc/blob/main/src/time/mod.rs): discrete-time helpers.
- [`error.rs`](https://github.com/burakssen/crcc/blob/main/src/error.rs): Rust error variants.
