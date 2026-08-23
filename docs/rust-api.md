# Rust API Reference

This page maps the public Rust surface. The source rustdoc remains authoritative for generic bounds and implementation-level backend types. Start with the [Rust usage guide](rust-guide.md) for runnable workflows.

Fallible domain operations use `CrccResult<T> = Result<T, CrccError>`.

## Cargo Features

| Feature | Default | Effect |
| --- | ---: | --- |
| `parry` | Yes | Enables `parry2d-f64` and Parry representation/query code. |
| `rhusics` | Yes | Enables Rhusics, cgmath, and collision dependencies. |
| `collide` | Yes | Enables the Collide dependency family. |
| `rayon` | No | Enables ordered batch APIs. |
| `python_bindings` | No | Enables PyO3 bindings and `rayon`. At least one engine is required. |
| `benchmarking` | No | Exposes documentation-hidden benchmark support and the native benchmark binary. |

A no-default-feature build has domain types but no functioning collision backend. Selecting a disabled `CollisionEngine` returns `CrccError::Unsupported`.

## Crate-Root Exports

### Geometry

```text
CollisionObject Compound
Circle Rectangle Triangle HalfSpace Empty FullSpace
geo::Polygon as Polygon
glamx::DPose2 as Pose
```

`Compound` is a type alias of `CollisionObject`, not a distinct representation.

### Scenes and queries

```text
CollisionChecker<E> SelectedCollisionChecker CollisionCheckerBuilder
DynamicObstacle PreparedStaticQuery PreparedDynamicQuery
CollisionEngine CollisionStatus CollisionResult
```

### Time and errors

```text
TimeStep CrccError CrccResult
```

Additional public module-level types include `SimpleCollisionObject`, polygon wrapper types, `GenericDynamicObstacle<E>`, `TimeStepSet`, `EngineCollisionObject`, and backend representations. Prefer root exports for application code.

## `CollisionObject`

Backend-independent union geometry.

### Pair queries

```rust
fn collides(
    &self,
    other: &Self,
    pos_self: Pose,
    pos_other: Pose,
    engine: CollisionEngine,
) -> CrccResult<bool>
```

```rust
fn collides_continuous(
    &self,
    start_pos_self: Pose,
    end_pos_self: Pose,
    other: &Self,
    start_pos_other: Pose,
    end_pos_other: Pose,
    engine: CollisionEngine,
) -> CrccResult<bool>
```

```rust
fn distance(
    &self,
    other: &Self,
    pos_self: Pose,
    pos_other: Pose,
    engine: CollisionEngine,
) -> CrccResult<f64>
```

Pair calls convert both operands to the selected backend per invocation.

### Constructors

| Constructor | Result and validation |
| --- | --- |
| `empty()` | Empty geometry; infallible. |
| `full_space()` | Entire plane; infallible. |
| `circle(center, radius)` | Finite center and finite positive radius. |
| `rectangle(rect, orientation)` | Finite, non-empty rectangle and finite orientation. |
| `triangle(triangle)` | Finite nonzero-area triangle. |
| `polygon(polygon)` | Finite, nondegenerate, topologically valid polygon. |
| `half_space(normal, offset)` | Finite nonzero normal and finite offset. |
| `half_space_from_points(p1, p2)` | Region right of a finite distinct directed pair. |
| `half_space_from_coeffs(a, b, c)` | Region `a*x + b*y <= c`. |

All but `empty` and `full_space` return `CrccResult<Self>`.

### Inspection and composition

```rust
fn collision_objects(&self) -> &[SimpleCollisionObject]
fn into_collision_objects(self) -> Vec<SimpleCollisionObject>
const fn is_empty(&self) -> bool
fn is_full_space(&self) -> bool
fn merge(self, other: Self) -> Self
fn merge_all(objects: impl IntoIterator<Item = Self>) -> Self
```

Empty children are removed and full space dominates during collection/merge.

### Swept bounds

```rust
fn swept_areas(&self, positions: &[Pose]) -> Vec<Self>
fn swept_area(&self, start_pos: Pose, end_pos: Pose) -> Option<Self>
```

One swept area is produced per adjacent pose pair. Fewer than two poses produce no interval. These objects are conservative bounds, not a contact-time result.

## Primitive and Polygon Types

`collision_object::simple` publicly exposes:

```text
SimpleCollisionObject
Circle Rectangle Triangle HalfSpace Empty FullSpace
ConvexPolygon NonConvexPolygon PolygonWithHoles
SweptArea
```

Direct low-level construction can bypass high-level invariants. Use `CollisionObject` constructors unless backend-level control is required.

## `DynamicObstacle`

```rust
fn new(
    shape: CollisionObject,
    positions: Vec<Pose>,
    time_offset: TimeStep,
) -> CrccResult<Self>
```

Fixed shape over consecutive poses. Empty poses are valid. Non-finite poses and unrepresentable final time are rejected.

```rust
fn time_variant(
    obstacles: Vec<CollisionObject>,
    positions: Vec<Pose>,
    time_offset: TimeStep,
) -> CrccResult<Self>
```

Shape and pose counts must match. Empty endpoint shapes suppress the adjacent interval.

```rust
fn convert_repr<E: From<CollisionObject>>(
    self,
) -> GenericDynamicObstacle<E>
```

Converts domain trajectory geometry to backend representation `E`.

## `TimeStep`

```rust
pub struct TimeStep(pub i32);
```

Constants:

```text
TimeStep::MIN TimeStep::MAX TimeStep::ZERO
```

Helpers:

| Method | Behavior |
| --- | --- |
| `pred()` | Previous step, saturating at `MIN`. |
| `succ()` | Next step, saturating at `MAX`. |
| `checked_succ()` | Next step or `None` at `MAX`. |
| `add_steps(usize)` | Forward offset, saturating at `MAX`. |
| `checked_add_steps(usize)` | Forward offset or `None` if unrepresentable. |
| `iter_range(range)` | Expand ordinary included/excluded/unbounded bounds. |

`TimeStepSet` is a public alias for `BTreeSet<TimeStep>`. Derived `Add`, `Sub`, and `Mul` operators do not have the named helpers' explicit checked/saturating contract.

## `CollisionCheckerBuilder`

```rust
const fn new() -> Self
fn with_static_obstacle(self, object: impl Into<CollisionObject>) -> Self
fn with_dynamic_obstacle(self, obstacle: DynamicObstacle) -> Self
fn with_road_boundary(self, lanelets: &[Polygon]) -> Self
fn build<E: EngineCollisionObject>(self) -> CollisionChecker<E>
fn build_with_engine(self, engine: CollisionEngine) -> Result<SelectedCollisionChecker, CrccError>
```

`build::<E>()` converts through `From<CollisionObject>` and is infallible at its signature. Some backend representations can defer conversion failure until query time. `build_with_engine` rejects disabled engines.

## `CollisionStatus` and `CollisionResult`

```rust
enum CollisionStatus {
    NoCollision,
    CollidesStatic,
    CollidesDynamic(TimeStep),
}
```

`CollisionStatus::collides(&self) -> bool` reports whether either collision variant is present.

```rust
type CollisionResult = CrccResult<CollisionStatus>;
```

For between-step collision, `CollidesDynamic(t)` attributes interval `t -> t+1` to its start.

## Generic `CollisionChecker<E>`

The generic checker accepts already-converted backend objects `E`.

| Method family | Purpose |
| --- | --- |
| `collides_static(&E)` | Identity pose across all active times. |
| `collides_dynamic(&GenericDynamicObstacle<E>)` | Dynamic query across all active times. |
| `collides_static_at(...)` | Static query at one step. |
| `collides_dynamic_at(...)` | Dynamic query at one step. |
| `collides_static_pos(...)` | Positioned static query across all active times. |
| `collides_static_pos_at(...)` | Positioned static query at one step. |
| `collides_static_range(...)` | Positioned static query over Rust bounds. |
| `collides_dynamic_range(...)` | Dynamic query over Rust bounds. |

Static scene geometry is checked before and independently of the dynamic time range.

## `SelectedCollisionChecker`

Runtime-selected checker accepting domain objects.

### Preparation

```rust
fn prepare_static(&self, query: &CollisionObject) -> Result<PreparedStaticQuery, CrccError>
fn prepare_dynamic(&self, query: &DynamicObstacle) -> Result<PreparedDynamicQuery, CrccError>
fn engine(&self) -> CollisionEngine
```

Prepared query types expose `engine()` and reject cross-engine execution.

### Direct queries

```text
collides_static
collides_dynamic
collides_static_at
collides_dynamic_at
collides_static_pos
collides_static_pos_at
collides_static_range
collides_dynamic_range
```

Range forms accept `impl RangeBounds<TimeStep>`. Inverted ranges are empty and do not panic.

### Prepared execution

```rust
fn collides_static_prepared(&self, query: &PreparedStaticQuery) -> CollisionResult
fn collides_static_prepared_range(
    &self,
    query: &PreparedStaticQuery,
    position: Pose,
    time_range: impl RangeBounds<TimeStep>,
) -> CollisionResult
fn collides_dynamic_prepared(&self, query: &PreparedDynamicQuery) -> CollisionResult
fn collides_dynamic_prepared_range(
    &self,
    query: &PreparedDynamicQuery,
    time_range: impl RangeBounds<TimeStep>,
) -> CollisionResult
fn collides_static_prepared_batch(
    &self,
    query: &PreparedStaticQuery,
    positions: &[Pose],
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
) -> Vec<CollisionResult>
fn collides_dynamic_prepared_batch(
    &self,
    queries: &[PreparedDynamicQuery],
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
) -> Vec<CollisionResult>
```

### Batches (`rayon`)

```rust
fn collides_static_batch(
    &self,
    queries: &[(CollisionObject, Pose)],
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
) -> Vec<CollisionResult>
```

```rust
fn collides_dynamic_batch(
    &self,
    queries: &[DynamicObstacle],
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
) -> Vec<CollisionResult>
```

Order is preserved. Automatic batches use estimated work, active worker count, and indexed grain sizing to choose between sequential execution and Rayon.

Mixed batches of raw and prepared queries share one workload estimate and one parallelism decision:

```rust
enum StaticBatchQuery<'a> {
    Raw(&'a CollisionObject),
    Prepared(&'a PreparedStaticQuery),
}

enum DynamicBatchQuery<'a> {
    Raw(&'a DynamicObstacle),
    Prepared(&'a PreparedDynamicQuery),
}

fn collides_static_heterogeneous_batch<'a, I>(
    &self,
    sources: I,
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
) -> Vec<CollisionResult>
where
    I: IntoIterator<Item = (StaticBatchQuery<'a>, Pose)>,

fn collides_dynamic_heterogeneous_batch<'a, I>(
    &self,
    sources: I,
    time_range: impl RangeBounds<TimeStep> + Clone + Sync,
) -> Vec<CollisionResult>
where
    I: IntoIterator<Item = DynamicBatchQuery<'a>>,
```

A prepared query built for a different backend makes every slot return `CrccError::Unsupported`.

## Engine-Level API

`collision_checker::engine` publicly exposes `EngineCollisionObject` and pair dispatch functions. Backend modules expose:

```text
ParryCollisionObject
RhusicsCoreCollisionObject
CollideCollisionObject
```

Use these with generic checkers. Runtime application code normally uses `CollisionEngine` and `SelectedCollisionChecker`.

Typed Rhusics and Collide representations do not implement backend-native `distance_at`; the runtime domain pair API uses the shared geometric distance fallback for those engines.

## `CrccError`

```rust
enum CrccError {
    InvalidRadius(f64),
    NotConvex,
    HasHoles,
    EmptyShape,
    InvalidGeometry(&'static str),
    Unsupported,
}
```

| Variant | Meaning |
| --- | --- |
| `InvalidRadius` | Circle center/radius validation failed. |
| `NotConvex` | A convex-only wrapper received non-convex geometry. |
| `HasHoles` | A wrapper forbidding interiors received holes. |
| `EmptyShape` | Geometry is degenerate or empty. |
| `InvalidGeometry` | Coordinates, topology, pose sequence, or time extent is invalid. |
| `Unsupported` | Engine, representation, or operation is unavailable. |

`Unsupported` is not a negative collision answer.

## Road-Boundary Semantics

`with_road_boundary` adds occupied geometry outside supplied `geo::Polygon` lanelets. It unions and simplifies drivable polygons, adds half-spaces outside the convex hull, and fills significant internal gaps. Empty lanelets yield full-space occupied boundary.

The Rust crate does not expose CommonRoad scenario types or XML parsing.

## Important Behavioral Notes

- Exact tangency differs by engine.
- Empty collides with nothing, including full space.
- Full space collides with every non-empty object.
- A zero distance does not imply every engine reports collision at exact contact.
- Static scene collision takes precedence over dynamic status.
- A singleton time range excludes outgoing motion.
- Continuous positives can be conservative.
