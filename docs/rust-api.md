# Rust API Reference

CRCC groups its API by module: `collision_object`, `collision_checker`, `dynamic_obstacle`, `time`, and `error`. Angles are in radians.

<!-- ponytail: simplified and table-structured Rust API guide -->

## Cargo Features

| Feature | Description |
| :--- | :--- |
| `parry`, `rhusics`, `collide` | Compiles corresponding backend engines (enabled by default). |
| `rayon` | Exposes parallel batch traits and runtime-selected query methods. |
| `python_bindings` | Builds the PyO3 Python extension (implies `rayon`). |

```toml
# Example Cargo.toml
[dependencies]
crcc = { path = "../crcc", default-features = false, features = ["parry", "rayon"] }
geo = "0.32"
glamx = "0.1"
```

---

## Collision Objects

`crcc::collision_object::CollisionObject` represents primitive or compound geometry.

```rust
// Constructors
CollisionObject::empty() -> Self
CollisionObject::full_space() -> Self
CollisionObject::circle(center: impl Into<Coordinate>, radius: f64) -> CrccResult<Self>
CollisionObject::rectangle(rect: geo::Rect, orientation: f64) -> CrccResult<Self>
CollisionObject::triangle(triangle: geo::Triangle) -> CrccResult<Self>
CollisionObject::polygon(polygon: geo::Polygon) -> CrccResult<Self>
CollisionObject::half_space(normal: [f64; 2], offset: f64) -> CrccResult<Self>
CollisionObject::half_space_from_points(p1: [f64; 2], p2: [f64; 2]) -> CrccResult<Self>
CollisionObject::half_space_from_coeffs(a: f64, b: f64, c: f64) -> CrccResult<Self>

// Composition & Inspection
CollisionObject::merge(self, other: Self) -> Self
CollisionObject::merge_all(objects: impl IntoIterator<Item = Self>) -> Self
CollisionObject::is_empty(&self) -> bool
CollisionObject::is_full_space(&self) -> bool
CollisionObject::swept_areas(&self, positions: &[Pose]) -> Vec<Self>
CollisionObject::swept_area(&self, start: Pose, end: Pose) -> Self
```

Discrete queries, distance, and CCD are dispatched using free functions in `crcc::collision_checker::engine::{collides, distance, collides_continuous}`.

---

## Engines

`crcc::collision_checker::engine::CollisionEngine` has variants `Parry`, `Rhusics`, and `Collide`.
Backend engine types implement `EngineCollisionObject`:
- `engine::parry::ParryCollisionObject`
- `engine::rhusics::RhusicsCoreCollisionObject`
- `engine::collide::CollideCollisionObject`

---

## Collision Checkers

### Generic Checker (`CollisionChecker<E>`)
Statically bound to one engine representation at compile time.

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

### Runtime-Selected Checker (`SelectedCollisionChecker`)
Supports selecting engines dynamically at runtime.

| Method | Signature |
| :--- | :--- |
| `engine` | `&self -> CollisionEngine` |
| `collides_static` / `_at` / `_pos` / `_pos_at` | `&self, ... -> CollisionResult` |
| `collides_dynamic` / `_at` | `&self, ... -> CollisionResult` |
| `collides_static_range` | `&self, &CollisionObject, Pose, impl RangeBounds<TimeStep> -> CollisionResult` |
| `collides_dynamic_range` | `&self, &DynamicObstacle, impl RangeBounds<TimeStep> -> CollisionResult` |
| `par_static` / `par_dynamic` | `&self, ... -> Vec<CollisionResult>` (requires `rayon`) |

### Builder (`CollisionCheckerBuilder`)
Methods mutate and return the builder:
- `new() -> Self`
- `with_static_obstacle(CollisionObject) -> Self`
- `with_dynamic_obstacle(DynamicObstacle) -> Self`
- `with_road_boundary(Vec<geo::Polygon>) -> Self`
- `build::<E>() -> CollisionChecker<E>`
- `build_with_engine(CollisionEngine) -> CrccResult<SelectedCollisionChecker>`

---

## Dynamic Obstacles & Time

- `DynamicObstacle::new(shape, positions, time_offset: TimeStep) -> Self`
- `DynamicObstacle::time_variant(shapes, positions, time_offset: TimeStep) -> Self`
- `TimeStep(pub i32)` wraps the integer time steps.
  - Constants: `TimeStep::MIN`, `TimeStep::MAX`, `TimeStep::ZERO`.
  - Methods: `pred()`, `succ()`, `add_steps(n)`, `iter_range(start, end)`.

---

## Status and Errors

- `CollisionStatus`: `NoCollision`, `CollidesStatic`, or `CollidesDynamic(TimeStep)`.
- `CollisionResult`: `Result<CollisionStatus, CrccError>`.
- `CrccError` variants: `InvalidRadius`, `NotConvex`, `HasHoles`, `EmptyShape`, `InvalidGeometry`, `Unsupported(String)`.
