# CommonRoad Collision Checker

Rust/Python collision checking utilities for CommonRoad scenarios.

## Implemented Features

- Collision primitives: `Circle`, `Rectangle`, `Triangle`, `Polygon`, `Compound`, `Empty`, `FullSpace`, and `HalfSpace`.
- Pairwise collision checks between collision objects, including positioned discrete checks and continuous checks between two poses.
- Collision checker builder for static obstacles, dynamic obstacles, and road-boundary obstacles.
- Static and dynamic obstacle queries with optional `min_time` / `max_time` filtering.
- Dynamic obstacles with either one fixed shape moving through poses or time-varying per-step shapes.
- Collision status reporting through `NoCollision`, `CollidesStatic`, and `CollidesDynamic(t)`.
- Parallel batch queries for static and dynamic candidates.
- Collision engine selection between `Parry` and `Rhusics`.
- CommonRoad helpers for occupancy conversion, obstacle conversion, scenario collision checker construction, and road-boundary generation.
- Rust library API and Python bindings built with PyO3/maturin.

## Usage

Running the example script will automatically build and install the package:

```bash
uv run main.py
```

Run a specific example directly:

```bash
uv run main.py smoke --scenario scenarios/ZAM_Yield-1_1_T-1.xml --engine parry
uv run main.py benchmark
uv run main.py visualize
```

## Python Public API

The top-level `crcc` package re-exports the main public classes:

```python
from crcc import (
    Circle,
    CollisionCheckerBuilder,
    CollisionEngine,
    Compound,
    DynamicObstacle,
    Pose,
    Rectangle,
)
```

### Poses

```python
Pose(translation: tuple[float, float], angle: float)
Pose.identity() -> Pose
Pose.from_translation(translation: tuple[float, float]) -> Pose
Pose.from_rotation(angle: float) -> Pose
pose.translation -> tuple[float, float]
pose.rotation -> float
pose.and_then(other: Pose) -> Pose
```

### Collision Objects

All shapes inherit from `CollisionObject`:

```python
obj.collides(
    other: CollisionObject,
    pos_self: Pose = Pose.identity(),
    pos_other: Pose = Pose.identity(),
    engine: CollisionEngine = CollisionEngine.Parry,
) -> bool

obj.collides_continuous(
    start_pos_self: Pose,
    end_pos_self: Pose,
    other: CollisionObject,
    start_pos_other: Pose,
    end_pos_other: Pose,
    engine: CollisionEngine = CollisionEngine.Parry,
) -> bool

obj.merge(other: CollisionObject) -> CollisionObject
CollisionObject.merge_all(collision_objects: list[CollisionObject]) -> CollisionObject
```

Shape constructors:

```python
Circle(radius: float, center: tuple[float, float] = (0.0, 0.0))
Rectangle(length: float, width: float, orientation: float = 0.0, center: tuple[float, float] = (0.0, 0.0))
Triangle(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float])
Polygon(exterior: list[tuple[float, float]], interiors: list[list[tuple[float, float]]] | None = None)
Compound(collision_objects: list[CollisionObject])
HalfSpace(outward_normal: tuple[float, float], offset: float = 0.0)
HalfSpace.from_points(p1: tuple[float, float], p2: tuple[float, float])
HalfSpace.from_coeffs(a: float, b: float, c: float = 0.0)
Empty()
FullSpace()
```

### Dynamic Obstacles

```python
DynamicObstacle(shape: CollisionObject, positions: list[Pose], time_offset: int)
DynamicObstacle.from_time_variant(
    obstacles: list[CollisionObject],
    time_offset: int = 0,
    positions: list[Pose] | None = None,
) -> DynamicObstacle
```

### Collision Checkers

```python
CollisionCheckerBuilder(engine: CollisionEngine = CollisionEngine.Parry)
builder.with_engine(engine: CollisionEngine) -> CollisionCheckerBuilder
builder.with_static_obstacle(static_obstacle: CollisionObject) -> CollisionCheckerBuilder
builder.with_dynamic_obstacle(dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder
builder.with_road_boundary_obstacle(lanelets: list[list[tuple[float, float]]]) -> CollisionCheckerBuilder
builder.build() -> CollisionChecker
```

```python
checker.collides_static(
    static_obstacle: CollisionObject,
    position: Pose | None = None,
    min_time: int | None = None,
    max_time: int | None = None,
) -> CollisionStatus

checker.collides_dynamic(
    dynamic_obstacle: DynamicObstacle,
    min_time: int | None = None,
    max_time: int | None = None,
) -> CollisionStatus

checker.par_collides_static(
    positioned_static_obstacles: list[tuple[CollisionObject, Pose]],
    min_time: int | None = None,
    max_time: int | None = None,
) -> list[CollisionStatus]

checker.par_collides_dynamic(
    dynamic_obstacles: list[DynamicObstacle],
    min_time: int | None = None,
    max_time: int | None = None,
) -> list[CollisionStatus]
```

`CollisionStatus` variants are `NoCollision`, `CollidesStatic`, and `CollidesDynamic(t)`.

```python
status.collides -> bool
status.time_step -> int | None
str(status) -> str
```

Available engines:

```python
CollisionEngine.Parry
CollisionEngine.Rhusics
```

### CommonRoad Helpers

```python
create_collision_checker_from_scenario(scenario, builder: CollisionCheckerBuilder | None = None) -> CollisionCheckerBuilder
add_commonroad_static_obstacle_to_builder(builder, static_obstacle) -> CollisionCheckerBuilder
add_commonroad_dynamic_obstacle_to_builder(builder, dynamic_obstacle) -> CollisionCheckerBuilder
add_road_boundary_to_builder(builder, lanelet_network) -> CollisionCheckerBuilder
create_road_boundary_obstacle(lanelet_network) -> CollisionObject
commonroad_shape_to_collision_object(shape) -> CollisionObject
commonroad_occupancy_to_collision_object(occupancy) -> CollisionObject
commonroad_polygon_to_collision_object(polygon) -> CollisionObject
shapely_geometry_to_collision_object(geometry) -> CollisionObject
commonroad_state_to_pose(state) -> Pose
```

## Rust Public API

The Rust crate exposes these modules:

- `collision_object`: composite `CollisionObject`, primitive constructors, merge helpers, and swept-area helpers.
- `collision_object::simple`: lower-level primitive shape types and shape-specific validation.
- `collision_checker`: `CollisionCheckerBuilder`, `CollisionChecker`, `CollisionStatus`, selected-engine wrappers, and parallel query support.
- `collision_checker::engine`: `CollisionEngine`, `EngineCollisionObject`, and direct discrete/continuous engine dispatch helpers.
- `dynamic_obstacle`: fixed-shape and time-varying `DynamicObstacle` construction.
- `time`: `TimeStep` and `TimeStepSet`.
- `error`: `CrccError` and `CrccResult`.

Feature flags:

- `parry`: enable the Parry collision engine.
- `rhusics`: enable the Rhusics collision engine.
- `python_bindings`: enable PyO3 bindings and parallel Python batch queries.
- `rayon`: enable Rayon-backed parallel query APIs.

## Development

Run the Rust checks:

```bash
cargo check --all-features
cargo test --all-features
cargo clippy --all-features --all-targets -- -D warnings
```

Run the Python checks:

```bash
uv run ruff check .
uv run pytest
```

## Planned Work

- Plotting of collision objects etc.
- Python: Separation of generic collision checking and CommonRoad-specific parts
