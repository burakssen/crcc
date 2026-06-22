# CommonRoad Collision Checker (`crcc`)

`crcc` is a high-performance, hybrid Rust/Python collision checking library tailored for **CommonRoad** autonomous driving scenarios. It provides fast, exact, and continuous collision checking between various geometric shapes, static obstacles, time-varying dynamic trajectories, and complex road networks.

By wrapping a core Rust library in Python bindings via PyO3 and Maturin, `crcc` delivers the developer experience of Python with the execution speed and concurrency of Rust (backed by Rayon).

---

## Key Features

- **Rich Collision Primitives**: Fully supports discrete and continuous collision checking for:
  * `Circle`
  * `Rectangle` (oriented bounding boxes)
  * `Triangle`
  * `Polygon` (convex, non-convex, and polygons with holes)
  * `Compound` (composition of multiple shapes)
  * `Empty` / `FullSpace` / `HalfSpace` (defined via coefficients, normals, or line points)
- **Continuous Collision Detection (CCD)**: Computes continuous overlaps (rigid shape casting) between moving shapes to prevent "tunneling" (missed collisions at high speeds). Fully supports both `FixedShape` and `VaryingShape` (time-variant) dynamic obstacles.
- **Batch & Parallel Queries**: Rayon-powered batch collision checks (`par_collides_static`, `par_collides_dynamic`) for high-throughput scenario simulation.
- **Pluggable Collision Engines**: Switch seamlessly between collision backends:
  * **Parry** (default, rigid-body physics oriented)
  * **Rhusics** (broad-phase sweep and prune)
- **CommonRoad scenario integration**: Standard helpers to load obstacles, build lanelet boundary shapes, and convert occupancies directly from CommonRoad XML scenario files.

---

## Installation & Quick Start

The project uses `uv` for python package and environment management.

To build, install, and launch the interactive playground example:
```bash
uv run main.py
```

Run specific examples directly:
```bash
# Smoke test on a yield scenario using the Parry engine
uv run main.py smoke --scenario scenarios/ZAM_Yield-1_1_T-1.xml --engine parry

# Run parallel check benchmarks comparing sequential vs Rayon execution
uv run main.py benchmark

# Run visual scenario animations showing cumulative ego-vehicle collisions
uv run main.py visualize

# Run the interactive ego vehicle playground
uv run main.py interactive
```

---

## Interactive Playground

The `interactive` example opens a Matplotlib-based GUI that queries the Rust collision checker in real-time as you move the ego vehicle:
- **Move Mouse**: Translates the ego vehicle across the scenario.
- **Scroll Mouse Wheel**: Rotates the ego vehicle by $5^\circ$ per scroll tick.
- **Slider**: Selects the active time step $t$ (re-rendering other dynamic obstacles at that step).
- **Feedback**: The vehicle is colored **GREEN** when the pose is clear, and **RED** when a collision is detected.

---

## Python Public API Reference

The top-level `crcc` package exports the main public classes:
```python
from crcc import (
    Pose,
    CollisionObject,
    Circle,
    Rectangle,
    Triangle,
    Polygon,
    Compound,
    HalfSpace,
    Empty,
    FullSpace,
    DynamicObstacle,
    CollisionCheckerBuilder,
    CollisionChecker,
    CollisionEngine,
    CollisionStatus,
)
```

### 1. Poses (`Pose`)
Poses represent 2D transformations (translation and rotation).
```python
# Constructor
pose = Pose(translation=(1.0, 2.0), angle=0.5)

# Static methods
Pose.identity() -> Pose
Pose.from_translation(translation: tuple[float, float]) -> Pose
Pose.from_rotation(angle: float) -> Pose

# Properties
pose.translation -> tuple[float, float]
pose.rotation -> float # Rotation angle in radians

# Composition
new_pose = pose.compose(other: Pose)
new_pose = pose * other  # Pose multiplication operator
```

### 2. Collision Objects (`CollisionObject`)
All shape primitives inherit from `CollisionObject`. They support discrete and continuous queries:
```python
# Discrete pairwise collision check
obj.collides(
    other: CollisionObject,
    pos_self: Pose = Pose.identity(),
    pos_other: Pose = Pose.identity(),
    engine: CollisionEngine = CollisionEngine.Parry,
) -> bool

# Continuous pairwise collision check (rigid shape casting)
obj.collides_continuous(
    start_pos_self: Pose,
    end_pos_self: Pose,
    other: CollisionObject,
    start_pos_other: Pose,
    end_pos_other: Pose,
    engine: CollisionEngine = CollisionEngine.Parry,
) -> bool

# Merge multiple shapes into one CollisionObject
merged = obj.merge(other: CollisionObject)
merged = CollisionObject.merge_all(shapes: list[CollisionObject])
```

#### Shape Constructors:
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

### 3. Dynamic Obstacles (`DynamicObstacle`)
Represents an obstacle moving along a trajectory over time.
```python
# Fixed shape moving through a list of poses
dynamic_obstacle = DynamicObstacle(shape: CollisionObject, positions: list[Pose], time_offset: int)

# Time-varying (varying shape/size) obstacle
dynamic_obstacle = DynamicObstacle.from_time_variant(
    obstacles: list[CollisionObject], # A shape per time step
    time_offset: int = 0,
    positions: list[Pose] | None = None, # Optional translations per step
)
```

### 4. Collision Checkers & Builders (`CollisionChecker`)
Builds and executes queries against scenario environments.
```python
# 1. Initialize Builder
builder = CollisionCheckerBuilder(engine: CollisionEngine = CollisionEngine.Parry)
builder.with_engine(engine: CollisionEngine) -> CollisionCheckerBuilder

# 2. Populate environment obstacles
builder.with_static_obstacle(query_shape: CollisionObject) -> CollisionCheckerBuilder
builder.with_dynamic_obstacle(dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder
builder.with_road_boundary_obstacle(lanelets: list[list[tuple[float, float]]]) -> CollisionCheckerBuilder

# 3. Build checker
checker = builder.build() -> CollisionChecker
```

#### Executing Queries:
```python
# Check stationary ego vehicle (query_shape) against the environment
status = checker.collides_static(
    query_shape: CollisionObject,
    position: Pose | None = None,
    min_time: int | None = None,  # Optional half-bounded range filtering
    max_time: int | None = None,
) -> CollisionStatus

# Check dynamic ego vehicle (dynamic_obstacle) trajectory against the environment
status = checker.collides_dynamic(
    dynamic_obstacle: DynamicObstacle,
    min_time: int | None = None,
    max_time: int | None = None,
) -> CollisionStatus

# Parallel Rayon-backed static batch check
statuses = checker.par_collides_static(
    positioned_query_shapes: list[tuple[CollisionObject, Pose]],
    min_time: int | None = None,
    max_time: int | None = None,
) -> list[CollisionStatus]

# Parallel Rayon-backed dynamic batch check
statuses = checker.par_collides_dynamic(
    dynamic_obstacles: list[DynamicObstacle],
    min_time: int | None = None,
    max_time: int | None = None,
) -> list[CollisionStatus]
```

#### `CollisionStatus` properties:
- `status.collides` -> `bool` (indicates if a collision occurred)
- `status.time_step` -> `int | None` (returns the first timestep where collision was detected)
- `str(status)` -> e.g. `"NoCollision"`, `"CollidesStatic"`, `"CollidesDynamic(15)"`

---

## Rust Public API

The Rust crate consists of the following modules:
* `collision_object`: Composite `CollisionObject`, primitives, merges, and swept areas.
* `collision_checker`: `CollisionChecker`, `CollisionCheckerBuilder`, `CollisionStatus`, selected-engine wrappers, and batch Rayon queries.
* `dynamic_obstacle`: Trajectory types for `FixedShape` and `VaryingShape`.
* `time`: `TimeStep` and `TimeStepSet` implementations.
* `error`: `CrccError` and results.

### Feature Flags
* `parry`: Compile the Parry collision detection backend.
* `rhusics`: Compile the Rhusics collision backend.
* `python_bindings`: Compile the PyO3 modules.
* `rayon`: Enable Rayon multi-threaded parallel queries.

---

## Development & Testing

Run Rust formatting and tests:
```bash
cargo check --all-features
cargo test --all-features
cargo clippy --all-features --all-targets -- -D warnings
```

Run Python linting and tests:
```bash
uv run ruff check .
uv run pytest
```
