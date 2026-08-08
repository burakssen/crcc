# Python Usage Guide

This guide follows the usual Python workflow: install CRCC, construct geometry, choose an engine, query pairs, build a scene, add trajectories, reuse prepared queries, batch work, and convert CommonRoad scenarios.

See [Concepts and engines](concepts.md) for the shared semantic contract and [Python API reference](python-api.md) for signatures.

## Install

CRCC is not currently published to PyPI. You can install it directly from GitHub as a Git dependency:

```bash
uv pip install git+https://github.com/burakssen/crcc
# or with pip
pip install git+https://github.com/burakssen/crcc
# or add to a uv project
uv add git+https://github.com/burakssen/crcc
```

Or declare it in `pyproject.toml`:

```toml
[project]
dependencies = [
    "crcc @ git+https://github.com/burakssen/crcc",
]
```

Installing as a Git dependency builds the native extension during installation and requires a Rust toolchain.

### Source Checkout

For development or running tutorials:

```bash
git clone https://github.com/burakssen/crcc.git
cd crcc
git lfs install
git lfs pull
uv sync --frozen
```

This installs project dependencies and builds the native `crcc._core` extension through Maturin. Verify the import:

```bash
uv run python -c "import crcc; print(crcc.CollisionEngine.Parry)"
```

### Pre-built Release Wheel

To install a compatible wheel downloaded from a GitHub release:

```bash
uv venv
uv pip install ./crcc-0.1.0-cp310-abi3-<platform>.whl
```

The ABI3 wheels target CPython 3.10 and newer. Match the wheel's operating system and architecture to the interpreter.

## Coordinate Conventions

- Coordinates and distances are `float` values.
- Angles are counter-clockwise radians.
- Shape centers and vertices are local coordinates.
- A `Pose` places local geometry in world coordinates.

```python
from math import pi

from crcc import Pose

world_pose = Pose((3.0, 4.0), pi / 2)
assert world_pose.translation == (3.0, 4.0)
assert world_pose.rotation == pi / 2
```

Compose poses with `compose` or `*`. The right-hand pose is applied first.

## Construct Geometry

All concrete shapes inherit from `CollisionObject`:

```python
from crcc import Circle, Compound, HalfSpace, Polygon, Rectangle, Triangle

circle = Circle(radius=0.5, center=(0.25, 0.0))
rectangle = Rectangle(length=2.0, width=1.0, orientation=0.2)
triangle = Triangle(point_a=(0.0, 0.0), point_b=(1.0, 0.0), point_c=(0.0, 1.0))
polygon = Polygon(
    exterior=[(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)],
    interiors=[[(-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5), (0.5, -0.5)]],
)
ground = HalfSpace.from_coeffs(0.0, 1.0, 0.0)  # y <= 0
compound = Compound([circle, rectangle, triangle])
```

Constructors reject non-finite, degenerate, or invalid geometry with `ValueError`. A circle radius and rectangle dimensions must be strictly positive. A polygon may be non-convex and may contain holes, but its rings must describe valid finite geometry.

`Empty()` never collides. `FullSpace()` collides with every non-empty object. `Compound([])` is empty.

## Query Two Objects

Use pair methods when no reusable scene is needed:

```python
from crcc import Circle, CollisionEngine, Pose

left = Circle(1.0)
right = Circle(1.0)
right_pose = Pose.from_translation((3.0, 0.0))

assert not left.collides(
    right,
    pos_other=right_pose,
    engine=CollisionEngine.Parry,
)
assert left.distance(right, pos_other=right_pose) == 1.0
```

Pair calls convert both objects to the selected backend each time. For repeated queries against a scene, build a checker instead.

## Check Continuous Motion

Discrete endpoint checks can miss tunneling. Supply start and end poses for both objects:

```python
from crcc import Circle, Pose, Rectangle

moving = Circle(0.5)
barrier = Rectangle(length=0.25, width=3.0)

possible_hit = moving.collides_continuous(
    Pose.from_translation((-2.0, 0.0)),
    Pose.from_translation((2.0, 0.0)),
    barrier,
    Pose.identity(),
    Pose.identity(),
)
assert possible_hit
```

Interpret the result conservatively: `False` certifies the complete interval is clear; `True` may be a conservative positive. Engine behavior for rotation, half-spaces, and exact tangency is summarized in [Engine selection](concepts.md#engine-selection).

## Build an Immutable Scene

The builder accepts static objects, dynamic trajectories, and road boundaries:

```python
from crcc import Circle, CollisionCheckerBuilder, CollisionEngine, Pose, Rectangle

checker = (
    CollisionCheckerBuilder(CollisionEngine.Rhusics)
    .with_static_obstacle(Rectangle(2.0, 2.0))
    .with_static_obstacle(Circle(0.25, center=(3.0, 0.0)))
    .build()
)

status = checker.collides_static(Circle(0.5), position=Pose.identity())
assert status.collides
assert status.time_step is None
assert checker.engine == CollisionEngine.Rhusics
```

Static scene geometry is checked before dynamic geometry. If it collides, the result is `CollidesStatic` regardless of time bounds.

## Add a Fixed-Shape Trajectory

`positions[0]` occurs at `time_offset`; each later pose advances one step:

```python
from crcc import Circle, CollisionCheckerBuilder, DynamicObstacle, Pose, Rectangle

trajectory = DynamicObstacle(
    Circle(0.5),
    [
        Pose.from_translation((-2.0, 0.0)),
        Pose.from_translation((2.0, 0.0)),
    ],
    time_offset=10,
)

checker = (
    CollisionCheckerBuilder()
    .with_static_obstacle(Rectangle(0.25, 3.0))
    .build()
)

status = checker.collides_dynamic(trajectory, min_time=10, max_time=11)
assert status.collides
assert status.time_step == 10
```

Bounds are inclusive. Both 10 and 11 are selected, so CRCC checks motion from 10 to 11. A range containing only 10 checks the pose at 10 but not that outgoing interval.

Empty pose sequences are valid and have no active times. Time values must fit signed 32-bit integers; Python conversion may raise `OverflowError` for out-of-range values.

## Use Time-Varying Geometry

Use `from_time_variant` when occupancy changes by step:

```python
from crcc import Circle, DynamicObstacle, Empty, Pose

occupancy = DynamicObstacle.from_time_variant(
    obstacles=[Circle(0.5), Empty(), Circle(0.5)],
    positions=[
        Pose.from_translation((-2.0, 0.0)),
        Pose.identity(),
        Pose.from_translation((2.0, 0.0)),
    ],
    time_offset=0,
)
```

Intervals touching the empty middle sample are empty. CRCC does not infer motion through a missing occupancy. If `positions` is omitted, identity poses are created for all shapes.

## Reuse Prepared Queries

Prepared queries avoid backend conversion when the same geometry or trajectory is reused:

```python
from crcc import Circle, CollisionCheckerBuilder, Pose

checker = CollisionCheckerBuilder().with_static_obstacle(Circle(1.0)).build()
query = Circle(0.25)
prepared = checker.prepare_static(query)

near = checker.collides_static_prepared(prepared, Pose.identity())
far = checker.collides_static_prepared(
    prepared,
    Pose.from_translation((5.0, 0.0)),
)
assert near.collides and not far.collides
assert prepared.engine == checker.engine
```

A prepared query belongs to one engine. Passing it to a checker built with another engine raises `ValueError` as an unsupported operation.

## Run Ordered Batches

Batch results preserve input order:

```python
from crcc import Circle, CollisionCheckerBuilder, Pose, Rectangle

checker = CollisionCheckerBuilder().with_static_obstacle(Rectangle(2.0, 2.0)).build()
results = checker.collides_static_batch(
    [
        (Circle(0.25), Pose.identity()),
        (Circle(0.25), Pose.from_translation((5.0, 0.0))),
    ]
)
assert [result.collides for result in results] == [True, False]
```

Use `collides_dynamic_batch` for trajectories. Inputs below 32 run sequentially; larger batches use Rayon and release the GIL during native work. Legacy `par_static` and `par_dynamic` are aliases of this automatic behavior.

## Convert a CommonRoad Scenario

CommonRoad support is in the Python-only `crcc.commonroad` module. Scenario XML files in this repository require Git LFS.

```python
from commonroad.common.file_reader import CommonRoadFileReader
from crcc import CollisionCheckerBuilder, CollisionEngine
from crcc.commonroad import scenario_builder

scenario, _ = CommonRoadFileReader(
    "scenarios/DEU_MerzenichRather-2_870_T-149.xml"
).open()

checker = scenario_builder(
    scenario,
    CollisionCheckerBuilder(CollisionEngine.Parry),
).build()
```

`scenario_builder` adds the road boundary, static obstacles, and dynamic obstacles. It returns a builder so callers can add project-specific geometry before `build()`.

Missing intermediate CommonRoad occupancies become empty geometry. The adapter suppresses motion across those gaps. An empty lanelet network adds no road constraint; directly calling the low-level boundary function on an empty network returns full space.

## Handle Errors

Native CRCC errors appear as `ValueError`:

```python
from crcc import Circle, Empty

try:
    Circle(0.0)
except ValueError as error:
    print(error)

try:
    Empty().distance(Circle(1.0))
except ValueError:
    # Distance to an empty set is unsupported by this API.
    pass
```

Python argument conversion can also raise `TypeError` or `OverflowError`. Treat unsupported queries as errors, never as collision-free results.

## Explore the Repository Tutorials

```bash
uv run main.py basic --engine parry
uv run main.py continuous --engine collide
uv run main.py commonroad --engine rhusics
uv run main.py playground
```

`main.py` is not installed by the wheel. See [Development and benchmarks](development.md) for the launcher, playground requirements, and benchmark workflows.
