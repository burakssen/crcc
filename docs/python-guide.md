# Python Guide

This guide covers the usual CRCC workflow in Python: create geometry, choose a collision engine, build a scene, and query it.

For signatures and the full list of classes, see the [Python API reference](python-api.md).

## Setup

From the repository root:

```bash
uv sync
uv run python
```

All core classes are available from `crcc`:

```python
from crcc import Circle, CollisionCheckerBuilder, Pose, Rectangle
```

Angles are in radians. Shapes use local coordinates and `Pose` places them in the scene.

## Pair queries

Use `collides` and `distance` when checking two objects directly:

```python
from crcc import Circle, Pose

left = Circle(1.0)
right = Circle(1.0)
right_pose = Pose.from_translation((3.0, 0.0))

assert not left.collides(right, pos_other=right_pose)
assert left.distance(right, pos_other=right_pose) == 1.0
```

The default engine is Parry. Pass `engine=CollisionEngine.Rhusics` or `CollisionEngine.Collide` to select another backend for a pair query.

## Continuous collision detection

Endpoint checks can miss a collision between poses. `collides_continuous` checks the complete motion interval:

```python
from crcc import Circle, Pose, Rectangle

moving = Circle(0.5)
barrier = Rectangle(0.25, 3.0)

hit = moving.collides_continuous(
    Pose.from_translation((-2.0, 0.0)),
    Pose.from_translation((2.0, 0.0)),
    barrier,
    Pose.identity(),
    Pose.identity(),
)
assert hit
```

A `False` result certifies separation. A `True` result means the interval may contain a collision; rotational sweeps can be conservative.

## Scene queries

A `CollisionChecker` stores static and dynamic obstacles. Build one with `CollisionCheckerBuilder`:

```python
from crcc import Circle, CollisionCheckerBuilder, CollisionEngine, Pose, Rectangle

checker = (
    CollisionCheckerBuilder(CollisionEngine.Rhusics)
    .with_static_obstacle(Rectangle(2.0, 2.0))
    .build()
)

status = checker.collides_static(Circle(0.5), Pose.identity())
assert status.collides
assert checker.engine == CollisionEngine.Rhusics
```

`CollisionStatus.collides` reports whether anything was hit. `time_step` identifies the first dynamic collision and is otherwise `None`.

## Dynamic obstacles and time windows

`DynamicObstacle` associates geometry with poses at discrete time steps. Motion between consecutive poses is checked continuously.

```python
from crcc import Circle, CollisionCheckerBuilder, DynamicObstacle, Pose, Rectangle

moving = DynamicObstacle(
    Circle(0.5),
    [Pose.from_translation((-2.0, 0.0)), Pose.from_translation((2.0, 0.0))],
    time_offset=10,
)
checker = CollisionCheckerBuilder().with_static_obstacle(Rectangle(0.25, 3.0)).build()

status = checker.collides_dynamic(moving, min_time=10, max_time=11)
assert status.collides
```

Python time bounds are inclusive. Omit both bounds to query the complete trajectory. Use `DynamicObstacle.from_time_variant` when the geometry also changes between steps.

## Batch queries

Batch methods preserve input order. Small batches run sequentially; larger batches use Rayon automatically.

```python
from crcc import Circle, CollisionCheckerBuilder, Pose, Rectangle

checker = CollisionCheckerBuilder().with_static_obstacle(Rectangle(2.0, 2.0)).build()
queries = [
    (Circle(0.25), Pose.identity()),
    (Circle(0.25), Pose.from_translation((5.0, 0.0))),
]

results = checker.collides_static_batch(queries)
assert [result.collides for result in results] == [True, False]
```

Use `collides_dynamic_batch` for multiple trajectories. Both batch methods accept the same inclusive time bounds as their single-query equivalents.

## CommonRoad scenarios

`crcc.commonroad.scenario_builder` adds the road boundary and all supported scenario obstacles to a builder:

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

Lower-level conversion helpers are listed in the [CommonRoad API section](python-api.md#commonroad-conversion).

## Interactive exploration

Run the playground against the bundled scenario:

```bash
uv run main.py playground
```

Object fills show collisions at the visible pose. Paths and outlines show conservative collision results for the next interval.

## Accuracy notes

- Translation is interpolated linearly.
- Rotation follows the shortest angular path.
- Translation-only convex sweeps are exact; more complex rotational sweeps may be over-approximated.
- Dividing a trajectory into smaller intervals can tighten conservative rotational bounds.
