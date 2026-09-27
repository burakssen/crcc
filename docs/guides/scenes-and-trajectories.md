# Scenes and trajectories

Build a checker for repeated queries against the same static scene or trajectories. Construction merges static geometry, converts scene objects to the selected backend, and indexes dynamic active times. The resulting checker is immutable.

## Build a static scene

```python
from crcc import Circle, CollisionBackend, CollisionCheckerBuilder, Pose, Rectangle

checker = (
    CollisionCheckerBuilder(backend=CollisionBackend.Parry)
    .add_static_obstacle(Rectangle(2.0, 2.0))
    .add_static_obstacle(Circle(0.25, center=(3.0, 0.0)))
    .build()
)

hit = checker.collides_static(Circle(0.5), position=Pose.identity())
clear = checker.collides_static(Circle(0.5), position=Pose.from_translation((8.0, 0.0)))
assert hit.collides and hit.time_step is None
assert not clear.collides
```

Static scene geometry is always checked, even when a dynamic time window is supplied. A static hit returns `CollidesStatic` before any dynamic result.

## Add a fixed-shape trajectory

```python
from crcc import Circle, CollisionCheckerBuilder, DynamicObstacle, Pose, Rectangle

trajectory = DynamicObstacle(
    Circle(0.5),
    [Pose.from_translation((-2.0, 0.0)), Pose.from_translation((2.0, 0.0))],
    time_offset=10,
)
checker = CollisionCheckerBuilder().add_static_obstacle(Rectangle(0.25, 3.0)).build()

status = checker.collides_dynamic(trajectory, min_time=10, max_time=11)
assert status.collides and status.time_step == 10
```

Both samples are selected, so the query checks occupancy at steps 10 and 11 and the interval from 10 to 11. A singleton window at 10 checks occupancy at 10 only.

## Represent changing or missing occupancy

Use a time-varying trajectory when its geometry changes by sample:

```python
from crcc import Circle, DynamicObstacle, Empty, Pose

trajectory = DynamicObstacle.from_time_variant(
    obstacles=[Circle(0.5), Empty(), Circle(0.5)],
    positions=[
        Pose.from_translation((-2.0, 0.0)),
        Pose.identity(),
        Pose.from_translation((2.0, 0.0)),
    ],
    time_offset=0,
)
```

Each shape and pose entry describes one sample. An interval touching an empty shape has no occupancy; CRCC does not interpolate motion through that gap. For the time and status model, see [scenes and time](../concepts/scenes-and-time.md). For signatures, see the [Python reference](../reference/python.md#dynamicobstacle).
