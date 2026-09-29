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

The interval from 10 to 11 is checked before occupancy at 10. A singleton **dynamic-query** window at 10 also includes that outgoing interval; a singleton **static-query** window checks occupancy only:

```python
from crcc import Circle, CollisionCheckerBuilder, DynamicObstacle, Pose, Rectangle

moving = Circle(0.5)
wall = Rectangle(0.25, 3.0)
start, end = Pose.from_translation((-2.0, 0.0)), Pose.from_translation((2.0, 0.0))
trajectory = DynamicObstacle(moving, [start, end], time_offset=10)
static_scene = CollisionCheckerBuilder().add_static_obstacle(wall).build()
dynamic_scene = CollisionCheckerBuilder().add_dynamic_obstacle(trajectory).build()

assert not moving.collides(wall, pos_self=start)  # Discrete endpoint check.
assert static_scene.collides_dynamic(trajectory, min_time=10, max_time=10).collides
assert not dynamic_scene.collides_static(wall, min_time=10, max_time=10).collides
assert dynamic_scene.collides_static(wall, min_time=10, max_time=11).collides
assert not static_scene.collides_dynamic(trajectory, min_time=12).collides
```

The final assertion shows that there is no occupancy after the trajectory ends. Query results contain neither the obstacle identity nor an exact contact time.

## Represent changing or missing occupancy

Use a time-varying trajectory when its geometry changes by sample:

```python
from crcc import Circle, CollisionCheckerBuilder, DynamicObstacle, Empty, Pose

trajectory = DynamicObstacle.from_time_variant(
    obstacles=[Circle(0.5), Empty(), Circle(0.5)],
    positions=[
        Pose.from_translation((-2.0, 0.0)),
        Pose.identity(),
        Pose.from_translation((2.0, 0.0)),
    ],
    time_offset=0,
)
assert not CollisionCheckerBuilder().add_static_obstacle(Circle(0.25)).build().collides_dynamic(trajectory).collides
```

Each shape and pose entry describes one sample. An interval touching an empty shape has no occupancy; CRCC does not interpolate motion through that gap. For the time and status model, see [scenes and time](../concepts/scenes-and-time.md). For signatures, see the [Python reference](../reference/python.md#dynamicobstacle).

## Rust trajectory query

```rust
use crcc::{CollisionCheckerBuilder, CollisionEngine, CollisionObject, DynamicObstacle, Pose, TimeStep};

fn main() -> Result<(), crcc::CrccError> {
    let trajectory = DynamicObstacle::new(
        CollisionObject::circle((0.0, 0.0), 0.5)?,
        vec![Pose::translation(-2.0, 0.0), Pose::translation(2.0, 0.0)],
        TimeStep(10),
    )?;
    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(CollisionObject::rectangle(
            geo::Rect::new((-0.125, -1.5), (0.125, 1.5)), 0.0,
        )?)
        .build_with_engine(CollisionEngine::Parry)?;
    let status = checker.collides_dynamic_range(&trajectory, TimeStep(10)..=TimeStep(10))?;
    assert_eq!(status, crcc::CollisionStatus::CollidesDynamic(TimeStep(10)));
    Ok(())
}
```
