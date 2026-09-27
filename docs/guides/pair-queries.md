# Pair queries

Use a pair query when you have two objects and do not need a reusable scene. Geometry is converted for the selected backend for each call. For repeated checks, build a [scene](scenes-and-trajectories.md).

## Discrete collision and distance

```python
from crcc import Circle, CollisionBackend, Pose

robot = Circle(0.5)
obstacle = Circle(1.0)
obstacle_pose = Pose.from_translation((3.0, 0.0))

assert not robot.collides(
    obstacle,
    pos_other=obstacle_pose,
    backend=CollisionBackend.Parry,
)
assert abs(robot.distance(obstacle, pos_other=obstacle_pose) - 1.5) < 1e-12
```

`collides` accepts a pose for each object; omitted poses are identity. `distance` returns non-negative separation and returns zero for overlap or contact. Distance to empty geometry is unsupported. See [geometry and poses](../concepts/geometry-and-poses.md) and [errors and limits](../reference/errors-and-limits.md).

## Check motion between poses

```python
from crcc import Circle, CollisionBackend, Pose, Rectangle

moving = Circle(0.5)
barrier = Rectangle(0.25, 3.0)
possible_collision = moving.collides_continuous(
    Pose.from_translation((-2.0, 0.0)),
    Pose.from_translation((2.0, 0.0)),
    barrier,
    Pose.identity(),
    Pose.identity(),
    backend=CollisionBackend.Parry,
)
assert possible_collision
```

The result is conservative: `False` certifies interval separation; `True` means collision is possible. Backend algorithms and limitations differ. See [continuous collision](../concepts/continuous-collision.md) before using the result in safety-related logic.

## Select a backend

Python pair methods accept `backend=`. Omitting it selects the compiled default, which is Parry when enabled. `engine=` and `CollisionEngine` remain deprecated aliases. Scene queries select a backend when constructing the builder:

```python
from crcc import CollisionBackend, CollisionCheckerBuilder

checker = CollisionCheckerBuilder(backend=CollisionBackend.Rhusics).build()
assert checker.backend == CollisionBackend.Rhusics
```

Check the [backend matrix](../concepts/backends.md) before depending on contact or continuous-query edge behavior.

## Rust pair query

```rust
use crcc::{CollisionEngine, CollisionObject, Pose};

fn main() -> Result<(), crcc::CrccError> {
    let robot = CollisionObject::circle((0.0, 0.0), 0.5)?;
    let obstacle = CollisionObject::circle((0.0, 0.0), 1.0)?;
    let pose = Pose::translation(3.0, 0.0);

    assert!(!robot.collides(&obstacle, Pose::IDENTITY, pose, CollisionEngine::Parry)?);
    assert_eq!(robot.distance(&obstacle, Pose::IDENTITY, pose, CollisionEngine::Parry)?, 1.5);
    Ok(())
}
```

Rust pair calls return `CrccResult`; use `?` or match errors explicitly. The Rust `Pose` alias is `glamx::DPose2`.
