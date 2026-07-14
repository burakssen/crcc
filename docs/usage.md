# Using CRCC

This guide presents the same tasks in Python and Rust. See the [Python API](python-api.md) or [Rust API](rust-api.md) for complete signatures.

## Shared semantics

Geometry is defined in local coordinates and placed with a pose. Pair queries receive a pose for each operand; checker queries receive the query pose because scene geometry is already positioned.

Python `min_time` and `max_time` bounds are inclusive. Rust accepts standard ranges over `TimeStep`: `a..=b` includes both endpoints and `a..b` excludes `b`. Omitting bounds searches all representable steps. To check motion from step `t` to `t + 1`, include both endpoints in the range.

Batch results always match input order. Implementations may execute sufficiently large batches in parallel, but that does not change results or ordering.

CCD is conservative. `False` certifies separation over the entire interval. `True` reports a possible collision and may be conservative for rotations, changing shapes, or numerically unresolved cases. Backend contact classification and unsupported shape-operation combinations can differ; these are semantic differences, not performance conclusions.

Only package/crate-root names and the Python `crcc.commonroad` module are public. Underscore-prefixed modules and methods, backend representations, and research-tool support are implementation details.

## Pair collision and distance

Python:

```python
from crcc.collision_checker import CollisionEngine
from crcc.collision_object import Circle
from crcc.pose import Pose

left = Circle(1.0)
right = Circle(1.0)
right_pose = Pose.from_translation((3.0, 0.0))

assert not left.collides(right, pos_other=right_pose, engine=CollisionEngine.Parry)
assert left.distance(right, pos_other=right_pose, engine=CollisionEngine.Parry) == 1.0
```

Rust:

```rust
use crcc::collision_checker::engine::CollisionEngine;
use crcc::collision_object::CollisionObject;
use glamx::DPose2 as Pose;

# fn main() -> Result<(), crcc::error::CrccError> {
let left = CollisionObject::circle((0.0, 0.0), 1.0)?;
let right = CollisionObject::circle((0.0, 0.0), 1.0)?;
let right_pose = Pose::translation(3.0, 0.0);

assert!(!left.collides(&right, Pose::IDENTITY, right_pose, CollisionEngine::Parry)?);
assert_eq!(left.distance(&right, Pose::IDENTITY, right_pose, CollisionEngine::Parry)?, 1.0);
# Ok(())
# }
```

## Continuous pair collision

Python:

```python
from crcc.collision_checker import CollisionEngine
from crcc.collision_object import Circle, Rectangle
from crcc.pose import Pose

moving = Circle(0.5)
barrier = Rectangle(0.25, 3.0)
possible_hit = moving.collides_continuous(
    Pose.from_translation((-2.0, 0.0)),
    Pose.from_translation((2.0, 0.0)),
    barrier,
    Pose.identity(),
    Pose.identity(),
    CollisionEngine.Parry,
)
assert possible_hit
```

Rust:

```rust
use crcc::collision_checker::engine::CollisionEngine;
use crcc::collision_object::CollisionObject;
use glamx::DPose2 as Pose;

# fn main() -> Result<(), crcc::error::CrccError> {
let moving = CollisionObject::circle((0.0, 0.0), 0.5)?;
let barrier = CollisionObject::rectangle(geo::Rect::new((-0.125, -1.5), (0.125, 1.5)), 0.0)?;
let possible_hit = moving.collides_continuous(
    Pose::translation(-2.0, 0.0),
    Pose::translation(2.0, 0.0),
    &barrier,
    Pose::IDENTITY,
    Pose::IDENTITY,
    CollisionEngine::Parry,
)?;
assert!(possible_hit);
# Ok(())
# }
```

## Build a checker and select an engine

Python:

```python
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Circle, Rectangle
from crcc.pose import Pose

checker = (
    CollisionCheckerBuilder(CollisionEngine.Rhusics)
    .with_static_obstacle(Rectangle(2.0, 2.0))
    .build()
)
status = checker.collides_static(Circle(0.5), Pose.identity())
assert status.collides
assert checker.engine == CollisionEngine.Rhusics
```

Rust:

```rust
use crcc::collision_checker::CollisionCheckerBuilder;
use crcc::collision_checker::engine::CollisionEngine;
use crcc::collision_object::CollisionObject;
use glamx::DPose2 as Pose;

# fn main() -> Result<(), crcc::error::CrccError> {
let wall = CollisionObject::rectangle(geo::Rect::new((-1.0, -1.0), (1.0, 1.0)), 0.0)?;
let query = CollisionObject::circle((0.0, 0.0), 0.5)?;
let checker = CollisionCheckerBuilder::new()
    .with_static_obstacle(wall)
    .build_with_engine(CollisionEngine::Rhusics)?;

assert!(checker.collides_static(&query)?.collides());
assert_eq!(checker.engine(), CollisionEngine::Rhusics);
# Ok(())
# }
```

## Dynamic obstacles and time windows

The first pose is active at `time_offset`. Each later pose advances one integer step. A fixed-shape checker query is tested against dynamic scene objects only inside the requested time window; static scene geometry is always checked.

Python:

```python
from crcc.collision_checker import CollisionCheckerBuilder
from crcc.collision_object import Circle, Rectangle
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose

moving = DynamicObstacle(
    Circle(0.5),
    [Pose.from_translation((-2.0, 0.0)), Pose.from_translation((2.0, 0.0))],
    10,
)
checker = CollisionCheckerBuilder().with_static_obstacle(Rectangle(0.25, 3.0)).build()
status = checker.collides_dynamic(moving, min_time=10, max_time=11)
assert status.collides
assert status.time_step == 10
```

Rust:

```rust
use crcc::collision_checker::CollisionCheckerBuilder;
use crcc::collision_checker::engine::CollisionEngine;
use crcc::collision_object::CollisionObject;
use crcc::dynamic_obstacle::DynamicObstacle;
use crcc::time::TimeStep;
use glamx::DPose2 as Pose;

# fn main() -> Result<(), crcc::error::CrccError> {
let moving = DynamicObstacle::new(
    CollisionObject::circle((0.0, 0.0), 0.5)?,
    vec![Pose::translation(-2.0, 0.0), Pose::translation(2.0, 0.0)],
    TimeStep(10),
);
let barrier = CollisionObject::rectangle(geo::Rect::new((-0.125, -1.5), (0.125, 1.5)), 0.0)?;
let checker = CollisionCheckerBuilder::new()
    .with_static_obstacle(barrier)
    .build_with_engine(CollisionEngine::Parry)?;
let status = checker.collides_dynamic_range(&moving, TimeStep(10)..=TimeStep(11))?;
assert!(status.collides());
# Ok(())
# }
```

Use Python `DynamicObstacle.from_time_variant(objects, time_offset, positions)` or Rust `DynamicObstacle::time_variant(objects, positions, time_offset)` when geometry changes between steps. Shape and pose counts must match.

## Batch queries

Python batches are always available because the extension is built with Rayon:

```python
from crcc.collision_checker import CollisionCheckerBuilder
from crcc.collision_object import Circle, Rectangle
from crcc.pose import Pose

checker = CollisionCheckerBuilder().with_static_obstacle(Rectangle(2.0, 2.0)).build()
queries = [(Circle(0.25), Pose.identity()), (Circle(0.25), Pose.from_translation((5.0, 0.0)))]
statuses = checker.par_static(queries)
assert [status.collides for status in statuses] == [True, False]
```

In Rust, enable `rayon` and pass positioned objects or dynamic obstacles:

```rust
# use crcc::collision_checker::CollisionCheckerBuilder;
# use crcc::collision_checker::engine::CollisionEngine;
# use crcc::collision_object::CollisionObject;
# use glamx::DPose2 as Pose;
# fn main() -> Result<(), crcc::error::CrccError> {
# let scene = CollisionObject::circle((0.0, 0.0), 1.0)?;
# let query = CollisionObject::circle((0.0, 0.0), 0.25)?;
# let checker = CollisionCheckerBuilder::new().with_static_obstacle(scene).build_with_engine(CollisionEngine::Parry)?;
let results = checker.collides_static_batch(
    &[(query.clone(), Pose::IDENTITY), (query, Pose::translation(5.0, 0.0))],
    ..,
);
assert_eq!(results.len(), 2);
assert!(results[0].as_ref().unwrap().collides());
assert!(!results[1].as_ref().unwrap().collides());
# Ok(())
# }
```

## CommonRoad workflow (Python)

```python
from commonroad.common.file_reader import CommonRoadFileReader
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.commonroad import create_collision_checker_from_scenario

scenario, _ = CommonRoadFileReader("scenarios/DEU_MerzenichRather-2_870_T-149.xml").open()
checker = create_collision_checker_from_scenario(
    scenario,
    CollisionCheckerBuilder(CollisionEngine.Parry),
).build()
```

The scenario conversion adds the road boundary, static occupancies, and supported dynamic predictions. Individual conversion helpers are listed in the [Python API](python-api.md).
