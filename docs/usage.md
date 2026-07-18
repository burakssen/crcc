# Using CRCC

This guide provides practical examples for common collision-checking tasks in both Python and Rust.

## Core Semantics

- **Geometry & Poses**: Shapes are defined in local coordinates and placed with a `Pose`.
- **Time Windows**: Python uses inclusive bounds (`min_time` / `max_time`). Rust uses `TimeStep` ranges (e.g. `t1..=t2`).
- **CCD**: Continuous Collision Detection is conservative. `False` guarantees separation; `True` indicates a potential collision.
- **Batch Queries**: Results are returned in parallel (using Rayon) but always preserve input order.
- **API Boundary**: Only package-root exports and the Python `crcc.commonroad` module are public.

---

## 1. Pair Collision and Distance

### Python
```python
from crcc.collision_checker import CollisionEngine
from crcc.collision_object import Circle
from crcc.pose import Pose

left, right = Circle(1.0), Circle(1.0)
r_pose = Pose.from_translation((3.0, 0.0))

assert not left.collides(right, pos_other=r_pose, engine=CollisionEngine.Parry)
assert left.distance(right, pos_other=r_pose, engine=CollisionEngine.Parry) == 1.0
```

### Rust
```rust
use crcc::collision_checker::engine::CollisionEngine;
use crcc::collision_object::CollisionObject;
use glamx::DPose2 as Pose;

# fn main() -> Result<(), crcc::error::CrccError> {
let left = CollisionObject::circle((0.0, 0.0), 1.0)?;
let right = CollisionObject::circle((0.0, 0.0), 1.0)?;
let r_pose = Pose::translation(3.0, 0.0);

assert!(!left.collides(&right, Pose::IDENTITY, r_pose, CollisionEngine::Parry)?);
assert_eq!(left.distance(&right, Pose::IDENTITY, r_pose, CollisionEngine::Parry)?, 1.0);
# Ok(())
# }
```

---

## 2. Continuous Pair Collision

### Python
```python
from crcc.collision_checker import CollisionEngine
from crcc.collision_object import Circle, Rectangle
from crcc.pose import Pose

moving = Circle(0.5)
barrier = Rectangle(0.25, 3.0)

# Check motion from (-2.0, 0) to (2.0, 0)
hit = moving.collides_continuous(
    Pose.from_translation((-2.0, 0.0)), Pose.from_translation((2.0, 0.0)),
    barrier, Pose.identity(), Pose.identity(),
    CollisionEngine.Parry,
)
assert hit
```

### Rust
```rust
use crcc::collision_checker::engine::CollisionEngine;
use crcc::collision_object::CollisionObject;
use glamx::DPose2 as Pose;

# fn main() -> Result<(), crcc::error::CrccError> {
let moving = CollisionObject::circle((0.0, 0.0), 0.5)?;
let barrier = CollisionObject::rectangle(geo::Rect::new((-0.125, -1.5), (0.125, 1.5)), 0.0)?;

let hit = moving.collides_continuous(
    Pose::translation(-2.0, 0.0), Pose::translation(2.0, 0.0),
    &barrier, Pose::IDENTITY, Pose::IDENTITY,
    CollisionEngine::Parry,
)?;
assert!(hit);
# Ok(())
# }
```

---

## 3. Builder and Engine Selection

### Python
```python
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Circle, Rectangle
from crcc.pose import Pose

checker = (
    CollisionCheckerBuilder(CollisionEngine.Rhusics)
    .with_static_obstacle(Rectangle(2.0, 2.0))
    .build()
)
assert checker.collides_static(Circle(0.5), Pose.identity()).collides
assert checker.engine == CollisionEngine.Rhusics
```

### Rust
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

---

## 4. Dynamic Obstacles and Time Windows

Static geometry is always checked. Dynamic geometry is checked only within the specified time step window.

### Python
```python
from crcc.collision_checker import CollisionCheckerBuilder
from crcc.collision_object import Circle, Rectangle
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose

moving = DynamicObstacle(
    Circle(0.5),
    [Pose.from_translation((-2.0, 0.0)), Pose.from_translation((2.0, 0.0))],
    time_offset=10,
)
checker = CollisionCheckerBuilder().with_static_obstacle(Rectangle(0.25, 3.0)).build()

status = checker.collides_dynamic(moving, min_time=10, max_time=11)
assert status.collides
assert status.time_step == 10
```

### Rust
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
Use `DynamicObstacle.from_time_variant` (Python) or `DynamicObstacle::time_variant` (Rust) for time-varying shape geometries.

---

## 5. Parallel Batch Queries

### Python
```python
from crcc.collision_checker import CollisionCheckerBuilder
from crcc.collision_object import Circle, Rectangle
from crcc.pose import Pose

checker = CollisionCheckerBuilder().with_static_obstacle(Rectangle(2.0, 2.0)).build()
queries = [(Circle(0.25), Pose.identity()), (Circle(0.25), Pose.from_translation((5.0, 0.0)))]

results = checker.par_static(queries)
assert [res.collides for res in results] == [True, False]
```

### Rust
*(Requires the `rayon` feature enabled)*
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

---

## 6. CommonRoad Scenario Conversion (Python Only)

```python
from commonroad.common.file_reader import CommonRoadFileReader
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.commonroad import scenario_builder

scenario, _ = CommonRoadFileReader("scenarios/DEU_MerzenichRather-2_870_T-149.xml").open()
checker = scenario_builder(
    scenario,
    CollisionCheckerBuilder(CollisionEngine.Parry),
).build()
```
Scenario conversion adds road boundaries, static occupancies, and predicted dynamic obstacle trajectories automatically.

---

## 7. Approximation and Interpolation Semantics

To perform Continuous Collision Detection (CCD) and dynamic trajectory checking efficiently, CRCC utilizes specific geometric approximations and pose interpolation models:

### Pose Interpolation
When query objects or obstacles move between discrete keyframes or endpoints:
* **Translation**: Linearly interpolated (`lerp`) using standard vector math.
* **Rotation**: Spherically linearly interpolated (`slerp`) using orientation matrices/quaternions (in Rust) or shortest-path angular modular interpolation (in Python).

### Swept Area Approximations
Continuous Collision Detection (CCD) checks the sweep of shapes over a time interval. Because simultaneous translation and rotation generate complex transcendental boundaries, CRCC over-approximates swept geometries:
1. **Translation Only (No Rotation)**: The swept area is computed using the `convex_hull` of the union of the start and end polygons. This is exact for convex polygons and conservative for non-convex polygons:
   $$\text{Swept Area} = \text{ConvexHull}(\text{start} \cup \text{end})$$
2. **Translation with Rotation**: The swept bounds are conservatively over-approximated using bounding box or bounding sphere enclosures (constructed from the maximum vertex radius of the shape).
3. **Half-Spaces with Rotation**: If a half-space rotates, it sweeps the entire plane, so the swept area is approximated by a `FullSpace` collision object.

### Controlling Accuracy
* **Conservative CCD guarantees safety**: A result of `False` certifies absolute separation, whereas `True` indicates a potential intersection.
* **Precision Tuning**: Dividing trajectories into smaller time intervals reduces the angular rotation per step and tightens the conservative bounds. The CCD engine also recursively subdivides candidate intervals until either separation is proven, a collision is found, or its tolerance/depth limits are reached.
