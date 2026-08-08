# CRCC Documentation

CRCC checks two-dimensional collisions for Rust and Python, optimized for autonomous driving workflows and CommonRoad scenario evaluation. Convert CommonRoad scenarios directly into high-performance collision checkers or perform direct primitive and polygon queries.

## Key Capabilities

- **CommonRoad Integration**: Convert complete CommonRoad scenarios (lanelet road boundaries, static obstacles, dynamic predictions, time-varying occupancies) automatically via `crcc.commonroad`.
- **Primitive & Polygon Geometry**: Circles, rectangles, triangles, half-spaces, convex/non-convex polygons with holes, and structural compounds.
- **Motion & Continuous Checking**: Pose-based discrete queries and conservative continuous motion checks over 32-bit discrete time steps.
- **Immutable Scene Checkers**: Pre-converted static and dynamic scene indexes supporting single queries, prepared query reuse, and Rayon parallel batching.
- **Multi-Engine Backend**: Select between Parry, Rhusics, and Collide backend algorithms.

CRCC is a collision-query library, not a physics engine. It does not resolve contacts, advance simulation state, mutate a scene in place, or return contact manifolds.

## Choose a Starting Point

| Goal | Start here |
| --- | --- |
| Convert & Evaluate CommonRoad Scenarios | [CommonRoad Scenario Conversion](python-guide.md#commonroad-scenario-conversion) |
| Understand collision, time, and engine semantics | [Concepts and engines](concepts.md) |
| Use CRCC from Python | [Python usage guide](python-guide.md) |
| Look up a Python signature | [Python API reference](python-api.md) |
| Use CRCC from Rust | [Rust usage guide](rust-guide.md) |
| Look up Rust exports and methods | [Rust API reference](rust-api.md) |
| Understand implementation boundaries | [Architecture](architecture.md) |
| Build, test, benchmark, or release the project | [Development and benchmarks](development.md) |

## CommonRoad Scenario Conversion in One Minute

Convert any CommonRoad XML scenario directly into a collision checker:

```python
from commonroad.common.file_reader import CommonRoadFileReader
from crcc import CollisionCheckerBuilder, CollisionEngine
from crcc.commonroad import scenario_builder

# Load CommonRoad scenario
scenario, _ = CommonRoadFileReader("scenarios/DEU_MerzenichRather-2_870_T-149.xml").open()

# Build collision checker with road boundaries, static obstacles, and dynamic predictions
checker = scenario_builder(
    scenario,
    builder=CollisionCheckerBuilder(CollisionEngine.Parry),
).build()

assert checker.engine == CollisionEngine.Parry
```

## Python Query in One Minute

From a source checkout:

```bash
git lfs install
git lfs pull
uv sync --frozen
```

```python
from crcc import Circle, CollisionCheckerBuilder, Pose, Rectangle

checker = (
    CollisionCheckerBuilder()
    .with_static_obstacle(Rectangle(length=0.25, width=3.0))
    .build()
)

status = checker.collides_static(
    Circle(0.5),
    position=Pose.from_translation((2.0, 0.0)),
)
assert not status.collides
```

## Rust in One Minute

```rust
use crcc::{CollisionCheckerBuilder, CollisionEngine, CollisionObject, Pose};

fn main() -> Result<(), crcc::CrccError> {
    let wall = CollisionObject::rectangle(
        geo::Rect::new((-0.125, -1.5), (0.125, 1.5)),
        0.0,
    )?;
    let query = CollisionObject::circle((0.0, 0.0), 0.5)?;
    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(wall)
        .build_with_engine(CollisionEngine::default())?;

    let status = checker.collides_static_pos(&query, Pose::translation(2.0, 0.0))?;
    assert!(!status.collides());
    Ok(())
}
```

## The Continuous-Query Contract

Continuous methods inspect both endpoints and the motion between them. Their Boolean result is intentionally asymmetric:

- `False`: the interval is certified clear by the selected implementation.
- `True`: collision is possible; conservative broad-phase or rotational handling can produce false positives.

Exact contact and conservative behavior differ at backend boundaries. Read [Concepts and engines](concepts.md) before using CRCC for safety decisions or interpreting engine comparisons.

## Installation Status

CRCC currently has no PyPI or crates.io publication workflow. Python users can build a source checkout or install a compatible wheel from a GitHub release. Rust users can use a Git dependency or local path dependency. The repository launcher, scenarios, playground, and benchmark tools are development assets and are not installed by a Python wheel.
