# CRCC Documentation

CRCC checks collisions among two-dimensional geometry from Rust and Python. Use it for direct shape queries, continuous motion checks, immutable scenes containing static and dynamic obstacles, ordered batch queries, or conversion from CommonRoad scenarios.

## What CRCC Models

- Primitive geometry: circles, rectangles, triangles, half-spaces, empty space, and full space.
- Polygon geometry: convex, non-convex, and holed polygons.
- Compounds: unions of any supported geometry.
- Motion: one pose per signed 32-bit time step, with conservative checks between adjacent poses.
- Scenes: immutable collections of static geometry and dynamic trajectories.
- Backends: Parry, Rhusics, and Collide behind the same high-level API.

CRCC is a collision-query library, not a physics engine. It does not resolve contacts, advance simulation state, mutate a scene in place, or return contact manifolds.

## Choose a Starting Point

| Goal | Start here |
| --- | --- |
| Understand collision, time, and engine semantics | [Concepts and engines](concepts.md) |
| Use CRCC from Python | [Python usage guide](python-guide.md) |
| Look up a Python signature | [Python API reference](python-api.md) |
| Use CRCC from Rust | [Rust usage guide](rust-guide.md) |
| Look up Rust exports and methods | [Rust API reference](rust-api.md) |
| Understand implementation boundaries | [Architecture](architecture.md) |
| Build, test, benchmark, or release the project | [Development and benchmarks](development.md) |

## Python in One Minute

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
