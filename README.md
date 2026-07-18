# CRCC

CRCC is a Rust and Python 2D collision-checking library for primitive, compound, static, and time-varying geometry. It supports discrete queries, conservative continuous collision detection (CCD), ordered parallel batches, and CommonRoad conversion.


## Setup & Development

```bash
# Clone & install dependencies
git clone <repository-url> crcc && cd crcc
uv sync

# Run tests & lint
cargo test --all-features
uv run pytest -q
uv run ruff check .
```

## Quick Start

### Python
```python
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Circle, Rectangle
from crcc.pose import Pose

# Build checker with static rectangle
checker = (
    CollisionCheckerBuilder(CollisionEngine.Parry)
    .with_static_obstacle(Rectangle(2.0, 2.0))
    .build()
)

# Query static collision
status = checker.collides_static(Circle(0.5), Pose.identity())
assert status.collides
```

### Rust
Add to `Cargo.toml`:
```toml
[dependencies]
crcc = { path = "../crcc" }
geo = "0.32"
```

```rust
use crcc::collision_checker::{CollisionCheckerBuilder, CollisionStatus};
use crcc::collision_checker::engine::parry::ParryCollisionObject;
use crcc::collision_object::CollisionObject;

fn main() -> Result<(), crcc::error::CrccError> {
    let wall = CollisionObject::rectangle(geo::Rect::new((-1.0, -1.0), (1.0, 1.0)), 0.0)?;
    let robot: ParryCollisionObject = CollisionObject::circle((0.0, 0.0), 0.5)?.into();

    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(wall)
        .build::<ParryCollisionObject>();

    assert_eq!(checker.collides_static(&robot)?, CollisionStatus::CollidesStatic);
    Ok(())
}
```

## Documentation & Guides
- [Usage & Examples](docs/usage.md)
- [Python API Reference](docs/python-api.md)
- [Rust API Reference](docs/rust-api.md)

## Run CLI Tools
```bash
uv run main.py basic --engine parry
uv run main.py continuous --engine rhusics
uv run main.py commonroad --engine collide
uv run main.py study --benchmark-profile smoke
```
For benchmarking details, see [benchmark tool guide](tools/benchmark/README.md).
