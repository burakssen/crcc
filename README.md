# CRCC

CRCC is a Rust and Python 2D collision-checking library for primitive, compound, static, and time-varying geometry. It supports discrete queries, conservative continuous collision detection (CCD), ordered Rayon batches, CommonRoad conversion, and runtime selection among Parry, Rhusics, and Collide.

The project remains on the 0.1 API. Python concepts live in their named modules, and Rust exposes both generic engine-typed checkers and runtime-selected checkers.

## Source-checkout setup

```bash
git clone <repository-url> crcc
cd crcc
uv sync
cargo test
```

No PyPI or crates.io release is assumed.

## Python quick start

```python
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Circle, Rectangle
from crcc.pose import Pose

checker = (
    CollisionCheckerBuilder(CollisionEngine.Parry)
    .with_static_obstacle(Rectangle(2.0, 2.0))
    .build()
)
status = checker.collides_static(Circle(0.5), Pose.identity())
assert status.collides
```

The package root continues to re-export the same core classes for convenience. CommonRoad conversion helpers live in `crcc.commonroad`.

## Rust quick start

```toml
[dependencies]
crcc = { path = "../crcc" }
geo = "0.32"
```

```rust
use crcc::collision_checker::{CollisionCheckerBuilder, CollisionStatus};
use crcc::collision_checker::engine::parry::ParryCollisionObject;
use crcc::collision_object::CollisionObject;

# fn main() -> Result<(), crcc::error::CrccError> {
let wall = CollisionObject::rectangle(
    geo::Rect::new((-1.0, -1.0), (1.0, 1.0)),
    0.0,
)?;
let robot: ParryCollisionObject = CollisionObject::circle((0.0, 0.0), 0.5)?.into();
let checker = CollisionCheckerBuilder::new()
    .with_static_obstacle(wall)
    .build::<ParryCollisionObject>();

assert_eq!(checker.collides_static(&robot)?, CollisionStatus::CollidesStatic);
# Ok(())
# }
```

Use `build_with_engine(crcc::collision_checker::engine::CollisionEngine::Parry)` when the backend must be chosen at runtime.

## Core semantics

- Collision objects contain local geometry; poses position them for queries.
- Dynamic obstacle samples are assigned to consecutive integer time steps.
- Python time bounds are inclusive. Rust uses ordinary ranges over `TimeStep`.
- Include both `t` and `t + 1` to check continuous motion across that segment.
- `False` from CCD certifies separation. `True` may be conservative.
- Batch results preserve input order. The established Python names are `par_static`, `par_static_threads`, and `par_dynamic`.
- Exact contact and unsupported-operation behavior can differ by backend.

Read the [usage guide](docs/usage.md), [Python API reference](docs/python-api.md), or [Rust API reference](docs/rust-api.md).

## Tutorials and research tools

```bash
uv run main.py basic --engine parry
uv run main.py continuous --engine rhusics
uv run main.py commonroad --engine collide
uv run main.py playground --engine parry
uv run main.py study --benchmark-profile smoke
uv run main.py report --benchmark-output benchmark_results
```

The earlier `concepts`, `shapes`, `dynamics`, `scenario`, and `all` command names remain available and route to the cleaned tutorials.

Benchmark details are in the [benchmark tool guide](tools/benchmark/README.md). Research-tool modules and backend benchmark support are not library APIs.

## Development

```bash
cargo fmt --all --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-features
cargo test --doc
cargo doc --no-deps --all-features
uv run ruff check .
uv run pytest -q
```
