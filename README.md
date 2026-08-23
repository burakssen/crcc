# CRCC

CRCC is a high-performance two-dimensional collision-checking library for Rust and Python, designed for motion planning and CommonRoad scenario evaluation. It provides direct conversion of CommonRoad scenarios (road boundaries, static obstacles, and dynamic trajectories), validated primitive and polygon geometry, discrete and continuous pair queries, immutable static/dynamic scenes, prepared queries, and ordered native batches.

Continuous collision detection is conservative: `False` certifies separation for the complete interval, while `True` may represent either a collision or a conservative positive.

## Documentation

- [Documentation home](docs/index.md)
- [CommonRoad & Python usage guide](docs/python-guide.md) and [Python API](docs/python-api.md)
- [Core concepts and engine behavior](docs/concepts.md)
- [Rust guide](docs/rust-guide.md) and [Rust API](docs/rust-api.md)
- [Architecture](docs/architecture.md)
- [Development and benchmarks](docs/development.md)

The MkDocs site is configured for `https://burakssen.com/crcc/`. Until GitHub Pages is enabled for the repository, use the checked-in pages linked above.

## Quick Start

CRCC is not currently published to PyPI or crates.io. Use a source checkout, a wheel attached to a GitHub release, or a Git dependency.

### Python As a Git Dependency

Install directly from GitHub into an environment (requires a Rust toolchain for the Maturin build step):

```bash
uv pip install git+https://github.com/burakssen/crcc
# or with pip
pip install git+https://github.com/burakssen/crcc
# or add to a uv project
uv add git+https://github.com/burakssen/crcc
```

Or add it to `pyproject.toml`:

```toml
[project]
dependencies = [
    "crcc @ git+https://github.com/burakssen/crcc",
]
```

### CommonRoad Scenario Conversion

Convert CommonRoad XML scenarios into high-speed collision checkers:

```python
from commonroad.common.file_reader import CommonRoadFileReader
from crcc import CollisionBackend, CollisionCheckerBuilder
from crcc.commonroad import scenario_builder

scenario, _ = CommonRoadFileReader("scenarios/DEU_MerzenichRather-2_870_T-149.xml").open()

# Converts lanelet road boundary, static obstacles, and dynamic predictions
checker = scenario_builder(
    scenario,
    builder=CollisionCheckerBuilder(CollisionBackend.Parry),
).build()
```

### Python From Source

Prerequisites are Git, Git LFS, a recent stable Rust toolchain, Python 3.10 or newer, and [`uv`](https://docs.astral.sh/uv/).

```bash
git clone https://github.com/burakssen/crcc.git
cd crcc
git lfs install
git lfs pull
uv sync --frozen
```

```python
from crcc import Circle, CollisionBackend, Pose

robot = Circle(0.5)
obstacle = Circle(1.0)
obstacle_pose = Pose.from_translation((3.0, 0.0))

assert not robot.collides(obstacle, pos_other=obstacle_pose, backend=CollisionBackend.Parry)
assert robot.distance(obstacle, pos_other=obstacle_pose) == 1.5
```

### Rust As a Git Dependency

Choose only the backend features the application needs:

```toml
[dependencies]
crcc = { git = "https://github.com/burakssen/crcc", default-features = false, features = ["parry"] }
geo = "0.32"
```

```rust
use crcc::{CollisionEngine, CollisionObject, Pose};

fn main() -> Result<(), crcc::CrccError> {
    let robot = CollisionObject::circle((0.0, 0.0), 0.5)?;
    let obstacle = CollisionObject::circle((0.0, 0.0), 1.0)?;
    let obstacle_pose = Pose::translation(3.0, 0.0);

    assert!(!robot.collides(
        &obstacle,
        Pose::IDENTITY,
        obstacle_pose,
        CollisionEngine::Parry,
    )?);
    Ok(())
}
```

## Repository Tutorials

`main.py` is a repository launcher; installing a wheel does not install a `crcc` command.

```bash
uv run main.py basic --engine parry
uv run main.py continuous --engine rhusics
uv run main.py commonroad --engine collide
uv run main.py playground
```

The launcher defaults to Rhusics. The compiled library API defaults to Parry when Parry is enabled.

## Development Checks

```bash
uv run --frozen pre-commit run --all-files --show-diff-on-failure
uv run --frozen pyright
uv run --frozen pytest -q
cargo test --locked --no-default-features
cargo test --locked --all-features
uvx --from mkdocs==1.6.1 mkdocs build --strict
```

See [Development and benchmarks](docs/development.md) for the complete feature matrix, package smoke test, CLI, playground, benchmark profiles, and release behavior.
