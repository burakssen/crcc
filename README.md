# CRCC

CRCC is a 2D collision-checking library for Rust and Python. It supports primitive and compound geometry, discrete and continuous collision queries, dynamic obstacles, ordered batch queries, and CommonRoad scenarios.

Continuous collision detection (CCD) is conservative: a negative result certifies separation, while a positive result may be an over-approximation.

## Choose your language

| Language | Start here | API reference |
| --- | --- | --- |
| Python | [Python guide](docs/python-guide.md) | [Python API](docs/python-api.md) |
| Rust | [Rust guide](docs/rust-guide.md) | [Rust API](docs/rust-api.md) |

Additional documentation:

- [Benchmark tool](tools/benchmark/README.md)
- [Broad-phase acceleration design note](docs/future-work.md)

## Development setup

The repository uses `uv` for the Python environment and Cargo for Rust:

```bash
git clone <repository-url> crcc
cd crcc
uv sync
```

Run the standard checks with:

```bash
uv run ruff check .
uv run pytest -q
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
```

## Tutorials and playground

The CLI exposes deterministic examples for the supported engines:

```bash
uv run main.py basic --engine parry
uv run main.py continuous --engine rhusics
uv run main.py commonroad --engine collide
uv run main.py playground
```

Run `uv run main.py` without an action to choose interactively.
