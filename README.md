# CommonRoad Collision Checker

Rust/Python collision checking utilities for CommonRoad scenarios.

## Usage

Running the example script will automatically build and install the package:

```bash
uv run main.py
```

Run a specific example directly:

```bash
uv run main.py smoke --scenario scenarios/ZAM_Yield-1_1_T-1.xml --engine parry
uv run main.py benchmark
uv run main.py visualize
```

## Development

Run the Rust checks:

```bash
cargo check --all-features
cargo test --all-features
cargo clippy --all-features --all-targets -- -D warnings
```

Run the Python checks:

```bash
uv run ruff check .
uv run python -m unittest discover -s tests
```

## Current Features

- Static and dynamic collision checks.
- Discrete and continuous collision detection.
- Parry and Rhusics collision engines.
- Python bindings with CommonRoad scenario helpers.
- Rust unit tests and Python example tests.

## Planned Work

- Plotting of collision objects etc.
- Support for dynamic obstacles with variable shapes
- Python: Separation of generic collision checking and CommonRoad-specific parts
- Documentation


# TODO
Add Occupancy Group tests
Add shape tests