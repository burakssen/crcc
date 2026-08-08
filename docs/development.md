# Development and Benchmarks

This page covers the repository workflow. End-user API usage belongs in the language guides.

## Prerequisites

- Git and Git LFS.
- Python 3.10 or newer.
- A recent stable Rust toolchain supporting edition 2024.
- A native compiler toolchain for PyO3 builds.
- `uv` for Python environments and tools.

The project does not declare an exact minimum Rust version. CI uses the current stable toolchain.

## Checkout

```bash
git clone https://github.com/burakssen/crcc.git
cd crcc
git lfs install
git lfs pull
git lfs ls-files
uv sync --frozen
```

Scenario XML files use Git LFS. Parsing failures that show an LFS pointer instead of XML usually mean `git lfs pull` was not run.

Rebuild the extension explicitly after Rust or binding changes:

```bash
uv run --frozen --with "maturin>=1.0,<2.0" maturin develop
```

## Standard Validation

```bash
uv run --frozen pre-commit run --all-files --show-diff-on-failure
uv run --frozen pyright
uv run --frozen pytest -q
```

Pre-commit runs file hygiene, YAML checks, Ruff, Rust formatting, Cargo check, and Clippy. Some hooks are formatters and may modify files; rerun after reviewing their changes.

Exercise every supported Rust feature boundary:

```bash
cargo test --locked --no-default-features
cargo test --locked --no-default-features --features parry
cargo test --locked --no-default-features --features rhusics
cargo test --locked --no-default-features --features collide
cargo test --locked --all-features
```

Build documentation exactly as CI does:

```bash
uvx --from mkdocs==1.6.1 mkdocs build --strict
```

Generated documentation is written to `site/` and is ignored by Git.

## Repository Tutorials

The launcher defaults to Rhusics. Select another engine explicitly when comparing behavior.

```bash
uv run main.py basic --engine parry
uv run main.py continuous --engine rhusics
uv run main.py commonroad --engine collide
uv run main.py all --engine parry
```

Use a different materialized CommonRoad scenario:

```bash
uv run main.py commonroad \
  --engine parry \
  --scenario scenarios/ZAM_Tutorial-1_2_T-1.xml
```

Running `uv run main.py` without an action opens a terminal selector. Compatibility action names `concepts`, `shapes`, `dynamics`, and `scenario` are aliases of the three canonical tutorials and are not separate workflows.

## Playground

```bash
uv run main.py playground
```

The playground opens a blocking Matplotlib GUI. It supports geometry placement, dynamic paths, time-varying occupancy, engine switching, timeline playback, and scenario context. It requires a graphical display and materialized LFS scenario data, so it is not a headless CI workflow.

## Package Build and Smoke Test

Build a source distribution, then build the wheel from that exact source artifact:

```bash
uv build --sdist
uv build --wheel dist/*.tar.gz
```

Test the wheel outside the project environment:

```bash
uv venv --clear /tmp/crcc-smoke
uv pip install --python /tmp/crcc-smoke/bin/python dist/*.whl
/tmp/crcc-smoke/bin/python -c "import crcc"
```

The project is not published to PyPI or crates.io. Tagged GitHub releases attach an sdist and platform wheels.

## Python Benchmark Pipeline

The benchmark pipeline is a research tool, not package API. Start with a deliberately bounded smoke command rather than the default all-suite matrix:

```bash
uv run main.py study \
  --benchmark-profile smoke \
  --benchmark-suite pair continuous distance \
  --benchmark-samples 100 \
  --benchmark-repetitions 2 \
  --benchmark-output target/crcc-python-bench
```

Regenerate reports from existing artifacts without rerunning measurements:

```bash
uv run main.py report --benchmark-output target/crcc-python-bench
```

Available suites are:

```text
pair continuous distance shape_complexity coverage_matrix scene_scaling
update_proxy rebuild_update api_overhead dynamic_batch time_variant
native_layers parallel density_scaling dynamic_scene scenario
```

Restrict engines, scenarios, and thread counts when investigating one question:

```bash
uv run main.py study \
  --benchmark-profile smoke \
  --benchmark-suite scenario parallel \
  --benchmark-engines parry rhusics \
  --benchmark-scenarios scenarios/ZAM_Tutorial-1_2_T-1.xml \
  --benchmark-thread-counts 1 2 4 \
  --benchmark-output target/crcc-python-bench
```

See the [benchmark tool reference](https://github.com/burakssen/crcc/blob/main/tools/benchmark/README.md) for profiles, artifacts, native binaries, and interpretation limits.

## Native Benchmark Binaries

Compare native backend objects with public Rust-layer calls:

```bash
cargo run --release --locked \
  --bin native_benchmark \
  --features benchmarking,parry,rhusics,collide \
  -- parry native circle_clear 100000
```

Compare scalar and reusable batch execution:

```bash
cargo run --release --locked \
  --bin parallel_benchmark \
  --features rayon,parry,rhusics,collide \
  -- parry static 1024 4 10
```

Both binaries emit CSV. The parallel binary verifies result equivalence before timing.

## Benchmark Interpretation

- `smoke` validates harness execution; it does not support publication claims.
- Results depend on hardware, thermal state, allocator behavior, and process order.
- Confidence intervals summarize repetitions in this harness, not hardware-independent performance.
- Conservative CCD false positives are permitted; false negatives and query errors are correctness failures.
- `update_proxy` changes query poses, not scene state.
- `rebuild_update` measures full immutable-checker reconstruction.
- Memory measurements include Python wrappers, allocator retention, and page granularity.
- Stress workloads are isolated capacity checks, not additions to every benchmark matrix.

## CI Ownership

GitHub runs public CI, cross-platform tagged wheel builds, GitHub Releases, and GitHub Pages. GitLab independently validates Linux merge requests, default-branch commits, and tags. It does not publish public documentation or packages.

Routine CI tests Python 3.10 and 3.13 because the extension uses the CPython stable ABI from 3.10. Release jobs build one `cp310-abi3` wheel per supported platform/architecture and smoke-test each wheel.

## Release Contract

Release tags must use `v<major>.<minor>.<patch>` and match the versions in both `Cargo.toml` and `pyproject.toml`.

```bash
git tag v0.1.0
git push origin v0.1.0
```

The release workflow validates the project, builds the sdist, builds wheels from the unpacked sdist, smoke-tests them, and creates a GitHub Release with generated notes. It does not publish to PyPI, crates.io, or the GitLab package registry.

## Current Policy Boundaries

The repository does not define contribution governance, a license, a security-reporting policy, or a changelog process. Do not infer those policies from the build configuration. Add the corresponding policy files before documenting commitments that do not yet exist.
