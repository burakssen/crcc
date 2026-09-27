# Installation

CRCC is not currently published to PyPI or crates.io. Python users can install from a GitHub release wheel or build from source; Rust users can use a Git or path dependency.

## Python source checkout

Prerequisites: Git, Git LFS, Python 3.10+, a recent stable Rust toolchain, and [`uv`](https://docs.astral.sh/uv/).

```bash
git clone https://github.com/burakssen/crcc.git
cd crcc
git lfs install
git lfs pull
uv sync --frozen
uv run python -c "import crcc; print(crcc.CollisionBackend.Parry)"
```

Git LFS materializes the CommonRoad XML scenarios used by tutorials and tests. The CRCC package itself can still be used for collision queries without loading those scenarios.

To install a compatible wheel downloaded from a GitHub release:

```bash
uv venv
uv pip install ./crcc-0.1.0-cp310-abi3-<platform>.whl
```

Release wheels use the CPython stable ABI beginning with Python 3.10; the wheel must match the operating system and architecture.

## Python Git dependency

Installing from Git builds the native extension and therefore requires a Rust toolchain:

```bash
uv add git+https://github.com/burakssen/crcc
```

The equivalent dependency declaration is:

```toml
[project]
dependencies = ["crcc @ git+https://github.com/burakssen/crcc"]
```

## Rust dependency

The crate uses Rust edition 2024 and does not declare an exact minimum supported Rust version. Select the backend features needed by your application:

```toml
[dependencies]
crcc = { git = "https://github.com/burakssen/crcc", default-features = false, features = ["parry"] }
geo = "0.32"
```

The default Cargo features enable `parry`, `rhusics`, and `collide`. Add `rayon` to enable Rust batch-query APIs. See [Cargo features](../reference/rust.md#cargo-features) and the [quick start](quick-start.md).
