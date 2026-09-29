# Installation

This collision library is not published to PyPI or crates.io: **PyPI's `crcc` is an unrelated CRC package**. Use a Git/path dependency or build from this repository.

## Python source checkout

Prerequisites: Git, Python 3.10+, current stable Rust, a native linker, and [`uv`](https://docs.astral.sh/uv/). Install Git LFS when using bundled scenarios. On Linux, install your distribution's C/C++ build tools; on macOS, install Xcode Command Line Tools; on Windows, use the native build tools required by your Rust target (MSVC Build Tools for the standard MSVC target).

The source uses edition 2024 and let-chain syntax (Rust 1.88 or later). No dependency-inclusive minimum supported Rust version is declared or tested; use current stable.

```bash
git clone https://github.com/burakssen/crcc.git
cd crcc
git lfs install
git lfs pull
uv sync --frozen
uv run python -c "import crcc; print(crcc.CollisionBackend.Parry)"
```

Git LFS materializes the CommonRoad XML scenarios used by tutorials and tests. The CRCC package itself can still be used for collision queries without loading those scenarios.

## Released wheels

To install a downloaded wheel whose API and platform you have selected:

```bash
uv venv
uv pip install ./path/to/downloaded-wheel.whl
```

Release wheels use the CPython stable ABI beginning with Python 3.10; the wheel must match the operating system and architecture.

| Platform | Release wheel availability | Routine behavioral CI |
| --- | --- | --- |
| Linux x86-64 / ARM64 | manylinux 2.28 (glibc 2.28+) | Linux, Python 3.10 and 3.13 |
| macOS Intel / Apple Silicon | macOS 11.0+ tags | Wheel import smoke checks |
| Windows x86-64 / ARM64 | Native Windows wheels | Wheel import smoke checks |
| Alpine / musl | No musllinux release wheels | Not tested by the wheel matrix |

These release assets do not establish behavioral parity on every platform. No free-threaded Python support guarantee is made by the ABI3 configuration.

## Python Git dependency

Installing from Git builds the native extension and therefore requires a Rust toolchain:

```bash
uv add git+https://github.com/burakssen/crcc
```

For reproducibility, append `@<full-commit-sha>` to the Git URL. The equivalent unpinned dependency declaration is:

```toml
[project]
dependencies = ["crcc @ git+https://github.com/burakssen/crcc"]
```

## Rust dependency

Select the backend features needed by your application:

```toml
[dependencies]
crcc = { git = "https://github.com/burakssen/crcc", default-features = false, features = ["parry"] }
geo = "0.32"
```

The default Cargo features enable `parry`, `rhusics`, and `collide`. Add `rayon` to enable Rust batch-query APIs. See [Cargo features](../reference/rust.md#cargo-features) and the [quick start](quick-start.md).

For a local checkout, replace `git = ...` with `path = "/path/to/crcc"`. For a reproducible Git dependency, add `rev = "<full-commit-sha>"`. There is no registry `cargo add crcc` installation path for this project.

## Check the installation

Run the [quick start](quick-start.md). If `CollisionBackend` cannot be imported, check that this repository's CRCC package is installed rather than the unrelated PyPI package. After modifying Rust bindings in a checkout, rebuild with Maturin as described in [Building and testing](../development/building-and-testing.md).
