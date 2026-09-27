# Building and testing

The repository builds the Rust library and its Python extension with Cargo and Maturin. CI runs Python checks on Python 3.10 and 3.13, Rust feature-configuration tests, package smoke checks, and a strict documentation build.

## Prerequisites and checkout

Install Git, Git LFS, a recent stable Rust toolchain, Python 3.10+, and `uv`. Then materialize scenarios and synchronize the locked Python environment:

```bash
git clone https://github.com/burakssen/crcc.git
cd crcc
git lfs install
git lfs pull
uv sync --frozen
```

Scenario XML files use Git LFS. If a parser reports XML errors at the first line and the file contains an LFS pointer, run `git lfs pull`.

After changing Rust or PyO3 code, rebuild the extension:

```bash
uv run --frozen --with "maturin>=1.0,<2.0" maturin develop
```

## Test and lint

Run the Python validation commands:

```bash
uv run --frozen pre-commit run --all-files --show-diff-on-failure
uv run --frozen pyright
uv run --frozen pytest -q
```

Exercise Rust's supported feature boundaries:

```bash
cargo test --locked --no-default-features
cargo test --locked --no-default-features --features parry
cargo test --locked --no-default-features --features rhusics
cargo test --locked --no-default-features --features collide
cargo test --locked --all-features
```

`pre-commit` includes file hygiene, YAML checks, Ruff, Rust formatting, Cargo checks, and Clippy. Review any formatter changes before rerunning.

## Build the documentation site

From the repository root:

```bash
uvx --from zensical==0.0.65 zensical serve
uvx --from zensical==0.0.65 zensical build --strict
```

The preview server watches the Markdown and configuration. A successful strict build writes the static site to `site/`; CI deploys that directory from the main branch. Navigation and Markdown extensions are configured in [`zensical.toml`](../../zensical.toml).

## Build and smoke-test a wheel

Build the wheel from the source distribution, then install it outside the repository environment:

```bash
uv build --sdist
uv build --wheel dist/*.tar.gz
uv venv --clear /tmp/crcc-smoke
uv pip install --python /tmp/crcc-smoke/bin/python dist/*.whl
/tmp/crcc-smoke/bin/python -c "import crcc"
```

The package has no PyPI or crates.io publishing workflow. Tagged GitHub releases attach an sdist and platform wheels. See the [benchmark guide](benchmarking.md) for research commands and limits.
