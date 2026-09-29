# Building and testing

The repository builds the Rust library and its Python extension with Cargo and Maturin. CI runs Python checks on Python 3.10 and 3.13, Rust feature-configuration tests, package smoke checks, and a strict documentation build.

## Prerequisites and checkout

See [Installation](../getting-started/installation.md) for toolchain/platform prerequisites. From a checkout, materialize scenarios and synchronize the locked Python environment:

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

`uv sync --frozen` installs both the local project and development dependencies. Maturin is the native build backend; `develop` is a debug/development build unless `--release` is passed. Cargo builds alone do not install the Python extension.

## Build Rust

```bash
cargo build --locked
cargo build --release --locked
cargo build --release --locked --no-default-features --features parry
```

The repository is one crate, not a Cargo workspace. Default features enable all three backends; `python_bindings` adds PyO3 and Rayon. Native benchmark binaries have their own required features. See [the feature table](../reference/rust.md#cargo-features).

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

The hooks **modify files**: Ruff runs with `--fix`, and formatters/file hygiene rewrite inputs. Check-only equivalents are:

```bash
uv run --frozen ruff check .
uv run --frozen ruff format --check .
cargo fmt --all -- --check
cargo check --locked
cargo clippy --locked
```

CI's default Cargo lint hooks are not the same as all-feature/all-target lint coverage. For API changes, test the relevant feature boundary too. Python tests cover three backends and include deterministic randomized cases, not an external fuzzing framework. Focused checks:

```bash
uv run --frozen pytest -q tests/test_public_api.py tests/test_dynamic.py tests/test_commonroad.py
uv run --frozen pytest -q tests/test_documentation.py tests/test_examples.py
cargo test --locked --test public_api --all-features
cargo test --locked --doc --no-default-features --features parry,rhusics,collide,rayon
uv run --frozen python tools/check_documentation.py
```

The documentation checker executes actual Python fences and compiles/runs actual Rust fences, including the indented quick-start tabs. Signature/pseudocode blocks use `text`; executable blocks must be self-contained. See [documentation maintenance](contributing.md#documentation-maintenance).

## Build the documentation site

From the repository root:

```bash
uvx --from zensical==0.0.65 zensical serve
uvx --from zensical==0.0.65 zensical build --strict
```

The preview server watches Markdown/configuration; the strict build writes `site/`. Navigation and Markdown extensions are configured in [`zensical.toml`](https://github.com/burakssen/crcc/blob/main/zensical.toml). A plain site build needs no installed extension or scenario data.

Generate Rust API documentation separately:

```bash
cargo doc --locked --lib --no-deps --no-default-features --features parry,rhusics,collide,rayon
```

Open `target/doc/crcc/index.html` locally. After the Zensical build, docs CI copies the **whole** `target/doc/` tree to `site/rustdoc/` to retain shared assets and dependency links; GitHub Pages deploys the combined site. Local preview of Markdown does not automatically serve Rustdoc. Python reference signatures are maintained from public wrappers/native stubs and checked against the built extension; no runtime-import API generator is required by Zensical.

To assemble/check the combined output locally (after both builds):

```bash
cp -R target/doc/. site/rustdoc
python tools/check_documentation.py --site
```

The copy includes shared Rustdoc assets and does not nest the tree on repeated assembly. The rendered-link check verifies local targets and anchors, including the published Rustdoc destination.

## Build and smoke-test a wheel

Build the wheel from the source distribution, then install it outside the repository environment:

```bash
uv build --sdist
uv build --wheel dist/*.tar.gz
uv venv /tmp/crcc-smoke
uv pip install --python /tmp/crcc-smoke/bin/python dist/*.whl
/tmp/crcc-smoke/bin/python -c "from crcc import Circle; assert Circle(1.0).collides(Circle(0.5))"
```

Use a new environment path and a clean `dist/` containing only the intended artifacts; the glob recipes assume one version. These paths are POSIX examples; Windows virtual environments use `Scripts/python.exe`. The package has no PyPI or crates.io publishing workflow. Tagged GitHub releases attach an sdist and platform wheels. GitHub routine behavioral tests are Linux; additional wheel jobs smoke-test installation. GitLab validates Linux and builds a documentation artifact. See [Benchmarking](benchmarking.md) and [repository structure](repository-structure.md).
