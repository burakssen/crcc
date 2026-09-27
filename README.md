# CRCC

CRCC is a 2D collision-query library for Rust and Python. It provides validated geometry, pair and scene queries, conservative continuous collision checks, and optional Parry, Rhusics, and Collide backends. Python users can convert CommonRoad scenarios with `crcc.commonroad`.

CRCC reports overlap; it does not resolve contacts or advance simulation state. Continuous-query positives can be conservative, and backend edge semantics differ. See the [backend guide](https://burakssen.com/crcc/concepts/backends/).

## Install

CRCC is not published to PyPI or crates.io. Install the Python package from GitHub with `uv` (a Rust toolchain is required to build the native extension):

```bash
uv add git+https://github.com/burakssen/crcc
```

For Rust, select the required backend features:

```toml
[dependencies]
crcc = { git = "https://github.com/burakssen/crcc", default-features = false, features = ["parry"] }
geo = "0.32"
```

## Example

```python
from crcc import Circle, CollisionBackend, CollisionCheckerBuilder, Pose, Rectangle

checker = (
    CollisionCheckerBuilder(backend=CollisionBackend.Parry)
    .add_static_obstacle(Rectangle(0.25, 3.0))
    .build()
)
status = checker.collides_static(Circle(0.5), position=Pose.from_translation((0.5, 0.0)))
assert status.collides
```

## Documentation and development

- [Documentation](https://burakssen.com/crcc/): [installation](https://burakssen.com/crcc/getting-started/installation/), [quick start](https://burakssen.com/crcc/getting-started/quick-start/), [Python API](https://burakssen.com/crcc/reference/python/), and [Rust API](https://burakssen.com/crcc/reference/rust/).
- [CommonRoad integration](https://burakssen.com/crcc/guides/commonroad/) and [architecture](https://burakssen.com/crcc/architecture/overview/).
- [Build and test](https://burakssen.com/crcc/development/building-and-testing/), [benchmarking](https://burakssen.com/crcc/development/benchmarking/), and [contributing](https://burakssen.com/crcc/development/contributing/).

The repository's `main.py` tutorials, playground, benchmarks, and scenarios are development assets; they are not installed as a `crcc` command. See the [development guide](https://burakssen.com/crcc/development/building-and-testing/) for source setup.
