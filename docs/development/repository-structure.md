# Repository structure

CRCC is one Rust crate with a mixed Python package built by Maturin. There is no Cargo workspace or separately packaged CommonRoad Rust crate.

| Path | Responsibility |
| --- | --- |
| `Cargo.toml`, `Cargo.lock` | Crate, optional backend features, binaries, locked Rust dependencies, Clippy policy |
| `src/lib.rs`, `src/error.rs` | Root API exports and recoverable errors |
| `src/collision_object/simple.rs` | Domain shapes, validation, polygon sanitation, swept geometry |
| `src/collision_object/mod.rs` | Compounds, construction, pair queries, geometry inspection |
| `src/collision_object/dynamic.rs` | Owned sample trajectories, cached interval bounds, representation conversion |
| `src/collision_object/distance.rs` | Shared runtime Rhusics/Collide distance calculations |
| `src/collision_checker/builder.rs` | Static merging, scene conversion, active-time set, approximate road boundary |
| `src/collision_checker/mod.rs` | Typed/runtime scene queries, prepared objects, batches and status attribution |
| `src/collision_checker/ccd_collider.rs` | Fixed/time-varying interval query representation |
| `src/collision_checker/engine/` | Trait/runtime dispatch and Parry, Rhusics, Collide adapters |
| `src/time/` | Signed time indices, checked/saturating arithmetic, range iteration |
| `src/python/` | PyO3 native classes/submodules and exception mapping |
| `python/crcc/` | Public re-exports, builder facade, CommonRoad adapter, `.pyi` signatures |
| `pyproject.toml`, `uv.lock` | Python package/build configuration, development dependencies, locked environment |
| `tests/` | Rust public API and Python API/behavior/examples/reporting/documentation tests |
| Inline Rust `#[cfg(test)]` modules | Geometry, backend, time and query regression tests |
| `examples/`, `main.py` | Importable Python tutorials and checkout CLI dispatcher |
| `tools/playground.py` | Matplotlib interactive scene/query viewer |
| `tools/benchmark/`, `src/bin/` | Python reporting/measurement pipeline and Rust benchmark binaries |
| `scenarios/` | 18 Git-LFS CommonRoad XML datasets |
| `docs/`, `zensical.toml` | Handwritten site and navigation/extensions |
| `.github/workflows/`, `.gitlab-ci.yml` | Validation, documentation, packaging and GitHub release/deployment jobs |
| `.pre-commit-config.yaml` | Fixing format/lint/file-hygiene hooks |

`target/`, `site/`, environments, benchmark outputs, and caches are generated local artifacts. They are not architectural modules or source API. Start with [Architecture](../architecture/overview.md), then [Building and testing](building-and-testing.md).
