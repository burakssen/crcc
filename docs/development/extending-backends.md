# Extending backends

A built-in backend is a feature-gated adapter from CRCC domain geometry to a concrete representation. Keep conversion and algorithms under `src/collision_checker/engine/<backend>/`.

For a Rust-only typed integration, an application can define its own representation `E`, implement `From<CollisionObject>` and `EngineCollisionObject`, and use `builder.build::<E>()` without adding a runtime enum variant. Integrating a backend into `SelectedCollisionChecker` and Python requires the coordinated changes below; there is no runtime plugin loader.

## Implementation boundary

1. Add the optional backend dependency and Cargo feature in `Cargo.toml`.
2. Add the module behind `#[cfg(feature = "<backend>")]` in `src/collision_checker/engine/mod.rs`.
3. Implement a representation convertible from `CollisionObject` and implement `EngineCollisionObject` for discrete and continuous queries. Implement `distance_at` only when the backend supports it; the trait's default reports `CrccError::Unsupported`.
4. Add the runtime `CollisionEngine` variant, default-selection logic, and dispatch branches for pair queries.
5. Extend `SelectedCollisionCheckerInner`, scene construction, prepared-query storage/dispatch, and Python backend exposure when adding a selectable engine.
6. Add feature-gated tests for construction, supported and unsupported geometry, contact boundaries, continuous behavior, distance, and scene queries.

Use the existing backend modules as examples: Parry maps backend errors into `CrccError`; Rhusics and Collide use their collision operations with analytic half-space handling. Feature combinations matter: CI tests no backend, each backend alone, and all features together.

Conversion is infallible at the trait boundary. Define how conversion failures are retained/reported on later use; never silently drop unsupported geometry. Decide contact tolerances, infinite geometry behavior, interpolation, conservative fallbacks, and distance coverage explicitly. Runtime distance dispatch can use a domain fallback independently of typed `distance_at`; wire the intended path deliberately.

Replacing a built-in backend also requires checking its dependency version/feature, representation conversion, runtime default-selection order, prepared-query variants, benchmark selectors/workloads and Python package build features.

## Validate the integration

Run the new feature configuration directly, then the full feature matrix:

```bash
cargo test --locked --no-default-features --features <backend>
cargo test --locked --all-features
uv run --frozen pytest -q
uv run --frozen pyright
```

Add parity assertions only for behavior the APIs intentionally share. Record backend-specific tangency or conservative CCD behavior as explicit expectations rather than forcing artificial equality. If the backend is exposed to Python, verify the `.pyi` signatures and CommonRoad/build flows remain consistent. See [backend abstraction](../architecture/backends-and-bindings.md) and [behavioral differences](../concepts/backends.md).
