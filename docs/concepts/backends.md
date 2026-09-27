# Backends

CRCC exposes one geometry and query model over three optional collision backends: Parry, Rhusics, and Collide. Python builds all three in the distributed extension and selects one at checker construction or on a pair query. Rust selects the backend at compile time with Cargo features, then can use either typed or runtime-selected checker APIs.

The APIs are shared; algorithm coverage and edge semantics are not identical. Exact tangency, half-space behavior, continuous queries, and unsupported shape combinations can differ. The following is a summary of the current implementation, not a promise of feature parity.

| Backend | Discrete queries | Continuous queries | Distance | Notes |
| --- | --- | --- | --- | --- |
| Parry | Parry shape queries | Parry nonlinear casts plus special cases | Native Parry distance | Compound/complex geometry is converted to Parry shapes. Some unsupported shape pairs return an error. |
| Rhusics | GJK for finite geometry; analytic half-spaces | Native time of impact for finite translation; conservative bounds for rotation; moving half-space queries can return conservative positives | Shared geometric fallback | Exact circle tangency follows Rhusics GJK semantics and can differ from Parry and Collide. |
| Collide | Convex/support-map queries, finite-component broad phase, and analytic half-spaces | Circle-motion special cases plus conservative recursive interval checks | Shared geometric fallback | If a broad-phase interval remains unresolved at its depth/tolerance limit, it is treated as a possible collision. |

All backends support the CRCC shapes through backend conversion, but that does not imply that every operation on every shape pair has an exact native implementation. Complex polygons are decomposed during conversion; half-spaces require backend-specific handling. An unsupported operation returns an error rather than a collision-free result.

## Behavior that matters

- Exact circle tangency is collision for Parry and Collide; Rhusics GJK treats the tested tangent pair as non-colliding.
- Parry uses native distance. Rhusics and Collide route runtime distance calls through CRCC's shared geometric calculation. Distance is non-negative and clamped to zero for overlap/contact, but near-contact magnitudes may differ slightly.
- The shared distance fallback evaluates half-space pairs analytically; Parry's native half-space pair distance can be unsupported. Rhusics and Collide also apply a `1e-9` slack in half-space contact tests, so near-boundary results can differ.
- Rhusics and Collide have explicit conservative handling for moving half-spaces. Do not assume Parry returns the same result for a crossing between clear endpoints; validate that case against the selected engine and version.
- Continuous positives may be conservative, particularly for rotation, half-spaces, and unresolved Collide intervals.
- Collision results on dynamic queries report the first selected time; a between-step hit is attributed to the interval's start.

Choose a backend by testing representative shapes and edge cases from the application's workload. Do not assume that backend names, enum integer values, or raw benchmark rankings are stable compatibility contracts. The [backend-selection guide](../guides/pair-queries.md#select-a-backend) shows the API; [engine internals](../architecture/backends-and-bindings.md) describes the implementation boundary.
