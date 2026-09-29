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
- The shared distance fallback evaluates half-space pairs analytically; Parry's native half-space pair distance can be unsupported. All three adapters use $`10^{-9}`$ slack for analytic half-space pair collision. Rhusics and Collide also use it for half-space/finite contact tests.
- Rhusics and Collide have explicit conservative handling for moving half-spaces. Do not assume Parry returns the same result for a crossing between clear endpoints; validate that case against the selected engine and version.
- Continuous positives may be conservative, particularly for rotation, half-spaces, and unresolved Collide intervals.
- Collision results on dynamic queries report the first selected time; a between-step hit is attributed to the interval's start.

Choose a backend by testing representative shapes and edge cases from the application's workload. Do not assume that backend names, enum integer values, or raw benchmark rankings are stable compatibility contracts. The [backend-selection guide](../guides/pair-queries.md#select-a-backend) shows the API; [engine internals](../architecture/backends-and-bindings.md) describes the implementation boundary.

## Conversion and query support

Parry uses native compounds and decomposes complex polygons via triangulation/mesh processing with triangle fallbacks. A failed conversion can be stored as an invalid representation and raise `Unsupported` only on use. Some upstream compound dispatch paths suppress unsupported child-pair results; CRCC's analytic half-space-pair intersection addresses a specific case, not every possible dispatch limitation.

Rhusics and Collide decompose complex polygons into triangles. Rhusics uses complex GJK for finite intersection. Collide builds a finite collider set with bounding-sphere candidate filtering; candidate overlap still needs a collision test. Infinite components are handled separately.

`EngineCollisionObject::distance_at` is implemented by Parry only. Calling this trait method directly on Rhusics/Collide representations returns `Unsupported`. Their **public domain/runtime** distance API instead uses CRCC's shared geometry calculation and does not require typed backend conversion.

## Continuous implementation details

- **Parry:** endpoint checks, swept-AABB rejection, an analytic eligible single-circle pair, otherwise nonlinear shape casts on normalized time `[0, 1]`. Moving half-space support is operation-dependent.
- **Rhusics:** endpoint checks and finite translational time-of-impact; any rotation uses radial motion-bound overlap. Motion involving a half-space can return a conservative positive immediately after clear endpoint checks.
- **Collide:** analytic eligible circle pairs; otherwise eight sampling intervals and recursive candidate subdivision. Unresolved overlap at depth `10` or interval width $`10^{-9}`$ returns a possible collision. These are implementation constants, not configurable public parameters.

The default backend priority is Parry, then Rhusics, then Collide, depending on enabled features. Disabled `CollisionEngine` variants still exist in Rust but querying them returns `Unsupported`. Python extension builds require at least one backend.
