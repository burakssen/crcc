# Extending geometry and queries

Geometry is a closed enum integrated across adapters, not a runtime plugin interface. Adding a variant requires coordinated source changes.

## Add a geometry type

1. Define its domain wrapper and checked constructor in `src/collision_object/simple.rs`. Specify local-coordinate meaning, emptiness, degeneracy, finite/derived-value validation, and errors.
2. Add it to `SimpleCollisionObject` and the `SweptArea` dispatch. Implement conservative bounds for adjacent poses, including rotation and unrepresentable calculations.
3. Add regular `CollisionObject` factories/exports where appropriate. Decide how merging, empty/full-space normalization, inspection, and equality should behave.
4. Extend each enabled backend's `simple.rs` conversion and collision/continuous handling. If unsupported, make that error explicit rather than silently treating the shape as empty.
5. Extend shared distance in `collision_object/distance.rs` and any backend-native distance path. Native trait distance and runtime fallback are different extension points.
6. Expose it through [Python bindings](python-bindings.md) if intended; add CommonRoad conversion only when an actual matching model type exists.
7. Test valid/invalid construction, boundary contact, compound expansion, poses, swept bounds, and pair/scene queries. Record intended backend differences instead of imposing false parity.

Use existing circles, rectangles, and polygons as examples. Adding a shape can change triangulation and conservative-motion cost; add a bounded benchmark workload and update [the supported matrix](../reference/supported-features.md).

## Add a query

Start from the actual layers: `EngineCollisionObject`, runtime free-function dispatch in `engine/mod.rs`, `CollisionObject` pair methods, then typed and selected scene methods where relevant. Define its input scope, support/error semantics, contact convention, and return value before implementing it. Queries that need different geometry or motion information may require changing the trait; there is no generic contact/manifold API waiting to be enabled.

For scene methods, check ordering, active-time selection, interval attribution, empty trajectories, prepared dispatch, and both batch paths. For Python, specify whole-call versus per-entry failure behavior. Add public-API tests, executable examples, Rustdoc and reference entries, then use [the validation matrix](building-and-testing.md#test-and-lint).
