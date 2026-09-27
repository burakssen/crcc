# CRCC

CRCC is a Rust library with Python bindings for 2D collision queries. It models validated geometry, checks object pairs and immutable scenes, and can convert CommonRoad scenarios through its Python adapter.

CRCC answers whether geometry overlaps; it does not resolve contacts, advance a simulation, or return contact manifolds.

## Start here

- [Install CRCC](getting-started/installation.md) from a source checkout, wheel, or Rust dependency.
- [Run the quick start](getting-started/quick-start.md) for Python and Rust.
- [Choose a backend](concepts/backends.md) and understand where backend semantics differ.
- [Convert a CommonRoad scenario](guides/commonroad.md) with the Python adapter.

## Capabilities

- Circles, rectangles, triangles, polygons with holes, half-spaces, empty/full space, and compounds.
- Discrete pair queries, separation distance, and conservative continuous collision queries.
- Immutable checkers with static geometry, fixed-shape or time-varying trajectories, prepared queries, and ordered batches.
- Parry, Rhusics, and Collide backends, selected at runtime by Python or either statically or dynamically from Rust.
- CommonRoad shape, occupancy, prediction, obstacle, and lanelet-boundary conversion in Python.

See [concepts](concepts/geometry-and-poses.md) for the data model, [guides](guides/pair-queries.md) for task-based examples, and the [API reference](reference/python.md) or [Rust reference](reference/rust.md) for signatures.

## For contributors

[Build and test the repository](development/building-and-testing.md), explore the [benchmark harness](development/benchmarking.md), or learn how to [add a backend](development/extending-backends.md).
