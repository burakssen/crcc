# CRCC

CRCC is a Rust library with Python bindings for 2D collision queries. Use it to check a vehicle, robot, or other occupied geometry against another object or a scene containing static and moving obstacles. Its Python adapter converts CommonRoad models into ordinary CRCC geometry and trajectories.

CRCC answers whether geometry overlaps; it does not resolve contacts, advance a simulation, or return contact manifolds.

```python
from crcc import Circle, CollisionCheckerBuilder, Pose, Rectangle

checker = CollisionCheckerBuilder().add_static_obstacle(Rectangle(2.0, 2.0)).build()
assert checker.collides_static(Circle(0.5), Pose.identity()).collides
assert not checker.collides_static(Circle(0.5), Pose.from_translation((4.0, 0.0))).collides
```

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

Rust owns geometry, trajectory bounds, scene queries, and backend conversion. Python exposes the runtime-selected checker through PyO3; Rust also offers typed backend dispatch. See the [architecture](architecture/overview.md), [supported features](reference/supported-features.md), and [limitations](reference/limitations.md). Continuous positives can be conservative, and boundary contact differs by backend.

## For contributors

[Build and test the repository](development/building-and-testing.md), explore the [benchmark harness](development/benchmarking.md), or learn how to [add a backend](development/extending-backends.md).
