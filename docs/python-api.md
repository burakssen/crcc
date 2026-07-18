# Python API Reference

This reference summarizes the public Python surface. Start with the [Python guide](python-guide.md) for task-oriented examples.

All core classes are exported from `crcc`. Angles are in radians. Invalid geometry, inverted time bounds, and unsupported operations raise `ValueError`.

## Geometry and poses

### `Pose`

`Pose(translation: tuple[float, float], angle: float)` represents a rigid 2D transform.

| Member | Result | Purpose |
| --- | --- | --- |
| `Pose.identity()` | `Pose` | Identity transform. |
| `Pose.from_translation(translation)` | `Pose` | Translation-only pose. |
| `Pose.from_rotation(angle)` | `Pose` | Rotation-only pose. |
| `translation` | `tuple[float, float]` | Translation component. |
| `rotation` | `float` | Rotation in radians. |
| `compose(other)` / `*` | `Pose` | Compose two poses. |

### `CollisionObject`

All shapes inherit from `CollisionObject`.

| Method | Purpose |
| --- | --- |
| `collides(other, pos_self=..., pos_other=..., engine=...)` | Discrete pair collision. |
| `collides_continuous(start_self, end_self, other, start_other, end_other, engine=...)` | Conservative collision query over a motion interval. |
| `distance(other, pos_self=..., pos_other=..., engine=...)` | Euclidean separation distance. |
| `merge(other)` | Union of two objects. |
| `CollisionObject.merge_all(objects)` | Union of an iterable of objects. |

### Shapes

| Class | Constructor |
| --- | --- |
| `Circle` | `Circle(radius, center=(0.0, 0.0))` |
| `Rectangle` | `Rectangle(length, width, orientation=0.0, center=(0.0, 0.0))` |
| `Triangle` | `Triangle(a, b, c)` |
| `Polygon` | `Polygon(exterior, interiors=None)` |
| `HalfSpace` | `HalfSpace(outward_normal, offset=0.0)` |
| `Compound` | `Compound(collision_objects)` |
| `Empty` | `Empty()`; never collides. |
| `FullSpace` | `FullSpace()`; occupies the entire plane. |

`HalfSpace.from_points(p1, p2)` constructs the region to the right of a directed line. `HalfSpace.from_coeffs(a, b, c)` constructs the region satisfying `a*x + b*y <= c`.

## Engines and results

### `CollisionEngine`

Available values are `Parry` (default), `Rhusics`, and `Collide`.

### `CollisionStatus`

| Member | Type | Meaning |
| --- | --- | --- |
| `collides` | `bool` | Whether a collision was reported. |
| `time_step` | `int | None` | First dynamic collision step, when applicable. |
| `str(status)` | `str` | Human-readable status. |

## Scene construction

### `CollisionCheckerBuilder`

`CollisionCheckerBuilder(engine=CollisionEngine.Parry)` creates an empty builder.

| Method | Purpose |
| --- | --- |
| `with_engine(engine)` | Select the backend. |
| `with_static_obstacle(obj)` | Add fixed geometry. |
| `with_dynamic_obstacle(obstacle)` | Add a trajectory. |
| `with_road_boundary(lanelets)` | Add the region outside lanelet polygons. |
| `build()` | Return an immutable `CollisionChecker`. |

### `CollisionChecker`

Python time bounds are inclusive.

| Member | Purpose |
| --- | --- |
| `engine` | Selected `CollisionEngine`. |
| `collides_static(query, position=None, min_time=None, max_time=None)` | Check a positioned shape against the scene. |
| `collides_dynamic(obstacle, min_time=None, max_time=None)` | Check a trajectory against the scene. |
| `collides_static_batch(queries, min_time=None, max_time=None)` | Ordered batch of positioned shape queries. |
| `collides_dynamic_batch(obstacles, min_time=None, max_time=None)` | Ordered batch of trajectory queries. |

Batch methods run small inputs sequentially and larger inputs through Rayon.

## Dynamic obstacles

`DynamicObstacle(shape, positions, time_offset)` uses one shape across a pose sequence.

`DynamicObstacle.from_time_variant(obstacles, time_offset=0, positions=None)` accepts different geometry at each step.

## CommonRoad conversion

The `crcc.commonroad` module provides Python-only adapters.

| Function | Purpose |
| --- | --- |
| `scenario_builder(scenario, builder=None)` | Add a scenario's road boundary and obstacles to a builder. |
| `add_static_obstacle(builder, obstacle)` | Add one CommonRoad static obstacle. |
| `add_dynamic_obstacle(builder, obstacle)` | Add one CommonRoad dynamic obstacle. |
| `add_road_boundary(builder, lanelet_network)` | Add the network boundary. |
| `road_boundary(lanelet_network)` | Return the boundary collision object. |
| `to_dynamic_obstacle(obstacle)` | Convert a CommonRoad trajectory. |
| `to_occupancy(occupancy)` | Convert a positioned occupancy. |
| `to_shape(shape)` | Convert obstacle geometry. |
| `to_polygon(polygon)` | Convert a Shapely polygon. |
| `from_shapely(geometry)` | Convert an empty, polygon, or multipolygon geometry. |
| `to_pose(state)` | Convert a CommonRoad state. |
