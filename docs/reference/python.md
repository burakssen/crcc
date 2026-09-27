# Python API reference

The supported Python package surface is exported by `crcc` and `crcc.commonroad`. Native type signatures are also maintained in the package's `.pyi` files. This page covers common entry points; see [guides](../guides/pair-queries.md) for runnable workflows.

## Top-level exports

```text
Circle CollisionBackend CollisionChecker CollisionCheckerBuilder
CollisionObject CollisionStatus Compound DynamicObstacle Empty FullSpace
HalfSpace Polygon Pose PreparedDynamicQuery PreparedStaticQuery Rectangle
Triangle road_boundary
```

`CollisionEngine` is an alias of `CollisionBackend`. Import CommonRoad adapters from `crcc.commonroad`.

## Geometry and poses

| Type | Constructor |
| --- | --- |
| `Circle` | `Circle(radius, center=(0.0, 0.0))` |
| `Rectangle` | `Rectangle(length, width, orientation=0.0, center=(0.0, 0.0))` |
| `Triangle` | `Triangle(point_a, point_b, point_c)` |
| `Polygon` | `Polygon(exterior, interiors=None)` |
| `HalfSpace` | `HalfSpace(outward_normal, offset=0.0)` |
| `Compound` | `Compound(collision_objects)` |
| `Empty`, `FullSpace` | No arguments |

Angles are counter-clockwise radians. Shape coordinates and centers are local. Constructors reject invalid geometry with `ValueError`.

`Pose(translation, angle)` represents a finite rigid transform. Helpers are `Pose.identity()`, `Pose.from_translation(...)`, and `Pose.from_rotation(...)`; `.compose(other)` and `pose * other` compose transforms, applying the right operand first. `.translation` and `.rotation` return the components.

`CollisionObject` is the queryable base type. Its operations are:

```python
obj.collides(other, pos_self=Pose.identity(), pos_other=Pose.identity(), backend=None)
obj.collides_continuous(start_pos_self, end_pos_self, other,
                        start_pos_other, end_pos_other, backend=None)
obj.distance(other, pos_self=Pose.identity(), pos_other=Pose.identity(), backend=None)
obj.merge(other)
CollisionObject.merge_all(objects)
```

The query methods return `bool`, `bool`, and non-negative `float` respectively; native failures raise `ValueError`. `engine=` remains a deprecated alias for `backend=` on pair methods.

## Backend and builder

Select `CollisionBackend.Parry`, `.Rhusics`, or `.Collide`. The distributed extension enables all three; the default is Parry.

```python
builder = CollisionCheckerBuilder(backend=None)
builder.add_static_obstacle(shape)
builder.add_dynamic_obstacle(trajectory)
checker = builder.build()
```

`backend=None` selects the compiled default. The builder methods mutate and return the builder for chaining. `build()` returns an immutable native checker. Deprecated aliases `with_static_obstacle`, `with_dynamic_obstacle`, `with_engine`, and `with_road_boundary` remain available; use the corresponding `add_*` methods, `backend=`, and `add_static_obstacle(road_boundary(...))` instead.

## Scene queries

`CollisionChecker` exposes `.backend`, `.prepare_static(shape)`, and `.prepare_dynamic(trajectory)`.

```python
checker.collides_static(query, position=None, min_time=None, max_time=None)
checker.collides_dynamic(query, min_time=None, max_time=None)
checker.collides_static_batch(queries, min_time=None, max_time=None, parallel=False)
checker.collides_dynamic_batch(queries, min_time=None, max_time=None, parallel=False)
```

Static queries accept either a `CollisionObject` or `PreparedStaticQuery`; dynamic queries accept `DynamicObstacle` or `PreparedDynamicQuery`. A prepared query is tied to its checker's backend. Static scene geometry is checked irrespective of the time window. Python time bounds are inclusive, and invalid/inverted bounds raise `ValueError`.

Batch result order matches input order. `parallel=True` requests Rayon; the default is sequential. Batch entries may mix raw and prepared objects. Older prepared-query method names and `par_*` aliases are deprecated; see the [stub file](https://github.com/burakssen/crcc/blob/main/python/crcc/_core/collision_checker.pyi) for the exact overloads retained by the current package.

`CollisionStatus` results expose `.collides` and `.time_step`. Static hits have `time_step is None`; dynamic hits provide the sample or interval start time.

## DynamicObstacle

```python
DynamicObstacle(shape, positions, time_offset)
DynamicObstacle.from_time_variant(obstacles, time_offset=0, positions=None)
```

For fixed geometry, each pose is active at `time_offset + index`. For varying geometry, shape and pose counts must match; omitted poses are identities. Empty geometry marks missing occupancy and suppresses its adjacent motion intervals.

## CommonRoad module

`crcc.commonroad` depends on `commonroad-io` and exposes:

```text
scenario_builder add_static_obstacle add_dynamic_obstacle add_road_boundary
road_boundary from_dynamic_obstacle from_occupancy from_shape from_polygon
from_shapely from_pose
```

`scenario_builder(scenario, builder=None)` adds lanelet boundary geometry (when lanelets exist), static obstacles, and dynamic obstacles. `from_shapely` accepts empty geometry, Polygon, and MultiPolygon; other non-empty geometry types raise `ValueError`. CommonRoad time steps must be exact integers. See the [CommonRoad guide](../guides/commonroad.md).

## Exceptions

CRCC errors become Python `ValueError`. Python argument conversion can also raise `TypeError` or `OverflowError`. Unsupported geometry/query pairs do not return `False`; see [errors and limits](errors-and-limits.md).
