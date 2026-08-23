# Python API Reference

This page documents the supported Python surface exported by `crcc`, `crcc.collision_checker`, and `crcc.commonroad`. Start with the [Python usage guide](python-guide.md) for complete workflows.

Angles are radians. Native `CrccError` values become Python `ValueError`. Python argument conversion may additionally raise `TypeError` or `OverflowError`.

## Root Exports

The following names are supported from `crcc`:

```text
Circle CollisionBackend CollisionChecker CollisionCheckerBuilder
CollisionEngine CollisionObject CollisionStatus Compound DynamicObstacle
Empty FullSpace HalfSpace Polygon Pose PreparedDynamicQuery
PreparedStaticQuery Rectangle Triangle road_boundary
```

`CollisionEngine` is a deprecated alias of `CollisionBackend`.


Import CommonRoad adapters from `crcc.commonroad`, not from the root package.

## CommonRoad Conversion (`crcc.commonroad`)

The `crcc.commonroad` module provides dedicated adapters to convert CommonRoad benchmark scenarios and geometric models into `crcc` collision objects and checkers. It requires `commonroad-io >= 2026.1`.

### Scenario and Builder Helpers

```python
scenario_builder(
    scenario: Scenario,
    builder: CollisionCheckerBuilder | None = None,
) -> CollisionCheckerBuilder

add_static_obstacle(builder, static_obstacle) -> CollisionCheckerBuilder
add_dynamic_obstacle(builder, dynamic_obstacle) -> CollisionCheckerBuilder
add_road_boundary(builder, lanelet_network) -> CollisionCheckerBuilder
```

`scenario_builder` adds road boundary, static obstacles, and dynamic obstacles. `add_road_boundary` skips an empty network.

### Model Conversion Helpers

```python
from_dynamic_obstacle(dynamic_obstacle) -> DynamicObstacle
from_occupancy(occupancy: Occupancy) -> CollisionObject
from_shape(shape: ObstacleShape) -> CollisionObject
from_polygon(polygon: shapely.geometry.Polygon) -> CollisionObject
from_shapely(geometry: BaseGeometry) -> CollisionObject
from_pose(state: TraceState) -> Pose
road_boundary(lanelet_network) -> CollisionObject
```

`from_shapely` accepts empty geometry, `Polygon`, and `MultiPolygon`; other non-empty geometry types raise `ValueError`. `from_pose` requires an exact two-dimensional position and orientation.

Trajectory predictions preserve their state timeline. Missing intermediate states become empty occupancy.

The module exposes `ROAD_BOUNDARY_SIMPLIFY_TOLERANCE` and `ROAD_BOUNDARY_MIN_HOLE_AREA` as informational mirrors of native boundary parameters.

## `Pose`

```python
Pose(translation: tuple[float, float], angle: float)
```

A finite rigid transform with counter-clockwise rotation.

| Member | Return | Description |
| --- | --- | --- |
| `Pose.identity()` | `Pose` | Identity transform. |
| `Pose.from_translation(translation)` | `Pose` | Translation-only transform. |
| `Pose.from_rotation(angle)` | `Pose` | Rotation-only transform. |
| `translation` | `tuple[float, float]` | Translation component. |
| `rotation` | `float` | Rotation angle. |
| `compose(other)` | `Pose` | Compose transforms; apply `other` first. |
| `pose * other` | `Pose` | Alias for composition. |

Construction rejects non-finite translation or rotation with `ValueError`.

## `CollisionObject`

Abstract base for queryable geometry. Construct a concrete shape instead of instantiating this class.

### Discrete collision

```python
obj.collides(
    other: CollisionObject,
    pos_self: Pose = Pose.identity(),
    pos_other: Pose = Pose.identity(),
    backend: CollisionBackend | None = None,
    engine: CollisionEngine | None = None,  # Deprecated alias.
) -> bool
```

Applies both poses and asks the selected backend whether occupied sets overlap. `None` selects the compiled default. The deprecated `engine=` keyword still works and warns.

### Continuous collision

```python
obj.collides_continuous(
    start_pos_self: Pose,
    end_pos_self: Pose,
    other: CollisionObject,
    start_pos_other: Pose,
    end_pos_other: Pose,
    backend: CollisionBackend | None = None,
    engine: CollisionEngine | None = None,  # Deprecated alias.
) -> bool
```

Checks endpoints and motion between them. `False` certifies interval separation. `True` can be conservative.

### Distance

```python
obj.distance(
    other: CollisionObject,
    pos_self: Pose = Pose.identity(),
    pos_other: Pose = Pose.identity(),
    backend: CollisionBackend | None = None,
    engine: CollisionEngine | None = None,  # Deprecated alias.
) -> float
```

Returns non-negative set separation, clamped to zero for overlap/contact. Parry uses native distance; Rhusics and Collide use the shared geometric implementation. Empty-set distance is unsupported.

### Union

```python
obj.merge(other: CollisionObject) -> CollisionObject
CollisionObject.merge_all(objects: Sequence[CollisionObject]) -> CollisionObject
```

Returns structural union geometry. Empty children are removed; full space dominates; merging an empty sequence returns empty geometry.

## Shape Classes

### `Circle`

```python
Circle(radius: float, center: tuple[float, float] = (0.0, 0.0))
```

Radius must be finite and strictly positive. Center is local geometry and must be finite.

### `Rectangle`

```python
Rectangle(
    length: float,
    width: float,
    orientation: float = 0.0,
    center: tuple[float, float] = (0.0, 0.0),
)
```

`length` is local x extent and `width` is local y extent. Dimensions must be positive; all values must be finite.

### `Triangle`

```python
Triangle(
    point_a: tuple[float, float],
    point_b: tuple[float, float],
    point_c: tuple[float, float],
)
```

Vertices must be finite and have nonzero area.

### `Polygon`

```python
Polygon(
    exterior: Sequence[tuple[float, float]],
    interiors: Sequence[Sequence[tuple[float, float]]] | None = None,
)
```

Supports convex, non-convex, and holed polygons. Rings must be finite, nondegenerate, and topologically valid. Engines decompose complex polygons during conversion.

### `HalfSpace`

```python
HalfSpace(
    outward_normal: tuple[float, float],
    offset: float = 0.0,
)
```

Represents `outward_normal dot point <= offset`. The normal and offset are normalized together. The normal must be finite and nonzero.

```python
HalfSpace.from_points(
    point_1: tuple[float, float],
    point_2: tuple[float, float],
) -> HalfSpace
```

Returns the region to the right of directed line `point_1 -> point_2`.

```python
HalfSpace.from_coeffs(a: float, b: float, c: float = 0.0) -> HalfSpace
```

Returns `a*x + b*y <= c`.

### `Compound`

```python
Compound(collision_objects: Sequence[CollisionObject])
```

Union of zero or more objects. Zero children produce empty geometry.

### `Empty` and `FullSpace`

```python
Empty()
FullSpace()
```

Empty never collides and has unsupported distance. Full space collides with every non-empty object and has distance zero to non-empty geometry.

## `CollisionBackend`

Runtime selector values:

```python
CollisionBackend.Parry
CollisionBackend.Rhusics
CollisionBackend.Collide
```

The distributed build enables all three and defaults to Parry. Integer equality exists in the current extension but is not a stable serialization contract.

## `CollisionStatus`

Checker queries return these instances; they are not constructed directly. Variants:

```python
NoCollision
CollidesStatic
CollidesDynamic(t: int)
```

| Member | Type | Description |
| --- | --- | --- |
| `collides` | `bool` | Whether a collision was reported. |
| `time_step` | `int | None` | Dynamic sample or interval-start attribution. |
| `str(status)` | `str` | `NoCollision`, `CollidesStatic`, or `CollidesDynamic(t)`. |
| `repr(status)` | `str` | Same readable representation at runtime. |

Statuses support value equality and hashing.

## `DynamicObstacle`

### Fixed shape

```python
DynamicObstacle(
    shape: CollisionObject,
    positions: Sequence[Pose],
    time_offset: int,
)
```

`positions[i]` is active at `time_offset + i`. An empty sequence is valid. Poses must be finite, and the final step must fit signed 32-bit time.

### Varying shape

```python
DynamicObstacle.from_time_variant(
    obstacles: Sequence[CollisionObject],
    time_offset: int = 0,
    positions: Sequence[Pose] | None = None,
) -> DynamicObstacle
```

Shape and pose counts must match. `positions=None` supplies identity poses. An interval touching an empty shape has no occupancy.

## `CollisionCheckerBuilder`

```python
CollisionCheckerBuilder(
    backend: CollisionBackend | None = None,
)
```

`None` selects the compiled default. The backend is chosen once, in the constructor.

| Method | Return | Description |
| --- | --- | --- |
| `add_static_obstacle(query_shape)` | same builder | Add static geometry. |
| `add_dynamic_obstacle(dynamic_obstacle)` | same builder | Add a trajectory. |
| `build()` | `CollisionChecker` | Build immutable native scene. |

The Python wrapper mutates and returns itself for chaining. The core state is cloned during build, so a builder can be reused.

Road boundaries compose from primitives:

```python
builder.add_static_obstacle(road_boundary(lanelets))
```

## `CollisionChecker`

Construct with `CollisionCheckerBuilder.build()`; direct construction is not public.

### Metadata and preparation

```python
checker.backend -> CollisionBackend
checker.prepare_static(query_shape: CollisionObject) -> PreparedStaticQuery
checker.prepare_dynamic(dynamic_obstacle: DynamicObstacle) -> PreparedDynamicQuery
```

Prepared classes cannot be directly constructed. Both expose `.backend`.

### Static query

```python
checker.collides_static(
    query: CollisionObject | PreparedStaticQuery,
    position: Pose | None = None,
    min_time: int | None = None,
    max_time: int | None = None,
) -> CollisionStatus
```

Static scene geometry is always checked first. Time bounds restrict dynamic-scene checks only. Passing a `PreparedStaticQuery` skips repeated geometry conversion; passing anything else raises `TypeError`.

### Dynamic query

```python
checker.collides_dynamic(
    query: DynamicObstacle | PreparedDynamicQuery,
    min_time: int | None = None,
    max_time: int | None = None,
) -> CollisionStatus
```

Dynamic queries check against static and dynamic scene geometry. A hit against static scene geometry is still attributed as `CollidesDynamic(t)`. Passing a `PreparedDynamicQuery` skips repeated trajectory conversion.

### Ordered batches

```python
checker.collides_static_batch(
    queries: Sequence[tuple[CollisionObject | PreparedStaticQuery, Pose]],
    min_time: int | None = None,
    max_time: int | None = None,
) -> list[CollisionStatus]
```

```python
checker.collides_dynamic_batch(
    queries: Sequence[DynamicObstacle | PreparedDynamicQuery],
    min_time: int | None = None,
    max_time: int | None = None,
) -> list[CollisionStatus]
```

Results preserve input order. Empty input returns `[]`. Automatic batching chooses sequential or Rayon execution from estimated work and active worker count; small batches are intentionally kept sequential. Entries may freely mix raw objects with prepared geometry:

```python
prepared = checker.prepare_static(query)
checker.collides_static_batch([
    (query, Pose.identity()),
    (prepared, Pose.from_translation((4.0, 0.0))),
])

prepared_dynamic = checker.prepare_dynamic(dynamic)
checker.collides_dynamic_batch([dynamic, prepared_dynamic])
```

Repeated references to one prepared static query reuse the dedicated one-query/many-poses path; mixed batches share a single workload estimate and Rayon decision without re-converting prepared geometry.

### Deprecated aliases

These names still work but emit `DeprecationWarning`; they will be removed in the next breaking release:

| Deprecated | Replacement |
| --- | --- |
| `CollisionEngine`, `.engine`, `engine=` | `CollisionBackend`, `.backend`, `backend=` |
| `collides_static_prepared(query)` | `collides_static(query)` |
| `collides_static_prepared_batch(query, poses)` | `collides_static_batch([(query, pose), ...])` |
| `collides_dynamic_prepared(query)` | `collides_dynamic(query)` |
| `collides_dynamic_prepared_batch([...])` | `collides_dynamic_batch([...])` |
| `par_static(...)`, `par_dynamic(...)` | `collides_static_batch(...)`, `collides_dynamic_batch(...)` |
| `par_static_threads(...)`, `_collides_static_batch_threads(...)` | removed; thread forcing lives only in `crcc._core._benchmark` for benchmarks |
| `with_engine(engine)` | pass `backend=` to the constructor |
| `with_static_obstacle(o)`, `with_dynamic_obstacle(o)` | `add_static_obstacle(o)`, `add_dynamic_obstacle(o)` |
| `with_road_boundary(lanelets)` | `add_static_obstacle(road_boundary(lanelets))` |

### Time bounds

Python bounds are inclusive. `None` means unbounded. `min_time > max_time` raises `ValueError`. Interval `t -> t+1` runs only if both values are selected.

## Low-Level Road Boundary

```python
from crcc.collision_checker import road_boundary

road_boundary(
    lanelets: Sequence[Sequence[tuple[float, float]]],
) -> CollisionObject
```

Returns occupied geometry outside the supplied drivable polygons. Empty input returns full space. Also exported from root `crcc`.

## Exceptions

| Exception | Typical cause |
| --- | --- |
| `ValueError` | Invalid geometry, invalid range, unsupported operation, backend mismatch, or invalid CommonRoad data. |
| `TypeError` | Python value cannot convert to the declared native argument type. |
| `OverflowError` | Integer does not fit signed 32-bit time or another native integer. |
| `RuntimeError` | Dedicated Rayon pool construction failed. |

Unsupported operations have not produced a collision result. Never treat their exception as `False`.
