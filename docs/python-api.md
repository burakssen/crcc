# Python API Reference

This page documents the supported Python surface exported by `crcc`, `crcc.collision_checker`, and `crcc.commonroad`. Start with the [Python usage guide](python-guide.md) for complete workflows.

Angles are radians. Native `CrccError` values become Python `ValueError`. Python argument conversion may additionally raise `TypeError` or `OverflowError`.

## Root Exports

The following names are supported from `crcc`:

```text
Circle CollisionChecker CollisionCheckerBuilder CollisionEngine
CollisionObject CollisionStatus Compound DynamicObstacle Empty FullSpace
HalfSpace Polygon Pose PreparedDynamicQuery PreparedStaticQuery Rectangle
Triangle
```

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
to_dynamic_obstacle(dynamic_obstacle) -> DynamicObstacle
road_boundary(lanelet_network) -> CollisionObject
to_polygon(polygon: shapely.geometry.Polygon) -> CollisionObject
to_shape(shape: ObstacleShape) -> CollisionObject
to_occupancy(occupancy: Occupancy) -> CollisionObject
from_shapely(geometry: BaseGeometry) -> CollisionObject
to_pose(state: TraceState) -> Pose
```

`from_shapely` accepts empty geometry, `Polygon`, and `MultiPolygon`; other non-empty geometry types raise `ValueError`. `to_pose` requires an exact two-dimensional position and orientation.

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
    engine: CollisionEngine | None = None,
) -> bool
```

Applies both poses and asks the selected engine whether occupied sets overlap. `None` selects the compiled default.

### Continuous collision

```python
obj.collides_continuous(
    start_pos_self: Pose,
    end_pos_self: Pose,
    other: CollisionObject,
    start_pos_other: Pose,
    end_pos_other: Pose,
    engine: CollisionEngine | None = None,
) -> bool
```

Checks endpoints and motion between them. `False` certifies interval separation. `True` can be conservative.

### Distance

```python
obj.distance(
    other: CollisionObject,
    pos_self: Pose = Pose.identity(),
    pos_other: Pose = Pose.identity(),
    engine: CollisionEngine | None = None,
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

## `CollisionEngine`

Runtime selector values:

```python
CollisionEngine.Parry
CollisionEngine.Rhusics
CollisionEngine.Collide
```

The distributed build enables all three and defaults to Parry. Integer equality exists in the current extension but is not a stable serialization contract.

## `CollisionStatus`

Constructors/variants:

```python
CollisionStatus.NoCollision() -> CollisionStatus
CollisionStatus.CollidesStatic() -> CollisionStatus
CollisionStatus.CollidesDynamic(t: int) -> CollisionStatus
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
    engine: CollisionEngine | None = None,
)
```

`None` selects the compiled default.

| Method | Return | Description |
| --- | --- | --- |
| `with_engine(engine)` | same builder | Replace selected engine. |
| `with_static_obstacle(query_shape)` | same builder | Add static geometry. |
| `with_dynamic_obstacle(dynamic_obstacle)` | same builder | Add a trajectory. |
| `with_road_boundary(lanelets)` | same builder | Add occupied space outside polygon sequences. |
| `build()` | `CollisionChecker` | Build immutable native scene. |

The Python wrapper mutates and returns itself for chaining. The core state is cloned during build, so a builder can be reused.

## `CollisionChecker`

Construct with `CollisionCheckerBuilder.build()`; direct construction is not public.

### Metadata and preparation

```python
checker.engine -> CollisionEngine
checker.prepare_static(query_shape: CollisionObject) -> PreparedStaticQuery
checker.prepare_dynamic(dynamic_obstacle: DynamicObstacle) -> PreparedDynamicQuery
```

Prepared classes cannot be directly constructed. Both expose `.engine`.

### Static query

```python
checker.collides_static(
    query_shape: CollisionObject,
    position: Pose | None = None,
    min_time: int | None = None,
    max_time: int | None = None,
) -> CollisionStatus
```

Static scene geometry is always checked first. Time bounds restrict dynamic-scene checks only.

```python
checker.collides_static_prepared(
    query: PreparedStaticQuery,
    position: Pose | None = None,
    min_time: int | None = None,
    max_time: int | None = None,
) -> CollisionStatus
```

### Dynamic query

```python
checker.collides_dynamic(
    dynamic_obstacle: DynamicObstacle,
    min_time: int | None = None,
    max_time: int | None = None,
) -> CollisionStatus
```

```python
checker.collides_dynamic_prepared(
    query: PreparedDynamicQuery,
    min_time: int | None = None,
    max_time: int | None = None,
) -> CollisionStatus
```

Dynamic queries check against static and dynamic scene geometry. A hit against static scene geometry is still attributed as `CollidesDynamic(t)`.

### Ordered batches

```python
checker.collides_static_batch(
    positioned_query_shapes: Sequence[tuple[CollisionObject, Pose]],
    min_time: int | None = None,
    max_time: int | None = None,
) -> list[CollisionStatus]
```

```python
checker.collides_dynamic_batch(
    dynamic_obstacles: Sequence[DynamicObstacle],
    min_time: int | None = None,
    max_time: int | None = None,
) -> list[CollisionStatus]
```

Results preserve input order. Empty input returns `[]`. Inputs below 32 execute sequentially; larger inputs use Rayon.

### Compatibility aliases

`par_static(...)` and `par_dynamic(...)` delegate to the corresponding automatic batch methods. `par_static_threads(..., threads=...)` creates a dedicated Rayon pool and clamps zero threads to one. New code should prefer `collides_*_batch` unless it explicitly needs a dedicated static-query pool.

The underscore-prefixed `_collides_static_batch_threads` is an implementation detail even though it is visible in the extension stub.

### Time bounds

Python bounds are inclusive. `None` means unbounded. `min_time > max_time` raises `ValueError`. Interval `t -> t+1` runs only if both values are selected.

## Low-Level Road Boundary

```python
from crcc.collision_checker import road_boundary

road_boundary(
    lanelets: Sequence[Sequence[tuple[float, float]]],
) -> CollisionObject
```

Returns occupied geometry outside the supplied drivable polygons. Empty input returns full space. This function is not exported from root `crcc`.

## Exceptions

| Exception | Typical cause |
| --- | --- |
| `ValueError` | Invalid geometry, invalid range, unsupported operation, engine mismatch, or invalid CommonRoad data. |
| `TypeError` | Python value cannot convert to the declared native argument type. |
| `OverflowError` | Integer does not fit signed 32-bit time or another native integer. |
| `RuntimeError` | Dedicated Rayon pool construction failed. |

Unsupported operations have not produced a collision result. Never treat their exception as `False`.
