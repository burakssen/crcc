# Python API reference

The supported Python surface is exported by `crcc` and `crcc.commonroad`. Geometry and query classes are native PyO3 types; the builder is a Python facade. Signatures below use `XY = tuple[float, float]` and `Sequence` for lists/tuples or equivalent sequence objects, **not arbitrary generators**. See [guides](../guides/pair-queries.md) for runnable workflows.

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

`Circle(radius: float, center: XY)` uses a strictly positive radius. `Rectangle(length: float, width: float, orientation: float, center: XY)` uses full local x/y extents, not half extents. Triangle points and polygon ring vertices are `XY`; `interiors` is a sequence of rings. `Compound` accepts a sequence of `CollisionObject` children. See [Geometry and poses](../concepts/geometry-and-poses.md) for validation/sanitization and coordinate conventions.

Half-space factories:

```text
HalfSpace.from_points(point_1: XY, point_2: XY) -> HalfSpace
HalfSpace.from_coeffs(a: float, b: float, c: float = 0.0) -> HalfSpace
```

The occupied side of a directed line is its right side; coefficient construction means $`a x + b y \le c`$. Constructors normalize normal and offset together. Angles are counter-clockwise radians. Invalid native geometry raises `ValueError`; argument extraction can raise `TypeError` or `OverflowError`.

`Pose(translation, angle)` represents a finite rigid transform. Helpers are `Pose.identity()`, `Pose.from_translation(...)`, and `Pose.from_rotation(...)`; `.compose(other)` and `pose * other` compose transforms, applying the right operand first. `.translation` and `.rotation` return the components.

```text
Pose(translation: XY, angle: float) -> Pose
Pose.identity() -> Pose
Pose.from_translation(translation: XY) -> Pose
Pose.from_rotation(angle: float) -> Pose
pose.translation -> XY                 # read-only
pose.rotation -> float                 # principal angle, read-only
pose.compose(other: Pose) -> Pose
pose * other -> Pose
```

`Pose` constructor arguments have no defaults. Construction/composition rejects nonfinite transforms with `ValueError`. Shape properties such as `.radius`, `.vertices`, and `.children` are not exposed. `CollisionObject`, `CollisionChecker`, and prepared-query types cannot be constructed directly; use concrete shapes, builder `build()`, and `prepare_*`.

`CollisionObject` is the queryable base type. Its operations are:

```text
obj.collides(other, pos_self=Pose.identity(), pos_other=Pose.identity(), backend=None)
obj.collides_continuous(start_pos_self, end_pos_self, other,
                        start_pos_other, end_pos_other, backend=None)
obj.distance(other, pos_self=Pose.identity(), pos_other=Pose.identity(), backend=None)
obj.merge(other)
CollisionObject.merge_all(objects)
```

Here `other` is a `CollisionObject`, poses are `Pose`, and `backend` is `CollisionBackend | None`. The query methods return `bool`, `bool`, and non-negative `float` respectively; native failures raise `ValueError`. `merge` and `merge_all` return a base `CollisionObject`, not necessarily a `Compound` subclass. They represent a union without exposing mutable children. `merge_all` takes a sequence.

Pair methods additionally accept `engine: CollisionBackend | None = None` after `backend` as a deprecated alias. It emits `DeprecationWarning`; providing both non-`None` selectors raises `TypeError`. Continuous positives can be conservative. Distance involving empty geometry is unsupported. See [backend behavior](../concepts/backends.md).

## Backend and builder

Select `CollisionBackend.Parry`, `.Rhusics`, or `.Collide`. The distributed extension enables all three; the default is Parry.

```text
builder = CollisionCheckerBuilder(backend=None)
builder.add_static_obstacle(shape)
builder.add_dynamic_obstacle(trajectory)
checker = builder.build()
```

`backend=None` selects the compiled default. The builder methods mutate and return the builder for chaining. `build()` returns an immutable native checker. Deprecated aliases `with_static_obstacle`, `with_dynamic_obstacle`, `with_engine`, and `with_road_boundary` remain available; use the corresponding `add_*` methods, `backend=`, and `add_static_obstacle(road_boundary(...))` instead.

```text
CollisionCheckerBuilder(backend: CollisionBackend | None = None,
                        *, engine: CollisionBackend | None = None)
builder.add_static_obstacle(query_shape: CollisionObject) -> CollisionCheckerBuilder
builder.add_dynamic_obstacle(dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder
builder.build() -> CollisionChecker
road_boundary(lanelets: Sequence[Sequence[XY]]) -> CollisionObject
```

`build()` clones the native builder; additions afterward do not change an existing checker. It accepts no backend override. `engine=` warns, and using both selectors raises `TypeError`. The facade can retain an invalid selector until `build()` fails. Backend selection is checked when the native builder is used; geometry query errors can occur later.

## Scene queries

`CollisionChecker` exposes `.backend`, `.prepare_static(shape)`, and `.prepare_dynamic(trajectory)`.

```text
checker.backend -> CollisionBackend
checker.prepare_static(query_shape: CollisionObject) -> PreparedStaticQuery
checker.prepare_dynamic(dynamic_obstacle: DynamicObstacle) -> PreparedDynamicQuery
prepared.backend -> CollisionBackend

checker.collides_static(
    query: CollisionObject | PreparedStaticQuery,
    position: Pose | None = None,
    min_time: int | None = None, max_time: int | None = None,
) -> CollisionStatus
checker.collides_dynamic(
    query: DynamicObstacle | PreparedDynamicQuery,
    min_time: int | None = None, max_time: int | None = None,
) -> CollisionStatus
checker.collides_static_batch(
    queries: Sequence[tuple[CollisionObject | PreparedStaticQuery, Pose]],
    min_time: int | None = None, max_time: int | None = None,
    parallel: bool = False,
) -> list[CollisionStatus]
checker.collides_dynamic_batch(
    queries: Sequence[DynamicObstacle | PreparedDynamicQuery],
    min_time: int | None = None, max_time: int | None = None,
    parallel: bool = False,
) -> list[CollisionStatus]
```

Prepared queries are backend-specific and reusable across same-backend checkers. Wrong-backend use raises `ValueError`. Static scalar `position=None` means identity; static batch entries require a `Pose`.

Python bounds are inclusive; omitted bounds are the signed 32-bit extremes. Inversion raises `ValueError`, wrong argument types can raise `TypeError`, and integers outside signed 32-bit range raise `OverflowError`. Static queries always check static scene geometry. Dynamic queries are restricted to active query samples and can include outgoing intervals beyond `max_time`; see [Scenes and time](../concepts/scenes-and-time.md).

Batch result order matches input order. `parallel=True` requests Rayon; the default is sequential. Batch entries may mix raw and prepared objects. Any native error raises for the entire Python call; no partial result/error list is returned. Empty batches return `[]` after validating time bounds. Native batch work releases the GIL; scalar methods do not explicitly release it.

`CollisionStatus` results expose `.collides: bool` and `.time_step: int | None`, and readable `str`/`repr`:

| Representation | `.collides` | `.time_step` |
| --- | --- | --- |
| `NoCollision` | False | None |
| `CollidesStatic` | True | None |
| `CollidesDynamic(t)` | True | t |

A dynamic query hitting static scene geometry still returns `CollidesDynamic(t)`. Use `.collides`, never status truthiness: `bool(CollidesStatic)` can be false. PyO3 also exposes `CollisionStatus.NoCollision()`, `.CollidesStatic()`, and `.CollidesDynamic(time_step)` variant constructors. Prefer properties over generated tuple internals such as `_0`.

## DynamicObstacle

```text
DynamicObstacle(shape: CollisionObject, positions: Sequence[Pose], time_offset: int)
DynamicObstacle.from_time_variant(
    obstacles: Sequence[CollisionObject], time_offset: int = 0,
    positions: Sequence[Pose] | None = None,
) -> DynamicObstacle
```

For fixed geometry, the pose at index $`i`$ is active at time $`t = \mathrm{time\_offset} + i`$. For varying geometry, shape and pose counts must match; omitted poses are identities. Empty geometry marks missing occupancy and suppresses its adjacent motion intervals.

`time_offset` is required for fixed geometry. Empty sequences are accepted. Mismatched lengths or unrepresentable trajectory end time raise `ValueError`; argument integer overflow raises `OverflowError`. No Python accessors expose stored samples. See [trajectory workflows](../guides/scenes-and-trajectories.md).

## CommonRoad module

`crcc.commonroad` depends on `commonroad-io` and exposes:

```text
scenario_builder add_static_obstacle add_dynamic_obstacle add_road_boundary
road_boundary from_dynamic_obstacle from_occupancy from_shape from_polygon
from_shapely from_pose
```

`scenario_builder(scenario, builder=None)` adds lanelet boundary geometry (when lanelets exist), static obstacles, and dynamic obstacles. `from_shapely` accepts empty geometry, Polygon, and MultiPolygon; other non-empty geometry types raise `ValueError`. CommonRoad time steps must be exact integers. See the [CommonRoad guide](../guides/commonroad.md).

All functions below are imported from `crcc.commonroad`. `StaticObstacle`/`DynamicObstacle` input types refer to CommonRoad, while return `DynamicObstacle` refers to CRCC:

```text
scenario_builder(scenario: Scenario, builder: CollisionCheckerBuilder | None = None)
    -> CollisionCheckerBuilder
add_static_obstacle(builder: CollisionCheckerBuilder, static_obstacle: StaticObstacle)
    -> CollisionCheckerBuilder
add_dynamic_obstacle(builder: CollisionCheckerBuilder, dynamic_obstacle: DynamicObstacle)
    -> CollisionCheckerBuilder
add_road_boundary(builder: CollisionCheckerBuilder, lanelet_network: LaneletNetwork)
    -> CollisionCheckerBuilder
road_boundary(lanelet_network: LaneletNetwork) -> CollisionObject
from_dynamic_obstacle(dynamic_obstacle: DynamicObstacle) -> crcc.DynamicObstacle
from_shape(shape: ObstacleShape) -> CollisionObject
from_occupancy(occupancy: Occupancy) -> CollisionObject
from_polygon(polygon: shapely.geometry.Polygon) -> CollisionObject
from_shapely(geometry: shapely.geometry.base.BaseGeometry) -> CollisionObject
from_pose(state: TraceState) -> Pose
```

Shapes are local; occupancies are world-positioned. Adapter invalid-state/time/unsupported-geometry checks raise `ValueError`; third-party conversion/parsing exceptions can also propagate. Import functions explicitly; incidental imported module names are not adapter APIs.

## Rust-to-Python mapping

| Python | Rust | Difference |
| --- | --- | --- |
| `CollisionObject` and subclasses | `CollisionObject` / `SimpleCollisionObject` | Opaque native geometry, no Python geometry getters |
| `CollisionBackend` | `CollisionEngine` | Different canonical name |
| `CollisionChecker` | `SelectedCollisionChecker` | Python uses runtime dispatch, not `CollisionChecker<E>` |
| `Pose` | `glamx::DPose2` | Python adds finite validation |
| `DynamicObstacle` | `DynamicObstacle` | Python takes sequences; Rust owns vectors |
| Status time | `TimeStep(i32)` | Python integer, no `TimeStep` wrapper |
| Batch list | `Vec<CollisionResult>` | Python raises on any failure |
| `ValueError` | `CrccError` | No per-variant Python exception classes |

## Compatibility aliases

The following remain implemented and emit `DeprecationWarning` on use. No removal version is specified here. Prefer canonical methods above.

| Compatibility API | Replacement |
| --- | --- |
| Builder `with_engine(engine)` | Set `backend=` when constructing the builder |
| Builder `with_static_obstacle(query_shape)` / `with_dynamic_obstacle(dynamic_obstacle)` | `add_static_obstacle` / `add_dynamic_obstacle` |
| Builder `with_road_boundary(lanelets)` | `add_static_obstacle(crcc.road_boundary(lanelets))` |
| Checker/prepared `.engine` | `.backend` |
| Pair `engine=` | `backend=` |

The prepared scalar aliases have the canonical scalar arguments, restricted to prepared inputs. Additional signatures:

```text
checker.collides_static_prepared(query, position=None, min_time=None, max_time=None)
checker.collides_dynamic_prepared(query, min_time=None, max_time=None)
checker.collides_static_prepared_batch(query: PreparedStaticQuery, positions: Sequence[Pose],
    min_time=None, max_time=None, parallel=False) -> list[CollisionStatus]
checker.collides_dynamic_prepared_batch(queries: Sequence[PreparedDynamicQuery],
    min_time=None, max_time=None, parallel=False) -> list[CollisionStatus]
checker.par_static(positioned_query_shapes: Sequence[tuple[CollisionObject, Pose]],
    min_time=None, max_time=None) -> list[CollisionStatus]
checker.par_dynamic(dynamic_obstacles: Sequence[DynamicObstacle],
    min_time=None, max_time=None) -> list[CollisionStatus]
```

`par_*` accepts raw inputs and requests parallel execution. `CollisionEngine` remains an identity alias of `CollisionBackend` in the facade, without an import warning. `_core` module layout and benchmark instrumentation are implementation details, not application APIs.

## Exceptions

CRCC errors become Python `ValueError`. Python argument conversion can also raise `TypeError` or `OverflowError`. Unsupported geometry/query pairs do not return `False`; see [errors and limits](errors-and-limits.md).
