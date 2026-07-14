# Python API reference

Import geometry from `crcc.collision_object`, checker types from `crcc.collision_checker`, trajectories from `crcc.dynamic_obstacle`, and transforms from `crcc.pose`. The package root also re-exports these names. Functions raise `ValueError` for invalid geometry, inverted time bounds, unavailable engines, and unsupported operations unless stated otherwise. Angles are radians; coordinates and sizes use the caller's consistent length unit.

## Geometry and poses

### `Pose`

`Pose(translation: tuple[float, float], angle: float)` represents a rigid transform.

- `Pose.identity() -> Pose`
- `Pose.from_translation(translation) -> Pose`
- `Pose.from_rotation(angle) -> Pose`
- `translation -> tuple[float, float]`
- `rotation -> float`
- `compose(other: Pose) -> Pose`
- `pose * other -> Pose`, equivalent to `compose`

Composition applies `other` first and then `pose`.

### `CollisionObject`

Base class for all geometry.

- `collides(other, pos_self=Pose.identity(), pos_other=Pose.identity(), engine=CollisionEngine.Parry) -> bool`
- `distance(other, pos_self=Pose.identity(), pos_other=Pose.identity(), engine=CollisionEngine.Parry) -> float`
- `collides_continuous(start_pos_self, end_pos_self, other, start_pos_other, end_pos_other, engine=CollisionEngine.Parry) -> bool`
- `merge(other) -> CollisionObject`
- `CollisionObject.merge_all(collision_objects) -> CollisionObject`

Distance is non-negative and may be unsupported for a backend/shape combination. For continuous queries, `False` certifies separation and `True` may be conservative.

### Shape constructors

- `Circle(radius: float, center=(0.0, 0.0))`: requires a finite positive radius.
- `Rectangle(length: float, width: float, orientation=0.0, center=(0.0, 0.0))`: requires finite positive dimensions.
- `Triangle(a, b, c)`: three `(x, y)` vertices; degenerate triangles are rejected.
- `Polygon(exterior, interiors=None)`: an exterior coordinate ring and optional interior rings. Non-convex polygons and holes are supported; invalid or empty topology is rejected.
- `HalfSpace(outward_normal, offset=0.0)`: points satisfying `normal dot point <= offset`.
- `HalfSpace.from_points(p1, p2) -> HalfSpace`: boundary through two distinct points; the stored region is to the right of the directed line.
- `HalfSpace.from_coeffs(a, b, c=0.0) -> HalfSpace`: points satisfying `a*x + b*y <= c`.
- `Compound(collision_objects)`: union of all children. An empty sequence produces empty geometry.
- `Empty()`: never collides.
- `FullSpace()`: occupies the entire plane.

All shape classes inherit the pair and merge methods from `CollisionObject`.

## Engines and results

### `CollisionEngine`

Runtime backend enum with values `CollisionEngine.Parry`, `CollisionEngine.Rhusics`, and `CollisionEngine.Collide`. Python pair methods and builders default to Parry.

### `CollisionStatus`

Constructors/variants are `CollisionStatus.NoCollision()`, `CollisionStatus.CollidesStatic()`, and `CollisionStatus.CollidesDynamic(time_step)`.

- `collides -> bool`: true for either collision variant.
- `time_step -> int | None`: first dynamic collision step, otherwise `None`.
- `str(status) -> str`: readable variant text.

## Checkers

### `CollisionCheckerBuilder`

`CollisionCheckerBuilder(engine: CollisionEngine | None = None)` creates an empty builder. Methods mutate and return the same builder for chaining:

- `with_engine(engine) -> CollisionCheckerBuilder`
- `with_static_obstacle(collision_object) -> CollisionCheckerBuilder`
- `with_dynamic_obstacle(dynamic_obstacle) -> CollisionCheckerBuilder`
- `with_road_boundary(lanelets) -> CollisionCheckerBuilder`
- `build() -> CollisionChecker`

Static children are merged during `build()`. A checker is immutable; rebuild it when scene geometry changes.

### `CollisionChecker`

Instances are created by the builder.

- `engine -> CollisionEngine`
- `collides_static(query_shape, position=None, min_time=None, max_time=None) -> CollisionStatus`
- `collides_dynamic(dynamic_obstacle, min_time=None, max_time=None) -> CollisionStatus`
- `par_static(positioned_query_shapes, min_time=None, max_time=None) -> list[CollisionStatus]`
- `par_static_threads(positioned_query_shapes, threads, min_time=None, max_time=None) -> list[CollisionStatus]`
- `par_dynamic(dynamic_obstacles, min_time=None, max_time=None) -> list[CollisionStatus]`

`positioned_query_shapes` contains `(CollisionObject, Pose)` pairs. Bounds are inclusive and may be omitted independently; include both `t` and `t + 1` to check motion across that segment. Static scene geometry is checked regardless of the time window. Batch results preserve input order. The automatic batch spellings `collides_static_batch` and `collides_dynamic_batch` remain available for tool code.

## Dynamic obstacles

### `DynamicObstacle`

`DynamicObstacle(shape, positions, time_offset)` defines a fixed local shape at successive poses. `positions[0]` is active at `time_offset`; each later pose advances one step.

`DynamicObstacle.from_time_variant(obstacles, time_offset=0, positions=None) -> DynamicObstacle` permits a different shape per step. Identity poses are used when `positions` is omitted. Shape and pose counts must match or `ValueError` is raised.

## `crcc.commonroad`

These Python-only helpers convert CommonRoad and Shapely objects into the core facade:

- `create_collision_checker_from_scenario(scenario: Scenario, builder: CollisionCheckerBuilder | None = None) -> CollisionCheckerBuilder`: adds road boundary, static obstacles, and dynamic obstacles to a builder.
- `add_commonroad_static_obstacle_to_builder(builder, static_obstacle: StaticObstacle) -> CollisionCheckerBuilder`
- `add_commonroad_dynamic_obstacle_to_builder(builder, dynamic_obstacle: commonroad.scenario.obstacle.DynamicObstacle) -> CollisionCheckerBuilder`
- `commonroad_dynamic_obstacle(dynamic_obstacle) -> DynamicObstacle`: supports trajectory and set-based predictions; other predictions raise `NotImplementedError`.
- `add_road_boundary_to_builder(builder, lanelet_network: LaneletNetwork) -> CollisionCheckerBuilder`
- `road_boundary(lanelet_network: LaneletNetwork) -> CollisionObject`: geometry for all space outside the network's lanelets.
- `commonroad_polygon(polygon: shapely.geometry.Polygon) -> CollisionObject`
- `commonroad_shape(shape: ObstacleShape) -> CollisionObject`
- `commonroad_occupancy(occupancy: Occupancy) -> CollisionObject`: returns world-positioned geometry.
- `shapely_geometry(geometry: BaseGeometry) -> CollisionObject`: supports empty geometry, Polygon, and MultiPolygon; other types raise `ValueError`.
- `commonroad_state_to_pose(state: TraceState) -> Pose`

The module also exposes `ROAD_BOUNDARY_SIMPLIFY_TOLERANCE` and `ROAD_BOUNDARY_MIN_HOLE_AREA` as implementation-tuning constants, but they are not included in its declared public export list.

See the [CommonRoad workflow](usage.md#commonroad-workflow-python) for an end-to-end example.
