# Python API Reference

All core classes are re-exported at the package root (`crcc`). Errors like invalid geometry or inverted time bounds raise `ValueError`. Angles are in radians.

## Transforms and Geometry

### `Pose`
Represents a 2D rigid transform. `Pose(translation: tuple[float, float], angle: float)`

| Method / Property | Signature | Description |
| :--- | :--- | :--- |
| `Pose.identity()` | `-> Pose` | Returns identity transform. |
| `Pose.from_translation(translation)` | `-> Pose` | Create pose from translation `(x, y)`. |
| `Pose.from_rotation(angle)` | `-> Pose` | Create pose from rotation angle. |
| `translation` | `tuple[float, float]` | Translation component. |
| `rotation` | `float` | Rotation component in radians. |
| `compose(other)` / `*` | `(other: Pose) -> Pose` | Compose poses (applies `other` then `self`). |

### `CollisionObject`
Base class for all geometric shapes.

| Method | Signature | Description |
| :--- | :--- | :--- |
| `collides` | `(other, pos_self=Pose.identity(), pos_other=Pose.identity(), engine=CollisionEngine.Parry) -> bool` | Discrete pairwise collision check. |
| `distance` | `(other, pos_self=Pose.identity(), pos_other=Pose.identity(), engine=CollisionEngine.Parry) -> float` | Pairwise Euclidean distance. |
| `collides_continuous` | `(start_pos_self, end_pos_self, other, start_pos_other, end_pos_other, engine=CollisionEngine.Parry) -> bool` | Continuous pairwise query. |
| `merge` | `(other) -> CollisionObject` | Returns the union of two objects. |
| `CollisionObject.merge_all` | `(collision_objects: Iterable) -> CollisionObject` | Returns the union of multiple objects. |

### Shapes
All shapes inherit from `CollisionObject`.

| Shape | Constructor Signature | Description / Constraints |
| :--- | :--- | :--- |
| `Circle` | `Circle(radius: float, center=(0.0, 0.0))` | Requires finite `radius > 0`. |
| `Rectangle` | `Rectangle(length: float, width: float, orientation=0.0, center=(0.0, 0.0))` | Requires finite positive dimensions. |
| `Triangle` | `Triangle(a: tuple, b: tuple, c: tuple)` | Triangle from three `(x, y)` vertices. |
| `Polygon` | `Polygon(exterior: list, interiors: list = None)` | Exterior and optional holes. |
| `HalfSpace` | `HalfSpace(outward_normal: tuple, offset: float = 0.0)` | Region where `normal dot point <= offset`. |
| `HalfSpace.from_points` | `from_points(p1: tuple, p2: tuple) -> HalfSpace` | Space to the right of the directed line. |
| `HalfSpace.from_coeffs`| `from_coeffs(a, b, c) -> HalfSpace` | Region satisfying `a*x + b*y <= c`. |
| `Compound` | `Compound(collision_objects: list)` | Union of multiple collision objects. |
| `Empty` | `Empty()` | Null shape; never collides. |
| `FullSpace` | `FullSpace()` | Space representing the entire plane. |

---

## Engines & Status

### `CollisionEngine`
Enum specifying the backend: `Parry` (default), `Rhusics`, or `Collide`.

### `CollisionStatus`
Return status from checker queries: `NoCollision()`, `CollidesStatic()`, or `CollidesDynamic(time_step)`.

| Property | Type | Description |
| :--- | :--- | :--- |
| `collides` | `bool` | `True` if any collision occurs. |
| `time_step` | `int \| None` | First dynamic collision step, otherwise `None`. |
| `__str__` | `str` | Readable description of status. |

---

## Checkers

### `CollisionCheckerBuilder`
`CollisionCheckerBuilder(engine: CollisionEngine | None = None)`

| Method | Signature | Description |
| :--- | :--- | :--- |
| `with_engine` | `(engine) -> Self` | Sets the collision backend engine. |
| `with_static_obstacle` | `(obj: CollisionObject) -> Self` | Adds static geometry. |
| `with_dynamic_obstacle` | `(obs: DynamicObstacle) -> Self` | Adds dynamic obstacle trajectory. |
| `with_road_boundary` | `(lanelets) -> Self` | Adds road boundaries. |
| `build` | `() -> CollisionChecker` | Builds an immutable collision checker. |

### `CollisionChecker`
Immutable checker instance.

| Method / Property | Signature | Description |
| :--- | :--- | :--- |
| `engine` | `CollisionEngine` | Selected backend engine. |
| `collides_static` | `(query, position=None, min_time=None, max_time=None) -> CollisionStatus` | Check query against static geometry. |
| `collides_dynamic` | `(dynamic_obstacle, min_time=None, max_time=None) -> CollisionStatus` | Check query against dynamic obstacles. |
| `par_static` | `(positioned_queries: list, min_time=None, max_time=None) -> list` | Parallel batch query (static). |
| `par_static_threads`| `(positioned_queries: list, threads: int, min_time=None, max_time=None) -> list` | Thread-configured batch static query. |
| `par_dynamic` | `(dynamic_obstacles: list, min_time=None, max_time=None) -> list` | Parallel batch query (dynamic). |

---

## Dynamic Obstacles

### `DynamicObstacle`
`DynamicObstacle(shape: CollisionObject, positions: list[Pose], time_offset: int)`

- `DynamicObstacle.from_time_variant(obstacles: list[CollisionObject], time_offset: int = 0, positions: list[Pose] = None) -> DynamicObstacle`: For shape changes across steps.

---

## `crcc.commonroad`
Python-only conversion helpers.

| Helper Function | Description |
| :--- | :--- |
| `scenario_builder(scenario, builder=None)` | Populates a builder with road boundary and obstacles. |
| `add_static_obstacle(builder, static_obstacle)` | Adds a static obstacle to the builder. |
| `add_dynamic_obstacle(builder, dynamic_obstacle)`| Adds a dynamic obstacle to the builder. |
| `to_dynamic_obstacle(dynamic_obstacle)` | Converts dynamic obstacle to `DynamicObstacle`. |
| `add_road_boundary(builder, lanelet_network)` | Adds network boundaries to builder. |
| `road_boundary(lanelet_network)` | Returns boundary `CollisionObject`. |
| `to_polygon(polygon)` / `to_shape(shape)` | Converts shape representation to `CollisionObject`. |
| `to_occupancy(occupancy)` | Returns world-positioned occupancy geometry. |
| `from_shapely(geometry)` | Converts Shapely Polygon/MultiPolygon to `CollisionObject`. |
| `to_pose(state)` | Converts trajectory state to `Pose`. |
