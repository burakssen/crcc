# Geometry and poses

CRCC represents occupied 2D geometry in local coordinates. A `Pose` supplies a world translation and counter-clockwise rotation in radians when geometry is queried or inserted into a trajectory.

## Shape types

The public object model includes circles, rectangles, triangles, polygons, half-spaces, `Empty`, `FullSpace`, and compounds. Constructors validate the domain geometry: coordinates must be finite, dimensions positive, and polygon topology valid. Rust applications should use `CollisionObject` constructors; Python applications construct the concrete classes exported from `crcc`.

Polygon input may be non-convex and may have interior rings. CRCC classifies and decomposes polygons when converting them to a backend representation; it does not expose polygon Boolean operations as the shape-composition API.

## Compounds are structural unions

A compound represents the union of its children for collision queries. Merging objects concatenates their components; it does not compute a new polygon boundary. Empty children are discarded, while a full-space child makes the union full space. An empty compound is equivalent to `Empty`.

## Poses

Shape centers and vertices are local to the object. For example, a circle whose local center is `(1, 0)` queried at translation `(3, 0)` has world center `(4, 0)` before accounting for rotation.

Python `Pose` validates finite values. Rust exports `glamx::DPose2` as `Pose`; it is a type alias and does not add CRCC validation to pair-query arguments. Dynamic-trajectory constructors validate their poses in both interfaces.

## Broad and narrow phase

Some scene and continuous queries use conservative bounds to reject clearly separated pairs before invoking a narrower backend query. A broad-phase overlap is only a candidate; narrow-phase work determines collision where supported. If the implementation cannot prove separation, a conservative continuous query may report a possible collision.

The exact broad- and narrow-phase algorithms vary by backend. See [backend trade-offs](backends.md) and the [query pipeline](../architecture/query-pipeline.md).
