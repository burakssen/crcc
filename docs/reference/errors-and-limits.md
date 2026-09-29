# Error handling and validation

An unsupported operation has not produced a collision answer. Do not convert an error into `false` or treat it as certified free space.

## Errors

Rust domain operations return `CrccResult<T> = Result<T, CrccError>`:

| Variant | Meaning |
| --- | --- |
| `InvalidRadius(f64)` | Circle center or radius was non-finite, or radius was not positive. Payload is the radius even for invalid centers. |
| `NotConvex` | A convex-only operation received non-convex geometry. |
| `HasHoles` | An operation that excludes interiors received a holed polygon. |
| `EmptyShape` | An operation requiring area received empty/degenerate geometry. |
| `InvalidGeometry(&'static str)` | Geometry, topology, pose, or trajectory validation failed. |
| `Unsupported` | The selected backend, representation, or query combination is unavailable. |

PyO3 maps `CrccError` to Python `ValueError`. Python argument conversion can also raise `TypeError` or `OverflowError`.

## Constructor validation

- Circle centers must be finite; radii finite and positive. Derived extents are not universally checked.
- Python rectangles require positive dimensions. Rust checks finite corners, orientation, derived dimensions and center, but accepts zero dimensions.
- Triangles reject nonfinite coordinates and computed zero area; no minimum-area epsilon is applied.
- Regular polygon constructors collapse consecutive duplicate vertices and discard unusable holes before checking retained coordinates/topology. They classify the polygon; decomposition is deferred to backend conversion.
- Rust `ConvexPolygon::new` checks only holes and convexity; `NonConvexPolygon::new` only holes; `PolygonWithHoles::new` performs no validation. Their conversion into `CollisionObject` does not repeat full validation. Prefer `CollisionObject::polygon`.
- Half-space constructors validate/normalize normal and offset. Public Rust fields allow bypassing these invariants.
- Rust pair-query poses are `DPose2` values and are not validated by the alias itself. Python `Pose` validates finite components; trajectory constructors check finite translation and rotation angle, not unit-length arbitrary Rust rotation components.
- `TimeStep` uses signed 32-bit integers. Trajectory extent must fit the representable range; empty trajectories are valid. Python bounds are inclusive and reject inversion; wrong types/out-of-range integers can raise `TypeError`/`OverflowError`. Rust inverted ranges select no dynamic times, but a static query still checks static scene geometry.
- Distance to empty geometry is unsupported. Backend distance implementations can differ slightly near contact.
- Backend support and semantics vary for shape combinations, tangency, and continuous queries. See [backend behavior](../concepts/backends.md).
- Continuous positives can be conservative. Negative results follow the backend's motion/contact convention and assume valid geometry/poses and representable calculations.
- CRCC does not resolve contact, compute impulses, evolve a simulation, mutate a built scene, or return contact manifolds.
- No public geometry serialization API is provided.

## Error timing and batches

Domain validation errors occur at construction. Backend `From<CollisionObject>` conversion is infallible at the type level; a representation can retain failure for a later query. Building/preparing therefore does not certify all shape/query combinations. Disabled runtime engines return `Unsupported`.

Scene queries short-circuit: an earlier hit/error can prevent later unsupported geometry from being examined. An empty dynamic window can return `NoCollision` without exercising a stored invalid representation.

Rust batches return one `CollisionResult` per input. Wrong-backend prepared entries in heterogeneous batches fail whole-batch preflight; homogeneous prepared dynamic batches fail per slot. Python collects native results into one list and raises on any error, without returning partial results.

## Panics and recovery

The core uses checked time operations and denies several panic-related Clippy patterns. This is not a blanket no-panic guarantee for dependencies, allocation failures, invalid lower-level geometry, or arbitrary malformed poses. Constructors and query APIs expose recoverable `CrccError` where implemented; do not catch every programming exception and label it an unsupported collision query.

```python
from crcc import Circle, CollisionCheckerBuilder, Pose

try:
    Circle(-1.0)
except ValueError:
    pass
else:
    raise AssertionError("negative radius must fail")

checker = CollisionCheckerBuilder().build()
try:
    checker.collides_static(Circle(1.0), Pose.identity(), min_time=2, max_time=1)
except ValueError:
    pass
else:
    raise AssertionError("inverted Python bounds must fail")
```

For numerical assumptions and unsupported functionality, see [Numerical robustness](../concepts/numerical-robustness.md) and [Limitations](limitations.md).

## Unsupported versus no collision

| Result | Interpretation |
| --- | --- |
| `Ok(false)` / `NoCollision` | Query completed and reported no collision for the selected scope. |
| `Err(CrccError::Unsupported)` / Python `ValueError` | Query was not completed for that backend/geometry combination. |
| `Ok(true)` / collision status | Collision was reported; for continuous queries it may be conservative. |
