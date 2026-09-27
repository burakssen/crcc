# Errors and limits

An unsupported operation has not produced a collision answer. Do not convert an error into `false` or treat it as certified free space.

## Errors

Rust domain operations return `CrccResult<T> = Result<T, CrccError>`:

| Variant | Meaning |
| --- | --- |
| `InvalidRadius(f64)` | Circle radius was non-finite or not positive. |
| `NotConvex` | A convex-only operation received non-convex geometry. |
| `HasHoles` | An operation that excludes interiors received a holed polygon. |
| `EmptyShape` | An operation requiring area received empty/degenerate geometry. |
| `InvalidGeometry(&'static str)` | Geometry, topology, pose, or trajectory validation failed. |
| `Unsupported` | The selected backend, representation, or query combination is unavailable. |

PyO3 maps `CrccError` to Python `ValueError`. Python argument conversion can also raise `TypeError` or `OverflowError`.

## Geometry and operation limits

- Circle radii and rectangle dimensions must be finite and positive; polygon inputs must be finite, nondegenerate, and topologically valid.
- Rust pair-query poses are `DPose2` values and are not validated by the alias itself. Python `Pose` validates finite components; trajectory constructors validate poses in either language.
- `TimeStep` uses signed 32-bit integers. Trajectory extent must fit the representable range. Python bounds are inclusive and reject `min_time > max_time`; Rust accepts standard range bounds and treats inverted ranges as empty.
- Distance to empty geometry is unsupported. Backend distance implementations can differ slightly near contact.
- Backend support and semantics vary for shape combinations, tangency, and continuous queries. See [backend behavior](../concepts/backends.md).
- Continuous positives can be conservative. A negative certifies separation under CRCC's contract; it does not mean the application should ignore invalid geometry or errors.
- CRCC does not resolve contact, compute impulses, evolve a simulation, mutate a built scene, or return contact manifolds.
- No public geometry serialization API is provided.

## Unsupported versus no collision

| Result | Interpretation |
| --- | --- |
| `Ok(false)` / `NoCollision` | Query completed and reported no collision for the selected scope. |
| `Err(CrccError::Unsupported)` / Python `ValueError` | Query was not completed for that backend/geometry combination. |
| `Ok(true)` / collision status | Collision was reported; for continuous queries it may be conservative. |
