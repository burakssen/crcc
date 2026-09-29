# Limitations and FAQ

## Geometry and queries

- Only 2D geometry is represented; no lines/points as public finite shape types, 3D geometry, meshes as user input, or general Shapely geometry collections.
- Backend conversion does not imply every combination supports discrete, distance, and continuous queries equally. An unsupported query is an error, not clear space.
- Rust lower-level wrappers/fields permit inputs that bypass normal constructor validation. Rust poses must be valid rigid transforms. See [validation](errors-and-limits.md).
- Boundary contact, tolerances, interpolation and numerical behavior differ by backend. There is no configurable global epsilon.
- Continuous queries return a Boolean/status, not exact contact time or penetration information. Positive results can overestimate rotation and infinite/time-varying geometry.
- Time-varying interval geometry is a swept union, not morphing or a time-resolved occupancy interpolation.
- Trajectories do not persist beyond their final sample; absent occupancy suppresses adjacent intervals. Times are indices, not seconds.
- A dynamic-query upper bound selects interval starts, not necessarily the interval end. See [Scenes and time](../concepts/scenes-and-time.md).

## Performance and state

Scenes are immutable. Updating obstacles means rebuilding; there is no incremental broad-phase update API. Dynamic obstacles are checked in order and active times are materialized. CRCC has no shared scene-wide spatial index: native compounds and Collide's local finite sets provide backend-specific acceleration. Avoid assuming scene queries are sublinear in obstacle count.

Raw pair/selected-checker queries clone/convert geometry at call boundaries. Prepared queries amortize conversion. Parallel batches have scheduling overhead and no adaptive size threshold. Large active-time spans and conservative sweeps can increase memory/work substantially.

No serialization, obstacle-ID results, mutable Python shape fields, or simulation state evolution is provided. The Matplotlib playground and benchmark tools are checkout utilities, not package entry points.

## Integration and platforms

CommonRoad adapters require exact-state data and support a subset of its model collections. Road boundaries are approximate; planning problems and scenario `dt` are not scene metadata. Read [the adapter contract](../guides/commonroad.md) before interpreting converted predictions.

Source builds need Rust and native build tools. Routine behavioral CI is Linux; release wheel smoke checks cover additional platforms. Benchmark RSS/subprocess tooling has narrower platform assumptions. See [Installation](../getting-started/installation.md) and [Benchmarking](../development/benchmarking.md).

## FAQ

**Why does a clear endpoint query collide continuously?** Motion may cross an obstacle between endpoints, or conservative bounds may retain an unresolved candidate. Check backend/motion support and compare sampled occupancy separately.

**Why is `time_step` present when the obstacle is static?** A dynamic query returns `CollidesDynamic(t)` even for a static scene hit. The status attributes the query sample/interval, not the scene obstacle type alone.

**Can I reuse prepared geometry in another scene?** Yes, with the same backend. Different-backend use fails.

**Why did `pip install crcc` install the wrong package?** That PyPI name belongs to an unrelated CRC project. Use this repository's source installation.

**Can I infer collision from distance zero?** Not across all backends/contact conventions. Use the collision query for its intended predicate.
