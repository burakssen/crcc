# Scenes and time

A collision checker is an immutable scene built from static geometry and zero or more dynamic obstacles. The builder performs backend conversion and scene preparation; repeated queries use the built checker.

## Static and dynamic geometry

Static scene objects have fixed geometry. A fixed-shape `DynamicObstacle` stores a shape and a sequence of poses. A time-varying obstacle stores a shape and pose at each sample, allowing occupancy geometry to change or be absent.

Each pose sample occupies one discrete time step. `positions[0]` is active at the obstacle's `time_offset`; subsequent entries advance one step. Motion intervals exist only between adjacent samples. There is no persistence after the final sample.

For a time-varying trajectory, an empty shape at either endpoint suppresses motion across that interval. CRCC does not invent motion across missing occupancy. Empty trajectories are accepted and have no active times; a one-pose trajectory has one sample and no interval. Negative offsets are allowed within signed 32-bit time limits.

## Query windows and results

Python `min_time` and `max_time` bounds are inclusive. Rust checker methods accept standard `RangeBounds<TimeStep>`. The current query policies differ:

| Query | Interval $`t \to t+1`$ selection |
| --- | --- |
| Static query against a scene | Both $`t`$ and $`t+1`$ must be selected scene active times. A singleton window checks occupancy only. |
| Dynamic query against a scene | Select the start $`t`$; its outgoing interval is checked whenever the query has sample $`t+1`$, even if $`t+1`$ is outside the window. |

Consequently, `collides_dynamic(..., min_time=10, max_time=10)` and Rust `collides_dynamic_at(..., TimeStep(10))` can include motion from 10 to 11. For endpoint-only checks, use discrete pair queries at the sampled poses. Dynamic windows are clipped to the query's active span.

**Static queries** always check static scene geometry, including an empty/inverted Rust window. A dynamic query only checks selected active query samples/intervals; an empty trajectory or disjoint window performs no scene checks. A dynamic query that hits static geometry is reported as `CollidesDynamic(t)` because the result describes the moving query.

Times are visited in ascending order. At each selected start, continuous checks precede sampled occupancy, and static scene checks precede dynamic scene checks. Obstacles are visited in insertion order. The result is the first collision under this order, not an exact earliest continuous time of impact or an obstacle identifier. A scene obstacle with no outgoing interval falls back to occupancy at `t`; it is not extended into a stationary trajectory.

Checker statuses are:

| Status | Meaning |
| --- | --- |
| `NoCollision` | No selected static, discrete, or interval query collided. |
| `CollidesStatic` | A static query hit the static scene geometry. |
| `CollidesDynamic(t)` | A hit involving a dynamic query or scene obstacle at sample `t` or interval starting at `t`. |

The interval attribution uses its start step and does not imply that the sampled shapes overlap at that exact step. See [continuous collision](continuous-collision.md) and the [scene guide](../guides/scenes-and-trajectories.md).

## Storage and updates

Builders merge static components and retain separate dynamic obstacles. Building materializes the union of active sample times as an ordered set and converts scene geometry/bounds. This costs memory proportional to the union's cardinality, not just the number of obstacles. There is no scene insertion/removal API after building: modify/reuse a builder and rebuild. Python `build()` clones its underlying builder; Rust build methods consume it.
