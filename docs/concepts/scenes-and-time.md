# Scenes and time

A collision checker is an immutable scene built from static geometry and zero or more dynamic obstacles. The builder performs backend conversion and scene preparation; repeated queries use the built checker.

## Static and dynamic geometry

Static scene objects have fixed geometry. A fixed-shape `DynamicObstacle` stores a shape and a sequence of poses. A time-varying obstacle stores a shape and pose at each sample, allowing occupancy geometry to change or be absent.

Each pose sample occupies one discrete time step. `positions[0]` is active at the obstacle's `time_offset`; subsequent entries advance one step. Motion intervals exist only between adjacent samples. There is no persistence after the final sample.

An empty shape at either endpoint suppresses motion across that interval. CRCC does not invent motion across missing occupancy.

## Query windows and results

Python `min_time` and `max_time` bounds are inclusive. A dynamic interval `t -> t+1` is checked only when both steps are selected. A one-step window checks that sample but not its outgoing interval. Rust checker methods accept standard `RangeBounds<TimeStep>`.

Static scene geometry is checked regardless of the dynamic time window. A dynamic query that hits static geometry is reported as `CollidesDynamic(t)` because the result describes the moving query. When several dynamic samples or intervals collide, the checker reports the earliest selected time.

Checker statuses are:

| Status | Meaning |
| --- | --- |
| `NoCollision` | No selected static, discrete, or interval query collided. |
| `CollidesStatic` | A static query hit the static scene geometry. |
| `CollidesDynamic(t)` | A dynamic query hit at sample `t` or in interval `t -> t+1`. |

The interval attribution uses its start step and does not imply that the sampled shapes overlap at that exact step. See [continuous collision](continuous-collision.md) and the [scene guide](../guides/scenes-and-trajectories.md).
