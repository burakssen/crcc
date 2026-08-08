# Concepts and Engine Behavior

This page defines the semantic contract shared by the Rust and Python APIs.

## Geometry and Poses

Geometry is defined in local coordinates. A `Pose` supplies a translation and counter-clockwise rotation in radians when the object is queried or inserted into a trajectory.

For example, a circle with local center `(1, 0)` and a query pose translating by `(3, 0)` has its world center at `(4, 0)`. Prefer finite coordinates and finite poses. High-level geometry constructors validate their inputs, and Python `Pose` validates its values. The Rust `Pose` alias itself does not add validation to `glamx::DPose2`.

## Geometry Sets

`Empty` represents no occupied point. `FullSpace` represents every point in the plane. A compound is the union of its children.

Normalization rules are simple:

- Empty children are removed from compounds.
- A full-space child makes the complete compound full space.
- Merging concatenates occupied components; it does not compute a polygon Boolean union.
- An empty compound is equivalent to empty geometry.

Distance involving empty geometry is unsupported because the implementation does not represent infinite set distance. Full space has distance zero to every non-empty object.

## Validated and Low-Level Geometry

Application code should use the high-level `CollisionObject` constructors in Rust or concrete shape classes in Python. They reject non-finite, empty, degenerate, or topologically invalid input.

Rust also exposes low-level simple-object types for backend and advanced use. Some can be constructed directly without all high-level invariants. Treat those APIs as an escape hatch, not the default geometry path.

## Pair Queries and Scene Queries

Pair queries compare two objects directly:

- Discrete collision at two poses.
- Continuous collision between start and end poses.
- Non-negative separation distance.

Pair queries convert geometry to the selected backend on each call.

Scene queries first build an immutable checker. Static scene objects are merged, dynamic obstacles are converted once, and active time steps are indexed. This costs more up front but is the preferred path for repeated queries.

Prepared queries convert a query once for reuse with a checker. They are tied to that checker's engine; passing one to a checker using another engine is unsupported.

## Discrete Time

CRCC time steps are signed 32-bit integers. A dynamic obstacle assigns its first pose to `time_offset`, the next pose to `time_offset + 1`, and so on.

A trajectory with samples at `t`, `t+1`, and `t+2` contains:

- Discrete occupancy at all three samples.
- A motion interval from `t` to `t+1`.
- A motion interval from `t+1` to `t+2`.
- No interval after `t+2`.

Python query bounds are inclusive. Rust range methods accept ordinary `RangeBounds`.

## Time Windows and Interval Ownership

Continuous checking of interval `t -> t+1` occurs only when both adjacent time steps are selected by the query range. A singleton window containing only `t` checks occupancy at `t`, not outgoing motion.

```python
# Discrete occupancy at t=10 only.
checker.collides_dynamic(trajectory, min_time=10, max_time=10)

# Occupancy at 10 and 11 plus motion interval 10 -> 11.
checker.collides_dynamic(trajectory, min_time=10, max_time=11)
```

A collision found between `t` and `t+1` is reported as `CollidesDynamic(t)`. The time is the interval start, not necessarily a discrete sample where overlap exists.

Static scene geometry is always checked first and is not suppressed by dynamic time bounds. A dynamic query that strikes static scene geometry still returns `CollidesDynamic(t)` because the status attributes the dynamic query's first colliding sample or interval.

Rust callers must provide valid ordered range bounds. The checker delegates to `BTreeSet::range`, whose invalid bound combinations can panic. Python validates `min_time <= max_time` and raises `ValueError` otherwise.

## Missing Occupancy and Varying Shapes

A time-varying trajectory can contain empty geometry to represent missing occupancy. An interval touching an empty endpoint is empty, so disappearance and reappearance do not create phantom motion across the gap.

For two non-empty but different endpoint shapes, CRCC merges conservative swept areas. This is an occupancy bound, not a defined geometric morph between shapes.

## Collision Status

Scene queries return one of three statuses:

| Status | Meaning |
| --- | --- |
| `NoCollision` | No selected static, discrete, or interval query collided. |
| `CollidesStatic` | Static scene geometry collided. Static checks take precedence. |
| `CollidesDynamic(t)` | A dynamic scene/query sample or interval was attributed to `t`. |

Dynamic time sets are ordered, so the checker returns the earliest selected dynamic result.

## Continuous Collision Detection

All engines include endpoint collision checks. Between endpoints, algorithms vary by backend and shape. Swept bounds reject obviously separated motion before narrower tests. Rotational and half-space handling may deliberately return conservative positives.

The safe interpretation is always:

- A negative continuous result certifies separation according to the implementation contract.
- A positive result requires application-level handling and may be an over-approximation.

## Engine Selection

The distributed Python build enables all engines and defaults to Parry. Rust defaults enable all engines but not Rayon. The repository tutorial launcher defaults to Rhusics so tutorial output can differ from an unqualified API call.

| Behavior | Parry | Rhusics | Collide |
| --- | --- | --- | --- |
| Finite primitives | Native shape queries | GJK-based | Convex/support-map based |
| Non-convex and holed polygons | Triangulated during conversion | Triangulated during conversion | Triangulated during conversion |
| Half-spaces | Supported | Analytic discrete; highly conservative moving checks | Analytic discrete; conservative CCD |
| Finite-shape distance | Native Parry distance | Shared geometric distance | Shared geometric distance |
| Exact circle tangency | Collision | Not collision under native GJK semantics | Collision |
| Translation CCD | Nonlinear shape cast and special cases | Native TOI for finite shapes | Analytic/sampled recursive checks |
| Rotation CCD | Nonlinear cast using shortest angular delta | Conservative motion bounds | Conservative sampled recursion with pose interpolation |

Do not serialize the Python engine objects by their current integer values; that mapping is an implementation detail.

## Choosing an Engine

Start with Parry unless a project has measured or compatibility-driven reasons to choose otherwise. Then test representative geometry and edge semantics with the chosen engine.

Use Rhusics or Collide when comparing backend behavior, reproducing an existing pipeline, or benchmarking a workload. If exact tangency matters, define the application's contact policy explicitly rather than assuming all engines agree.

## Batch Execution

Batch methods preserve input order. With Rayon enabled, the selected checker executes batches with fewer than 32 items sequentially and batches of 32 or more in the Rayon pool. The legacy `par_static` and `par_dynamic` Python names use the same automatic threshold; their names do not guarantee parallel execution.

Prepared queries avoid repeated conversion for one query reused many times. Batch methods improve execution amortization for many distinct queries. They solve different costs and can be chosen independently.

## Road Boundaries

Road-boundary helpers treat supplied lanelet polygons as drivable space and construct occupied geometry outside their union. The implementation simplifies the union, represents space outside its convex hull with half-spaces, and adds significant gaps or holes inside the hull.

The low-level `road_boundary([])` result is full space. The CommonRoad adapter intentionally skips boundary insertion when a lanelet network is empty, treating an absent map as no road constraint. This distinction matters when handling incomplete scenarios.

## Errors and Unsupported Operations

Rust returns `CrccError`; Python maps native CRCC errors to `ValueError`. Python conversion can also raise `TypeError` or `OverflowError` before native code runs.

Common unsupported cases include:

- Selecting an engine not enabled by Cargo features.
- Using a prepared query with a different engine.
- Asking for distance involving empty geometry.
- A backend conversion or shape combination the selected engine cannot represent.

Unsupported means the operation was not computed. It must not be interpreted as collision-free.
