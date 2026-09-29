# Continuous collision detection

Discrete queries compare geometry at supplied poses. Continuous queries inspect two endpoint poses for each object and the motion between them. Use continuous checking when endpoint-only tests could miss tunneling.

## Result contract

The Boolean contract is asymmetric:

- `false` reports separation for the complete interval according to the selected implementation and contact convention, assuming valid geometry/rigid poses and representable floating-point calculations.
- `true` means collision is possible. Conservative bounds or backend limitations can produce a positive even when the exact motion is clear.

Treat a positive as a candidate requiring application-level handling; do not interpret unsupported operations or errors as clear space.

This is a Boolean query, not a returned time of impact, penetration depth, or contact manifold. Finite inputs alone do not guarantee exact numerical arithmetic. See [numerical robustness](numerical-robustness.md).

## Dynamic trajectories

Scene checkers apply continuous checks between adjacent active samples. Static-query windows require both selected endpoints; dynamic-query windows select interval starts and may include the outgoing interval beyond the upper bound. See [query windows](scenes-and-time.md#query-windows-and-results). When an interval collides, `CollidesDynamic(t)` attributes it to its start time `t`.

For fixed geometry, the narrow query receives the shape and adjacent poses. For time-varying shapes, CRCC combines the sweeps of **both endpoint shapes across the same pair of poses**, then tests that world-space union with identity poses. This does not define a geometric morph or preserve exact time-resolved occupancy. Even equal shapes supplied through `from_time_variant` need not give the precision of a fixed-shape trajectory. Empty endpoint geometry suppresses the interval; discrete nonempty endpoint occupancy remains queryable.

## Query stages

Implementations may first test conservative swept bounds (broad phase), then perform a backend-specific motion query (narrow phase). Bounds may over-approximate rotating geometry. In particular, some engines deliberately report a possible collision when they cannot disprove one. Algorithm and geometry support differ by backend; see [backend trade-offs](backends.md).

## Swept geometry

`CollisionObject::swept_areas` returns one bound per adjacent pose pair (zero for fewer than two poses). Trajectory constructors cache these bounds:

- Translating circles: an axis-aligned endpoint-center envelope expanded by the radius.
- Finite polygonal geometry without changed rotation: convex hull of endpoint geometry; concavities and holes can be filled.
- Rotating finite geometry: an axis-aligned translation envelope expanded by distance from the object origin, potentially much looser than actual occupancy.
- Translating half-spaces: the larger projected offset; changed rotation yields full space.
- Unrepresentable checked bounds: full-space fallback.

Scene broad-phase overlap is a backend collision check of these bounds. There is no common scene-wide spatial index. A fixed-shape narrow query can reject a candidate, while a time-varying swept-union query cannot recover exact shape evolution.

Endpoint rotations do not encode multiple revolutions. Parry wraps angular displacement to the shortest path; Collide's current interpolation linearly interpolates principal angles and can take the long route across the $`-\pi/\pi`$ boundary. Rhusics uses conservative radial bounds for rotation. Specify intermediate samples when the intended path cannot be represented by two endpoints.

For a working example, see [continuous pair queries](../guides/pair-queries.md#check-motion-between-poses) and [dynamic scenes](../guides/scenes-and-trajectories.md#add-a-fixed-shape-trajectory).
