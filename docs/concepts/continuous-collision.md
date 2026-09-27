# Continuous collision detection

Discrete queries compare geometry at supplied poses. Continuous queries inspect two endpoint poses for each object and the motion between them. Use continuous checking when endpoint-only tests could miss tunneling.

## Result contract

The Boolean contract is asymmetric:

- `false` certifies separation for the complete interval according to the selected implementation.
- `true` means collision is possible. Conservative bounds or backend limitations can produce a positive even when the exact motion is clear.

Treat a positive as a candidate requiring application-level handling; do not interpret unsupported operations or errors as clear space.

## Dynamic trajectories

Scene checkers apply continuous checks between adjacent active samples. Both endpoints must be included by a Python time window for the interval to be considered. When an interval collides, `CollidesDynamic(t)` attributes it to its start time `t`.

For time-varying shapes, CRCC combines conservative swept bounds for the endpoint occupancies. This bounds motion; it does not define a geometric morph from one shape to another. Empty endpoint geometry suppresses the interval.

## Query stages

Implementations may first test conservative swept bounds (broad phase), then perform a backend-specific motion query (narrow phase). Bounds may over-approximate rotating geometry. In particular, some engines deliberately report a possible collision when they cannot disprove one. Algorithm and geometry support differ by backend; see [backend trade-offs](backends.md).

For a working example, see [continuous pair queries](../guides/pair-queries.md#check-motion-between-poses) and [dynamic scenes](../guides/scenes-and-trajectories.md#add-a-fixed-shape-trajectory).
