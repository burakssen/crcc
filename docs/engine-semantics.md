# Engine Semantics and Known Divergences

CRCC's contract is that the `parry`, `rhusics`, and `collide` backends answer identically
for the same query. That holds for ordinary geometry, but three boundary behaviours are
intentionally backend-specific, and a few numerical conventions differ. This page records
them so benchmark numbers and correctness rows can be interpreted correctly.

## Exact-touch (tangency) semantics

| Discrete query at exact contact | parry | rhusics | collide |
| --- | --- | --- | --- |
| circle/circle, rect/rect, convex polygons | collides | **no collision** | collides |

Root cause: `collision-rs` GJK rejects with `v·d <= 0`, so origin-on-boundary counts as
outside; parry GJK and collide's inclusive sphere/margin tests count it as inside.
The fuzz suite encodes this as `expected_by_backend={"rhusics": false}` for tangent pairs.

## Half-space tolerance band

`rhusics` and `collide` test half-space contacts with a `1e-9` slack
(`n·p <= offset + 1e-9`); `parry` tests exactly (`n·p <= 0`). A shape whose deepest point
lies within 1e-9 *outside* a half-space reads as colliding on rhusics/collide and clear on
parry. Half-space vs half-space pairs are answered analytically on all three backends.

## Continuous collision (CCD) conventions

- Trajectories occupy geometry **only at sampled time steps**; motion is considered between
  consecutive samples of two active trajectories. An obstacle whose trajectory ends at `t`
  does not persist past `t`.
- Conservative answers: `collide` bisection returns `true` when an interval is too small to
  disprove contact, and `rhusics` reports `true` for any sweep involving half-spaces.
  These may over-report collisions by design; they never miss them.
- **Known parry limitation**: sweeps that cross a half-space boundary between two clear
  endpoints report no collision, because upstream shape-casting does not support
  half-spaces and silently skips those parts. Rhusics/collide conservatively report the
  crossing. Discrete queries are unaffected.

## Degenerate geometry

- `parry`: geometry it cannot represent (fewer than three non-collinear hull points after
  pruning, failed triangulation merges) poisons the whole object and surfaces as
  `Unsupported` at query time — fail-loud.
- `rhusics`/`collide`: degenerate triangulation output degrades silently to empty
  geometry — fail-silent. Domain validation in `SimpleCollisionObject::polygon` makes this
  unlikely in practice.
- Needle-thin convex polygons are kept as-is by all backends; parry uses an unpruned-hull
  fallback when its collinearity epsilon would collapse the polygon.

## Distance

Only `parry` computes distance natively; `rhusics`/`collide` use the shared geometric
fallback (`distance_geo`). Both clamp to non-negative values and agree analytically, but
magnitudes near contact can differ by ~1e-9 (GJK vs exact vertex arithmetic). For
half-space × half-space, parry reports `Unsupported` while the fallback returns the
analytic gap.

## Numerical tolerances in use

| Constant | Value | Where |
| --- | --- | --- |
| `HALF_SPACE_EPSILON` | 1e-9 | rhusics/collide/parry half-space contact slack |
| `ROTATION_EPSILON` | 1e-12 rad | parry/collide CCD "no rotation" gate |
| swept-area rotation gate | exact inequality | `DynamicObstacle` swept areas |
| road-boundary simplification | 1e-2 m | `road_boundary` union cleanup |
| hole artifact threshold | 1e-3 m² | `road_boundary` hole filter |

Each gate errs conservative individually; expect small cross-engine divergences only in
near-tangent configurations.
