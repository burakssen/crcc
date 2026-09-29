# Numerical robustness

CRCC uses double-precision coordinates and poses. It inherits algorithms from `geo`, `glamx`, and the selected collision backend; it does not apply one global configurable epsilon or promise exact arithmetic.

## Contact is backend-dependent

The tested exactly tangent circle pair collides in Parry and Collide but not in Rhusics GJK. This observation is not a guarantee for every tangent shape combination. Nearly touching shapes can also differ because decomposition and support-map algorithms differ.

Half-spaces include their boundary. Analytic half-space pair tests use a $`10^{-9}`$ slack. Rhusics/Collide half-space-versus-finite tests also use $`10^{-9}`$; Parry delegates other supported combinations. This tolerance is in coordinate units for normalized normals. Half-space pair separation requires exact antiparallel normals; slightly nonparallel infinite half-spaces intersect, possibly very far away.

Distance and collision are not interchangeable predicates: the shared distance fallback has no matching collision slack. A small positive distance can coexist with half-space collision; Rhusics circle tangency can have distance zero without collision.

## Input validation and degeneracy

Use the regular [geometry constructors](geometry-and-poses.md), and read their [validation rules](../reference/errors-and-limits.md). Polygon construction sanitizes duplicate vertices and unusable holes before validating topology. Triangles reject computed zero area, without a user-configurable minimum-area threshold. Rust zero-size rectangles are accepted despite Python rejecting them; avoid depending on useful backend behavior for them.

Rust pair and swept-area APIs accept `Pose` directly without validating it. Trajectories check finite translation and rotation angle, not arbitrary rotation-component normalization. Construct unit rotations using the pose/rotation library's normal constructors. Python `Pose` adds finite-value checks.

## Finite is not necessarily representable

Very large finite coordinates/radii can overflow derived extents, distances, projections, or squared values. Some checked paths reject them; others fall back to full-space bounds or rely on backend arithmetic. There is no universal valid coordinate range, cross-backend precision bound, or panic-free guarantee for arbitrary malformed lower-level inputs. Use a consistent, moderate coordinate scale and exercise application-specific boundary cases.

## Motion and preprocessing

Conservative swept geometry can fill polygon holes/concavities and greatly overestimate rotation. Changed half-space rotation yields full space. Parry and Collide use different angular interpolation conventions near $`-\pi/\pi`$; endpoint poses cannot encode multiple turns. See [continuous queries](continuous-collision.md).

Road-boundary construction simplifies at `0.01` coordinate units and ignores regions of area at most `0.001` squared units. These approximations can remove real boundary detail; full-space error fallback does not make preprocessing an exact conservative complement of the original road.

The tests establish particular examples and regressions, not a formal proof for all floating-point inputs. Treat unsupported operations as errors and conservative positives as possible collisions, never as a numerical confidence score.
