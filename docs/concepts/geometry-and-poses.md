# Geometry and poses

CRCC represents occupied 2D geometry in local coordinates. A `Pose` supplies a world translation and counter-clockwise rotation in radians when geometry is queried or inserted into a trajectory.

## Shape types

The public object model includes circles, rectangles, triangles, polygons, half-spaces, `Empty`, `FullSpace`, and compounds. Rust applications should use `CollisionObject` constructors; Python applications construct the concrete classes exported from `crcc`.

| Geometry | Representation and use |
| --- | --- |
| Circle | Local center and strictly positive radius. |
| Rectangle | Full x/y dimensions, local center, and local orientation. Rust takes a `geo::Rect` and orientation. |
| Triangle | Three finite vertices with nonzero area. |
| Polygon | Exterior ring and optional interior rings; may be concave. |
| Half-space | Infinite occupied side $`\mathbf{n} \cdot \mathbf{x} \le c`$, where $`\mathbf{n}`$ is the outward normal and $`c`$ the offset, including its boundary. |
| Empty / full space | No occupancy / all occupancy; useful for missing data and unconstrained/unrepresentable regions. |
| Compound | Structural union of components, including mixtures of finite and infinite geometry. |

Python rejects zero rectangle dimensions. Rust checks finite rectangle coordinates and derived dimensions but currently accepts zero width/height. Lower-level Rust polygon wrappers perform fewer checks than `CollisionObject::polygon`; see [validation](../reference/errors-and-limits.md).

Polygon input may be non-convex and may have interior rings. CRCC classifies and decomposes polygons when converting them to a backend representation; it does not expose polygon Boolean operations as the shape-composition API.

More precisely, the domain constructor removes consecutive duplicate vertices and drops unusable interior rings, then validates retained coordinates and polygon topology and classifies the result. Backend conversion later decomposes non-convex/holed geometry. Invalid original holes can therefore disappear rather than raise an error.

```python
from crcc import Circle, Compound, HalfSpace, Polygon

region = Polygon(
    exterior=[(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)],
    interiors=[[(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]],
)
assert not Circle(0.25).collides(region)  # Circle lies inside the hole.
assert Circle(0.25, center=(1.5, 0.0)).collides(region)
assert Circle(0.25, center=(0.0, -1.0)).collides(
    HalfSpace.from_points((0.0, 0.0), (1.0, 0.0))
)  # Right side of the directed line: y <= 0.
assert not Circle(0.25).collides(Compound([]))
```

Half-space constructors normalize the normal **and offset together**. `HalfSpace((2, 0), 4)` means $`x \le 2`$; `from_coeffs(a, b, c)` means $`a x + b y \le c`$. Normalization preserves the inequality.

## Compounds are structural unions

A compound represents the union of its children for collision queries. Merging objects concatenates their components; it does not compute a new polygon boundary. Empty children are discarded, while a full-space child makes the union full space. An empty compound is equivalent to `Empty`.

## Poses

Shape centers and vertices are local to the object. For example, a circle whose local center is `(1, 0)` queried at translation `(3, 0)` has world center `(4, 0)` before accounting for rotation.

Rectangle orientation rotates around its local center. The query pose then rotates/translates the whole object around the object origin. For a rectangle centered away from `(0, 0)`, these are different operations.

Python `Pose` validates finite values. Rust exports `glamx::DPose2` as `Pose`; it is a type alias and does not add CRCC validation to pair-query arguments. Trajectory constructors check finite translations and rotation angles, but do not establish unit-length rotation components for arbitrarily constructed Rust poses. Supply valid rigid transforms.

```python
from crcc import Pose

translate = Pose.from_translation((3.0, 0.0))
local = Pose.from_translation((1.0, 0.0))
assert (translate * local).translation == (4.0, 0.0)
```

Composition applies the right operand first. Units are application-defined; all geometry and translations must use the same length units. CRCC time steps are indices, not seconds.

## Broad and narrow phase

Some scene and continuous queries use conservative bounds to reject clearly separated pairs before invoking a narrower backend query. A broad-phase overlap is only a candidate; narrow-phase work determines collision where supported. If the implementation cannot prove separation, a conservative continuous query may report a possible collision.

The exact broad- and narrow-phase algorithms vary by backend. See [backend trade-offs](backends.md) and the [query pipeline](../architecture/query-pipeline.md).
