from dataclasses import dataclass

from crcc.collision_object import Circle, Compound, Empty, FullSpace, HalfSpace, Polygon, Rectangle, Triangle
from matplotlib.patches import Circle as CirclePatch, Polygon as PolygonPatch, Rectangle as RectanglePatch
from matplotlib.transforms import Affine2D


@dataclass(frozen=True)
class VisualShape:
    kind: str
    params: tuple
    label: str


def demo_shapes():
    return (
        VisualShape("circle", (0.8,), "Circle"),
        VisualShape("rectangle", (1.6, 0.9, 0.25), "Rectangle"),
        VisualShape("triangle", (((-0.8, -0.5), (0.8, -0.4), (0.0, 0.8))), "Triangle"),
        VisualShape("polygon", (((-0.9, -0.6), (0.6, -0.7), (0.9, 0.2), (0.0, 0.9), (-0.8, 0.3), (-0.9, -0.6))), "Polygon"),
        VisualShape(
            "polygon_hole",
            (
                ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0)),
                (((-0.3, -0.3), (0.3, -0.3), (0.3, 0.3), (-0.3, 0.3), (-0.3, -0.3)),),
            ),
            "Polygon+hole",
        ),
        VisualShape("compound", (), "Compound"),
        VisualShape("half_space", (), "HalfSpace"),
        VisualShape("full_space", (), "FullSpace"),
        VisualShape("empty", (), "Empty"),
    )


def collision_object(shape: VisualShape, center=(0.0, 0.0)):
    x, y = center
    if shape.kind == "circle":
        return Circle(shape.params[0], center)
    if shape.kind == "rectangle":
        length, width, orientation = shape.params
        return Rectangle(length, width, orientation, center)
    if shape.kind == "triangle":
        return Triangle(*_offset_points(shape.params, center))
    if shape.kind == "polygon":
        return Polygon(_offset_points(shape.params, center))
    if shape.kind == "polygon_hole":
        exterior, interiors = shape.params
        return Polygon(_offset_points(exterior, center), [_offset_points(points, center) for points in interiors])
    if shape.kind == "compound":
        return Compound([Circle(0.45, (x - 0.35, y)), Rectangle(0.8, 0.5, 0.25, (x + 0.35, y))])
    if shape.kind == "half_space":
        return HalfSpace((0.0, 1.0), y - 0.4)
    if shape.kind == "full_space":
        return FullSpace()
    if shape.kind == "empty":
        return Empty()
    raise ValueError(f"unknown shape kind: {shape.kind}")


def draw_visual_shape(
    ax,
    shape: VisualShape,
    center,
    color,
    alpha=0.65,
    *,
    edgecolor=None,
    linewidth=1.8,
    scale=1.0,
    zorder=10,
):
    edgecolor = edgecolor or color
    x, y = center
    if shape.kind == "circle":
        patch = CirclePatch(
            center,
            shape.params[0] * scale,
            facecolor=color,
            edgecolor=edgecolor,
            alpha=alpha,
            linewidth=linewidth,
            zorder=zorder,
        )
        ax.add_patch(patch)
        return [patch]
    if shape.kind == "rectangle":
        length, width, orientation = shape.params
        length *= scale
        width *= scale
        patch = RectanglePatch(
            (-length / 2, -width / 2),
            length,
            width,
            facecolor=color,
            edgecolor=edgecolor,
            alpha=alpha,
            linewidth=linewidth,
            zorder=zorder,
        )
        patch.set_transform(Affine2D().rotate(orientation).translate(x, y) + ax.transData)
        ax.add_patch(patch)
        return [patch]
    if shape.kind == "triangle":
        patch = PolygonPatch(
            _offset_points(shape.params, center, scale),
            closed=True,
            facecolor=color,
            edgecolor=edgecolor,
            alpha=alpha,
            linewidth=linewidth,
            zorder=zorder,
        )
        ax.add_patch(patch)
        return [patch]
    if shape.kind in {"polygon", "polygon_hole"}:
        points = shape.params[0] if shape.kind == "polygon_hole" else shape.params
        patch = PolygonPatch(
            _offset_points(points, center, scale),
            closed=True,
            facecolor=color,
            edgecolor=edgecolor,
            alpha=alpha,
            linewidth=linewidth,
            zorder=zorder,
        )
        ax.add_patch(patch)
        return [patch]
    if shape.kind == "compound":
        artists = []
        artists.extend(
            draw_visual_shape(
                ax,
                VisualShape("circle", (0.45,), "part"),
                (x - 0.35 * scale, y),
                color,
                alpha,
                edgecolor=edgecolor,
                linewidth=linewidth,
                scale=scale,
                zorder=zorder,
            )
        )
        artists.extend(
            draw_visual_shape(
                ax,
                VisualShape("rectangle", (0.8, 0.5, 0.25), "part"),
                (x + 0.35 * scale, y),
                color,
                alpha,
                edgecolor=edgecolor,
                linewidth=linewidth,
                scale=scale,
                zorder=zorder,
            )
        )
        return artists
    if shape.kind == "half_space":
        line = ax.plot(
            [x - 0.9 * scale, x + 0.9 * scale],
            [y - 0.4 * scale, y - 0.4 * scale],
            color=edgecolor,
            linewidth=linewidth,
            zorder=zorder,
        )[0]
        fill = ax.fill_between(
            [x - 0.9 * scale, x + 0.9 * scale],
            y - 0.4 * scale,
            y + 0.9 * scale,
            color=color,
            alpha=0.18,
            zorder=zorder,
        )
        return [line, fill]
    if shape.kind == "full_space":
        size = 1.8 * scale
        patch = RectanglePatch(
            (x - size / 2, y - size / 2),
            size,
            size,
            facecolor=color,
            edgecolor=edgecolor,
            alpha=0.14,
            linewidth=linewidth,
            zorder=zorder,
        )
        ax.add_patch(patch)
        return [patch]
    if shape.kind == "empty":
        return [ax.text(x, y, "empty", ha="center", va="center", fontsize=8, color=edgecolor, zorder=zorder)]
    raise ValueError(f"unknown shape kind: {shape.kind}")


def _offset_points(points, center, scale=1.0):
    x, y = center
    return [(px * scale + x, py * scale + y) for px, py in points]
