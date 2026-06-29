from crcc.collision_checker import CollisionEngine
from matplotlib import pyplot as plt

from examples.drawing import collision_object, demo_shapes, draw_visual_shape

COLOR_COLLIDED = "#dc2626"
COLOR_CLEAR = "#059669"
COLOR_CELL = "#f3f4f6"
COLOR_MUTED = "#6b7280"


def pair_display_centers(left, right, collided: bool):
    if not collided:
        return (-2.0, 0.05), (2.0, 0.05)
    if "empty" in {left.kind, right.kind}:
        return (-0.45, 0.05), (0.45, 0.05)
    if "full_space" in {left.kind, right.kind}:
        return (0.0, 0.05), (0.0, 0.05)
    if "half_space" in {left.kind, right.kind}:
        return (0.0, 0.0), (0.0, 0.0)
    return (-0.10, 0.04), (0.10, 0.04)


def pair_case(left, right, collided: bool, engine: CollisionEngine):
    left_center, right_center = pair_display_centers(left, right, collided)
    try:
        actual = collision_object(left, left_center).collides(collision_object(right, right_center), engine=engine)
    except Exception:
        actual = False
    expected = collided
    supported = actual == expected
    return {
        "left_center": left_center,
        "right_center": right_center,
        "expected": expected,
        "actual": actual,
        "supported": supported,
    }


def pair_collision_cases(engine: CollisionEngine):
    shapes = demo_shapes()
    cases = []
    for left in shapes:
        for right in shapes:
            cases.append(
                {
                    "left": left,
                    "right": right,
                    "hit": pair_case(left, right, True, engine),
                    "clear": pair_case(left, right, False, engine),
                }
            )
    return tuple(cases)


def collision_matrix(engine: CollisionEngine):
    shapes = demo_shapes()
    labels = tuple(shape.label for shape in shapes)
    matrix = []
    for left in shapes:
        row = []
        for right in shapes:
            try:
                row.append(collision_object(left).collides(collision_object(right), engine=engine))
            except Exception:
                row.append(False)
        matrix.append(tuple(row))
    return labels, tuple(matrix)


def draw_collision_matrix(engine: CollisionEngine):
    shapes = demo_shapes()
    labels, matrix = collision_matrix(engine)
    cases = pair_collision_cases(engine)
    fig, axes = plt.subplots(len(shapes), len(shapes), figsize=(16, 16), layout="constrained")
    fig.suptitle("Collision Matrix: Shape Type Against Shape Type", fontsize=14, fontweight="bold")

    artists = []
    for y, left in enumerate(shapes):
        for x, right in enumerate(shapes):
            ax = axes[y][x]
            ax.set_aspect("equal")
            ax.set_xlim(-1.25, 1.25)
            ax.set_ylim(-1.1, 1.1)
            ax.set_xticks([])
            ax.set_yticks([])
            collided = matrix[y][x]
            case = cases[y * len(shapes) + x]
            ax.set_facecolor(COLOR_CELL)
            artists.extend(_draw_case(ax, left, right, case["hit"], -0.58, "hit", COLOR_COLLIDED))
            artists.extend(_draw_case(ax, left, right, case["clear"], 0.58, "clear", COLOR_CLEAR))
            if not collided:
                artists.append(ax.text(0.0, 0.94, "matrix: clear", ha="center", va="center", fontsize=5.8, color=COLOR_MUTED))
            if y == 0:
                ax.set_title(labels[x], fontsize=7, pad=2)
            if x == 0:
                ax.set_ylabel(labels[y], fontsize=7, rotation=0, ha="right", va="center", labelpad=26)
            for spine in ax.spines.values():
                spine.set_color("#d1d5db")
                spine.set_linewidth(0.6)
    return fig, axes, artists


def _draw_case(ax, left, right, case, x_offset: float, label: str, color: str):
    artists = []
    if not case["supported"]:
        artists.append(ax.text(x_offset, 0.02, "n/a", ha="center", va="center", fontsize=7, color=COLOR_MUTED))
    else:
        left_center = (case["left_center"][0] * 0.28 + x_offset, case["left_center"][1])
        right_center = (case["right_center"][0] * 0.28 + x_offset, case["right_center"][1])
        artists.extend(draw_visual_shape(ax, left, left_center, color, 0.42, linewidth=0.9, scale=0.24))
        artists.extend(draw_visual_shape(ax, right, right_center, color, 0.42, linewidth=0.9, scale=0.24))
    artists.append(ax.text(x_offset, -0.84, label, ha="center", va="center", fontsize=6.5, color=color))
    return artists


def run(engine: CollisionEngine):
    """Show a visual collision matrix for all public shape categories."""
    fig, _ax, _artists = draw_collision_matrix(engine)
    plt.show()
    return fig
