from dataclasses import dataclass

import numpy as np
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Circle, Rectangle
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.patches import Circle as CirclePatch, Rectangle as RectanglePatch
from matplotlib.transforms import Affine2D

FEATURE_ANIMATION_INTERVAL_MS = 180
FEATURE_FRAME_HOLD_COUNT = 2
FEATURE_TIME_STEPS = list(range(17))
FEATURE_PLOT_LIMITS = [-5.0, 5.0, -3.0, 3.0]

COLOR_COLLIDED = "red"
COLOR_CLEAR = "green"
COLOR_STATIC = "0.25"


@dataclass(frozen=True)
class FeatureShape:
    kind: str
    center: tuple[float, float]
    radius: float | None = None
    length: float | None = None
    width: float | None = None
    orientation: float = 0.0


@dataclass(frozen=True)
class FeatureExample:
    title: str
    checker: object
    dynamic_obstacle: object
    time_steps: list[int]
    static_shapes: list[FeatureShape]
    dynamic_shapes: list[FeatureShape]
    plot_limits: list[float]


def feature_path_centers(time_steps):
    if len(time_steps) == 1:
        return [(0.0, 0.0)]
    return [(-4.0 + 8.0 * index / (len(time_steps) - 1), 0.0) for index, _time_step in enumerate(time_steps)]


def feature_time_variant_shape(center, index, count):
    progress = index / (count - 1)
    if progress < 0.25:
        return FeatureShape("circle", center, radius=0.45)
    if progress < 0.75:
        width_scale = 1.0 - abs(progress - 0.5) / 0.25
        return FeatureShape(
            "rectangle",
            center,
            length=1.1 + 1.1 * width_scale,
            width=0.7 + 0.5 * width_scale,
            orientation=0.2 * np.sin(progress * np.pi * 2.0),
        )
    return FeatureShape("circle", center, radius=0.45)


def feature_example_fixed_dynamic(engine):
    time_steps = FEATURE_TIME_STEPS
    centers = feature_path_centers(time_steps)
    poses = [Pose.from_translation(center) for center in centers]
    dynamic_obstacle = DynamicObstacle(Rectangle(1.4, 0.8), poses, time_steps[0])
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(1.0)).build()
    return FeatureExample(
        title="Fixed-shape dynamic obstacle",
        checker=checker,
        dynamic_obstacle=dynamic_obstacle,
        time_steps=time_steps,
        static_shapes=[FeatureShape("circle", (0.0, 0.0), radius=1.0)],
        dynamic_shapes=[
            FeatureShape("rectangle", pose.translation, length=1.4, width=0.8, orientation=pose.rotation)
            for pose in poses
        ],
        plot_limits=FEATURE_PLOT_LIMITS,
    )


def feature_example_time_variant(engine):
    time_steps = FEATURE_TIME_STEPS
    dynamic_shapes = [
        feature_time_variant_shape(center, index, len(time_steps))
        for index, center in enumerate(feature_path_centers(time_steps))
    ]
    obstacles = []
    for shape in dynamic_shapes:
        if shape.kind == "circle":
            assert shape.radius is not None
            obstacles.append(Circle(shape.radius, shape.center))
        else:
            assert shape.length is not None
            assert shape.width is not None
            obstacles.append(
                Rectangle(
                    shape.length,
                    shape.width,
                    shape.orientation,
                    shape.center,
                )
            )
    dynamic_obstacle = DynamicObstacle.from_time_variant(obstacles, time_offset=time_steps[0])
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(0.75)).build()
    return FeatureExample(
        title="Time-variant dynamic obstacle",
        checker=checker,
        dynamic_obstacle=dynamic_obstacle,
        time_steps=time_steps,
        static_shapes=[FeatureShape("circle", (0.0, 0.0), radius=0.75)],
        dynamic_shapes=dynamic_shapes,
        plot_limits=FEATURE_PLOT_LIMITS,
    )


def feature_animation_frames(examples):
    time_steps = []
    for example in examples:
        time_steps.extend(example.time_steps)
    return [time_step for time_step in sorted(set(time_steps)) for _ in range(FEATURE_FRAME_HOLD_COUNT)]


def draw_feature_shape(ax, shape, color, alpha, *, edgecolor=None, linewidth=1.5):
    if shape.kind == "circle":
        assert shape.radius is not None
        patch = CirclePatch(
            shape.center,
            shape.radius,
            facecolor=color,
            edgecolor=edgecolor or color,
            linewidth=linewidth,
            alpha=alpha,
        )
        ax.add_patch(patch)
        return patch
    if shape.kind == "rectangle":
        assert shape.length is not None
        assert shape.width is not None
        lower_left = (-shape.length / 2.0, -shape.width / 2.0)
        patch = RectanglePatch(
            lower_left,
            shape.length,
            shape.width,
            facecolor=color,
            edgecolor=edgecolor or color,
            linewidth=linewidth,
            alpha=alpha,
        )
        transform = Affine2D().rotate(shape.orientation).translate(*shape.center) + ax.transData
        patch.set_transform(transform)
        ax.add_patch(patch)
        return patch
    raise ValueError(f"Unsupported feature shape: {shape.kind}")


def draw_feature_frame(ax, example, time_step):
    ax.clear()
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(example.plot_limits[0], example.plot_limits[1])
    ax.set_ylim(example.plot_limits[2], example.plot_limits[3])
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid(True, color="0.85", linewidth=0.8)

    artists = []
    # Draw trail
    centers = np.array([shape.center for shape in example.dynamic_shapes])
    line = ax.plot(centers[:, 0], centers[:, 1], color="0.45", linestyle="--", linewidth=1.5, alpha=0.85)[0]
    points = ax.scatter(centers[:, 0], centers[:, 1], s=18, color="0.45", alpha=0.6)
    artists.extend([line, points])

    # Draw static obstacles
    for shape in example.static_shapes:
        artists.append(draw_feature_shape(ax, shape, COLOR_STATIC, 0.18, edgecolor=COLOR_STATIC, linewidth=2.5))

    # Draw current dynamic shape
    shape = example.dynamic_shapes[example.time_steps.index(time_step)]
    status = example.checker.collides_dynamic(example.dynamic_obstacle, min_time=time_step, max_time=time_step)
    color = COLOR_COLLIDED if status.collides else COLOR_CLEAR
    linewidth = 3.0 if status.collides else 2.0
    artists.append(draw_feature_shape(ax, shape, color, 0.75, edgecolor=color, linewidth=linewidth))

    # Current position marker
    marker = ax.scatter([shape.center[0]], [shape.center[1]], s=42, color=color, edgecolors="black", linewidths=0.8)
    artists.append(marker)

    status_label = "COLLISION" if status.collides else "clear"
    ax.set_title(f"{example.title}\ntime step {time_step}: {status_label}", color=color if status.collides else "black")
    return artists


def run(engine: CollisionEngine):
    """Run features animation examples demonstrating dynamic and time-variant obstacle collision queries."""
    examples = [
        feature_example_fixed_dynamic(engine),
        feature_example_time_variant(engine),
    ]
    frames = feature_animation_frames(examples)
    fig, axes = plt.subplots(1, len(examples), figsize=(13.5, 5.6), squeeze=False)
    axes = axes[0]
    legend_handles = [
        Line2D([0], [0], color=COLOR_STATIC, lw=4, label="Static checker obstacle"),
        Line2D([0], [0], color=COLOR_CLEAR, lw=4, label="Dynamic obstacle clear"),
        Line2D([0], [0], color=COLOR_COLLIDED, lw=4, label="Dynamic obstacle colliding"),
        Line2D([0], [0], color="0.45", lw=1.5, ls="--", label="Obstacle path"),
    ]

    def draw_frame(frame_index):
        artists = []
        for ax, example in zip(axes, examples, strict=True):
            time_step = frames[frame_index]
            artists.extend(draw_feature_frame(ax, example, time_step))
        axes[-1].legend(handles=legend_handles, loc="upper right", framealpha=0.95)
        return artists

    animation = FuncAnimation(
        fig,
        draw_frame,
        frames=len(frames),
        interval=FEATURE_ANIMATION_INTERVAL_MS,
        repeat=True,
    )
    setattr(fig, "_crcc_animation", animation)
    plt.show()
    return animation
