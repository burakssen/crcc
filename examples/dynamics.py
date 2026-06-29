from dataclasses import dataclass

import numpy as np
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

from examples.drawing import VisualShape, collision_object, draw_visual_shape
from examples.utils import scenario_time_steps

FRAME_HOLD_COUNT = 2
ANIMATION_INTERVAL_MS = 160
COLOR_COLLIDED = "#dc2626"
COLOR_CLEAR = "#059669"
COLOR_PATH = "#2563eb"


@dataclass(frozen=True)
class ScenarioDynamicExample:
    checker: object
    dynamic_obstacle: DynamicObstacle
    time_steps: tuple[int, ...]
    poses: tuple[Pose, ...]
    visual_shapes: tuple[VisualShape, ...]
    plot_limits: tuple[float, float, float, float]
    first_collision_time: int | None


def scenario_path(pose_bounds, count: int):
    lower, upper = pose_bounds
    y_mid = (lower[1] + upper[1]) / 2.0
    x_start = lower[0] + (upper[0] - lower[0]) * 0.08
    x_end = upper[0] - (upper[0] - lower[0]) * 0.08
    return tuple(Pose((x, y_mid + np.sin(i / max(1, count - 1) * np.pi) * 2.0), 0.0) for i, x in enumerate(np.linspace(x_start, x_end, count)))


def scenario_shape_sequence(count: int):
    base = (
        VisualShape("circle", (1.2,), "circle"),
        VisualShape("rectangle", (4.5, 2.0, 0.15), "rectangle"),
        VisualShape("compound", (), "compound"),
        VisualShape("polygon", (((-2.0, -0.9), (1.7, -1.0), (2.0, 0.8), (-1.2, 1.1), (-2.0, -0.9))), "polygon"),
    )
    return tuple(base[min(len(base) - 1, int(i / max(1, count - 1) * len(base)))] for i in range(count))


def scenario_dynamic_example(scenario, checker, pose_bounds, max_steps: int = 40):
    available_steps = scenario_time_steps(scenario)
    time_steps = tuple(available_steps[:max_steps])
    poses = scenario_path(pose_bounds, len(time_steps))
    visual_shapes = scenario_shape_sequence(len(time_steps))
    objects = [collision_object(shape) for shape in visual_shapes]
    dynamic_obstacle = DynamicObstacle.from_time_variant(objects, time_offset=time_steps[0], positions=poses)
    statuses = [checker.collides_dynamic(dynamic_obstacle, min_time=t, max_time=t) for t in time_steps]
    first_collision = next((t for t, status in zip(time_steps, statuses, strict=True) if status.collides), None)
    return ScenarioDynamicExample(
        checker,
        dynamic_obstacle,
        time_steps,
        poses,
        visual_shapes,
        (pose_bounds[0][0], pose_bounds[1][0], pose_bounds[0][1], pose_bounds[1][1]),
        first_collision,
    )


def animation_frames(example: ScenarioDynamicExample):
    return [step for step in example.time_steps for _ in range(FRAME_HOLD_COUNT)]


def draw_frame(ax, scenario, example: ScenarioDynamicExample, time_step: int):
    ax.clear()
    draw_params = MPDrawParams(time_begin=time_step, time_end=time_step)
    renderer = MPRenderer(draw_params=draw_params, plot_limits=list(example.plot_limits), ax=ax)
    scenario.draw(renderer, draw_params)
    renderer.render()

    index = example.time_steps.index(time_step)
    centers = np.array([pose.translation for pose in example.poses])
    artists = [
        ax.plot(centers[:, 0], centers[:, 1], color=COLOR_PATH, linestyle="--", linewidth=1.8, alpha=0.8)[0],
        ax.scatter(centers[:, 0], centers[:, 1], s=16, color=COLOR_PATH, alpha=0.45),
    ]
    status = example.checker.collides_dynamic(example.dynamic_obstacle, min_time=time_step, max_time=time_step)
    color = COLOR_COLLIDED if status.collides else COLOR_CLEAR
    artists.extend(draw_visual_shape(ax, example.visual_shapes[index], example.poses[index].translation, color, 0.72, linewidth=2.6))
    ax.set_title(
        f"Scenario time-variant dynamic query | t={time_step} | {'collision' if status.collides else 'clear'}"
    )
    return artists


def run(scenario, checker, scenario_path, pose_bounds):
    """Animate a time-variant dynamic query across a CommonRoad scenario."""
    example = scenario_dynamic_example(scenario, checker, pose_bounds)
    frames = animation_frames(example)
    fig, ax = plt.subplots(figsize=(11, 7))
    legend_handles = [
        Line2D([0], [0], color=COLOR_PATH, lw=2, ls="--", label="generated path"),
        Line2D([0], [0], color=COLOR_CLEAR, lw=4, label="clear"),
        Line2D([0], [0], color=COLOR_COLLIDED, lw=4, label="collision"),
    ]

    def update(frame_index):
        artists = draw_frame(ax, scenario, example, frames[frame_index])
        ax.legend(handles=legend_handles, loc="upper right")
        return artists

    animation = FuncAnimation(fig, update, frames=len(frames), interval=ANIMATION_INTERVAL_MS, repeat=True)
    setattr(fig, "_crcc_animation", animation)
    fig.suptitle(f"{scenario_path}: generated time-variant dynamic shape")
    plt.show()
    return animation
