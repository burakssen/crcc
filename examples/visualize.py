from pathlib import Path

import numpy as np
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.scenario.state import InitialState
from commonroad.visualization.draw_params import MPDrawParams, ShapeParams
from commonroad.visualization.mp_renderer import MPRenderer
from crcc.collision_object import Rectangle
from crcc.pose import Pose
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

from examples.utils import CAR_SIZE, scenario_time_steps

VISUALIZATION_SAMPLE_COUNT = 1_000
VISUALIZATION_RANDOM_SEED = 2026
ANIMATION_INTERVAL_MS = 100

COLOR_COLLIDED = "red"
COLOR_CLEAR = "green"

CAR_OBSTACLE_SHAPE = RectObstacleShape(width=CAR_SIZE[1], length=CAR_SIZE[0])
VISUALIZATION_PRESET_POSES = {
    "DEU_MerzenichRather-2_870_T-149.xml": [
        ((55.29, -1.99), 1.326),
        ((37.33, 4.07), -2.207),
    ],
}


def visualization_poses(scenario_path, pose_bounds, count=VISUALIZATION_SAMPLE_COUNT, seed=VISUALIZATION_RANDOM_SEED):
    """Generate deterministic poses for scenario animation, including preset points if available."""
    from examples.utils import sample_poses

    preset_poses = [
        Pose(translation, rotation)
        for translation, rotation in VISUALIZATION_PRESET_POSES.get(Path(scenario_path).name, [])
    ]
    if count <= len(preset_poses):
        return preset_poses[:count]

    rng = np.random.default_rng(seed)
    random_poses = sample_poses(count - len(preset_poses), pose_bounds, rng)
    return preset_poses + random_poses


def car_occupancy_for_pose(pose):
    return CAR_OBSTACLE_SHAPE.compute_occupancy_for_state(
        InitialState(position=np.array(pose.translation), orientation=pose.rotation)
    )


def update_collision_flags(collided_flags, collision_results):
    for index, collision_status in enumerate(collision_results):
        collided_flags[index] = collided_flags[index] or collision_status.collides
    return collided_flags


def run(scenario, checker, scenario_path, pose_bounds, sample_count=VISUALIZATION_SAMPLE_COUNT):
    """Run an animated scenario simulation showing cumulative ego-vehicle collisions."""
    poses = visualization_poses(scenario_path, pose_bounds, count=sample_count)
    samples = poses[:sample_count]
    car = Rectangle(*CAR_SIZE)
    positioned_cars = [(car, pose) for pose in samples]
    collided_flags = [False] * len(samples)
    time_steps = scenario_time_steps(scenario)

    fig, ax = plt.subplots()
    plot_limits = [pose_bounds[0][0], pose_bounds[1][0], pose_bounds[0][1], pose_bounds[1][1]]
    legend_handles = [
        Line2D([0], [0], color=COLOR_CLEAR, lw=4, label="No collision so far"),
        Line2D([0], [0], color=COLOR_COLLIDED, lw=4, label="Collided earlier or now"),
    ]

    def draw_frame(time_step):
        ax.clear()
        draw_params = MPDrawParams(time_begin=time_step, time_end=time_step)
        renderer = MPRenderer(draw_params=draw_params, plot_limits=plot_limits, ax=ax)
        scenario.draw(renderer, draw_params)
        for pose, collided in zip(samples, collided_flags, strict=True):
            rectangle = car_occupancy_for_pose(pose)
            color = COLOR_COLLIDED if collided else COLOR_CLEAR
            rectangle.draw(renderer, ShapeParams(facecolor=color, edgecolor=color, opacity=0.5))
        renderer.render()
        ax.legend(handles=legend_handles, loc="upper right")
        ax.set_title(f"Cumulative collision status through time step {time_step}")
        return ax.artists

    def initialize_frame():
        return draw_frame(time_steps[0])

    def update_frame(time_step):
        collision_results = checker.par_collides_static(
            positioned_cars,
            min_time=time_step,
            max_time=time_step,
        )
        update_collision_flags(collided_flags, collision_results)
        return draw_frame(time_step)

    animation = FuncAnimation(
        fig,
        update_frame,
        frames=time_steps,
        init_func=initialize_frame,
        interval=ANIMATION_INTERVAL_MS,
        repeat=False,
    )
    setattr(fig, "_crcc_animation", animation)
    plt.show()
    return animation
