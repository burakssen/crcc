import argparse
import time
from enum import Enum
from pathlib import Path

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.state import InitialState
from commonroad.visualization.draw_params import MPDrawParams, ShapeParams
from commonroad.visualization.mp_renderer import MPRenderer
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Circle, Polygon, Rectangle
from crcc.commonroad import create_collision_checker_from_scenario
from crcc.pose import Pose
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

SCENARIO_PATH = "scenarios/DEU_MerzenichRather-2_870_T-149.xml"
ENGINE = CollisionEngine.Rhusics
CAR_SIZE = (4.5, 2.0)
POSE_BOUNDS_PADDING = 5.0
BENCHMARK_SAMPLE_COUNT = 100_000
VISUALIZATION_SAMPLE_COUNT = 1_000
VISUALIZATION_RANDOM_SEED = 2026
DIAGNOSTIC_SAMPLE_COUNT = 1_000
MERGE_SCENARIO_NAME = "DEU_MerzenichRather-2_870_T-149.xml"
ANIMATION_INTERVAL_MS = 100
COLOR_COLLIDED = "red"
COLOR_CLEAR = "green"
CAR_OBSTACLE_SHAPE = RectObstacleShape(width=CAR_SIZE[1], length=CAR_SIZE[0])
VISUALIZATION_PRESET_POSES = {
    MERGE_SCENARIO_NAME: [
        ((55.29, -1.99), 1.326),
        ((37.33, 4.07), -2.207),
    ],
}


class ExampleAction(Enum):
    GEOMETRY = "geometry"
    SMOKE = "smoke"
    BENCHMARK = "benchmark"
    VISUALIZE = "visualize"
    ALL = "all"


ENGINE_CHOICES = {
    "parry": CollisionEngine.Parry,
    "rhusics": CollisionEngine.Rhusics,
}
ACTION_CHOICES = {action.value: action for action in ExampleAction}


def main(argv=None):
    args = parse_args(argv)
    action = args.action or prompt_for_action()
    run_action(action, args.scenario, args.engine)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Run structured CommonRoad collision checker examples.")
    parser.add_argument(
        "action",
        nargs="?",
        choices=sorted(ACTION_CHOICES),
        help="example to run; omit to choose from an interactive menu",
    )
    parser.add_argument(
        "--scenario",
        default=SCENARIO_PATH,
        help=f"CommonRoad scenario XML path (default: {SCENARIO_PATH})",
    )
    parser.add_argument(
        "--engine",
        choices=sorted(ENGINE_CHOICES),
        default=engine_name(ENGINE),
        type=str.lower,
        help="collision engine to use for scenario-based examples",
    )
    args = parser.parse_args(argv)
    if args.action is not None:
        args.action = ACTION_CHOICES[args.action]
    args.engine = ENGINE_CHOICES[args.engine]
    return args


def prompt_for_action(prompt=input):
    actions = list(ExampleAction)
    print("Select an example:")
    for index, action in enumerate(actions, start=1):
        print(f"{index}. {action.value}")

    while True:
        selection = prompt("Choice: ").strip().lower()
        if selection.isdigit():
            index = int(selection)
            if 1 <= index <= len(actions):
                return actions[index - 1]
        for action in actions:
            if selection == action.value:
                return action
        print(f"Enter a number from 1 to {len(actions)} or one of: {', '.join(action.value for action in actions)}")


def engine_name(engine):
    for name, engine_choice in ENGINE_CHOICES.items():
        if engine_choice == engine:
            return name
    raise ValueError(f"Unsupported collision engine: {engine}")


def run_action(action, scenario_path: str, engine):
    if action == ExampleAction.GEOMETRY:
        run_geometry_examples()
        return

    scenario, checker = load_collision_checker(scenario_path, engine)
    pose_bounds = scenario_pose_bounds(scenario)

    if action == ExampleAction.SMOKE:
        run_smoke_checks(scenario, checker, scenario_path, pose_bounds)
    elif action == ExampleAction.BENCHMARK:
        run_parallel_benchmark(checker, pose_bounds)
    elif action == ExampleAction.VISUALIZE:
        run_visualization(scenario, checker, scenario_path, pose_bounds)
    elif action == ExampleAction.ALL:
        run_smoke_checks(scenario, checker, scenario_path, pose_bounds)
        run_geometry_examples()
        run_parallel_benchmark(checker, pose_bounds)
        run_visualization(scenario, checker, scenario_path, pose_bounds)
    else:
        raise ValueError(f"Unsupported example action: {action}")


def load_collision_checker(scenario_path: str, engine=ENGINE):
    scenario, _ = CommonRoadFileReader(scenario_path).open()
    builder = CollisionCheckerBuilder(engine=engine)
    checker = create_collision_checker_from_scenario(scenario, builder=builder).build()
    return scenario, checker


def run_smoke_checks(scenario, checker, scenario_path: str, pose_bounds):
    car = Rectangle(*CAR_SIZE)
    print(f"Loaded scenario: {Path(scenario_path).name}")
    print(f"Lanelets: {len(scenario.lanelet_network.lanelets)}")
    print(f"Static obstacles: {len(scenario.static_obstacles)}")
    print(f"Dynamic obstacles: {len(scenario.dynamic_obstacles)}")
    print(f"Pose sample bounds: {format_pose_bounds(pose_bounds)}")

    if Path(scenario_path).name == MERGE_SCENARIO_NAME:
        print(
            "Merge smoke check, road boundary:",
            checker.collides_static(car, Pose((55.29, -1.99), 1.326)),
        )
        print(
            "Merge smoke check, any-time dynamic collision:",
            checker.collides_static(car, Pose((37.33, 4.07), -2.207)),
        )
        return

    poses = sample_poses(DIAGNOSTIC_SAMPLE_COUNT, pose_bounds)
    results = [checker.collides_static(car, pose) for pose in poses]
    print(
        f"Scenario diagnostic: {count_collisions(results)} / {len(results)} sampled poses collide at some scenario time"
    )


def run_geometry_examples():
    # Keep a few small shape-to-shape examples as a quick sanity check.
    rectangle = Rectangle(2, 3)
    circle = Circle(1)
    print("Should collide", rectangle.collides(circle, pos_other=Pose((1.5, 0), 0)))

    outer_polygon = Polygon(
        exterior=[(0, 0), (4, 0), (4, 4), (0, 4)],
        interiors=[[(1, 1), (2, 1), (2, 2), (1, 2)]],
    )
    inner_polygon = Polygon(
        exterior=[(1.25, 1.25), (1.75, 1.25), (1.75, 1.75), (1.25, 1.75)],
        interiors=[],
    )
    overlapping_polygon = Polygon(
        exterior=[(0.5, 0.5), (3.5, 0.5), (3.5, 3.5), (0.5, 3.5)],
        interiors=[],
    )
    print("Should not collide", outer_polygon.collides(inner_polygon))
    print("Should collide", outer_polygon.collides(overlapping_polygon))


def run_parallel_benchmark(checker, pose_bounds):
    car = Rectangle(*CAR_SIZE)
    poses = sample_poses(BENCHMARK_SAMPLE_COUNT, pose_bounds)
    positioned_cars = [(car, pose) for pose in poses]

    parallel_results, parallel_elapsed = timed(
        lambda: checker.par_collides_static(positioned_cars),
    )
    print(f"Parallel any-time checks: {parallel_elapsed:.4f} seconds, {count_collisions(parallel_results)} collisions")

    sequential_results, sequential_elapsed = timed(
        lambda: [checker.collides_static(car, pose) for car, pose in positioned_cars],
    )
    print(
        f"Sequential any-time checks: {sequential_elapsed:.4f} seconds, "
        f"{count_collisions(sequential_results)} collisions"
    )


def run_visualization(scenario, checker, scenario_path, pose_bounds):
    poses = visualization_poses(scenario_path, pose_bounds)
    return visualize_animated_results(scenario, checker, poses, pose_bounds)


def scenario_pose_bounds(scenario):
    vertices = np.concatenate(
        [lanelet.polygon.vertices for lanelet in scenario.lanelet_network.lanelets],
    )
    min_xy = vertices.min(axis=0) - POSE_BOUNDS_PADDING
    max_xy = vertices.max(axis=0) + POSE_BOUNDS_PADDING
    lower_bounds = [min_xy[0], min_xy[1], -np.pi]
    upper_bounds = [max_xy[0], max_xy[1], np.pi]
    return lower_bounds, upper_bounds


def format_pose_bounds(pose_bounds):
    lower_bounds, upper_bounds = pose_bounds
    return (
        f"x=[{lower_bounds[0]:.2f}, {upper_bounds[0]:.2f}], "
        f"y=[{lower_bounds[1]:.2f}, {upper_bounds[1]:.2f}], "
        f"theta=[{-np.pi:.2f}, {np.pi:.2f}]"
    )


def sample_poses(count, pose_bounds, rng=np.random):
    lower_bounds, upper_bounds = pose_bounds
    return [Pose((x, y), rotation) for x, y, rotation in rng.uniform(lower_bounds, upper_bounds, (count, 3))]


def visualization_poses(scenario_path, pose_bounds, count=VISUALIZATION_SAMPLE_COUNT, seed=VISUALIZATION_RANDOM_SEED):
    preset_poses = [
        Pose(translation, rotation)
        for translation, rotation in VISUALIZATION_PRESET_POSES.get(Path(scenario_path).name, [])
    ]
    if count <= len(preset_poses):
        return preset_poses[:count]

    rng = np.random.default_rng(seed)
    random_poses = sample_poses(count - len(preset_poses), pose_bounds, rng)
    return preset_poses + random_poses


def timed(operation):
    start = time.perf_counter()
    result = operation()
    return result, time.perf_counter() - start


def count_collisions(results):
    return sum(result.collides for result in results)


def scenario_time_steps(scenario):
    time_steps = []
    for dynamic_obstacle in scenario.dynamic_obstacles:
        time_steps.append(dynamic_obstacle.initial_state.time_step)
        if isinstance(dynamic_obstacle.prediction, TrajectoryPrediction):
            time_steps.extend(state.time_step for state in dynamic_obstacle.prediction.trajectory.state_list)
    return sorted(set(time_steps)) or [0]


def update_collision_flags(collided_flags, collision_results):
    for index, collision_status in enumerate(collision_results):
        collided_flags[index] = collided_flags[index] or collision_status.collides
    return collided_flags


def visualize_animated_results(scenario, checker, poses, pose_bounds):
    samples = poses[:VISUALIZATION_SAMPLE_COUNT]
    car = Rectangle(*CAR_SIZE)
    positioned_cars = [(car, pose) for pose in samples]
    collided_flags = [False] * len(samples)
    time_steps = scenario_time_steps(scenario)

    fig, ax = plt.subplots()
    plot_limits = pose_bounds_to_plot_limits(pose_bounds)
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
    fig._crcc_animation = animation
    plt.show()
    return animation


def pose_bounds_to_plot_limits(pose_bounds):
    lower_bounds, upper_bounds = pose_bounds
    return [lower_bounds[0], upper_bounds[0], lower_bounds[1], upper_bounds[1]]


def car_occupancy_for_pose(pose):
    return CAR_OBSTACLE_SHAPE.compute_occupancy_for_state(
        InitialState(position=np.array(pose.translation), orientation=pose.rotation)
    )


if __name__ == "__main__":
    main()
