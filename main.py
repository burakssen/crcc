import random
import time
from pathlib import Path

import commonroad.geometry.shape as cr_shape
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.visualization.draw_params import MPDrawParams, ShapeParams
from commonroad.visualization.mp_renderer import MPRenderer
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Circle, Polygon, Rectangle
from crcc.commonroad import create_collision_checker_from_scenario
from crcc.pose import Pose
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

SCENARIO_PATH = "scenarios/DEU_MerzenichRather-2_870_T-149.xml"
ENGINE = CollisionEngine.Rhusics
CAR_SIZE = (4.5, 2.0)
POSE_BOUNDS_PADDING = 5.0
BENCHMARK_SAMPLE_COUNT = 100_000
VISUALIZATION_SAMPLE_COUNT = 5000
DIAGNOSTIC_SAMPLE_COUNT = 1_000
MERGE_SCENARIO_NAME = "DEU_MerzenichRather-2_870_T-149.xml"
ANIMATION_INTERVAL_MS = 100
COLOR_COLLIDED = "red"
COLOR_CLEAR = "green"


def main():
    scenario, checker = load_collision_checker(SCENARIO_PATH)
    pose_bounds = scenario_pose_bounds(scenario)
    run_smoke_checks(scenario, checker, SCENARIO_PATH, pose_bounds)
    run_geometry_examples()
    demo_parallel(scenario, checker, pose_bounds)


def load_collision_checker(scenario_path: str):
    scenario, _ = CommonRoadFileReader(scenario_path).open()
    builder = CollisionCheckerBuilder(engine=ENGINE)
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


def demo_parallel(scenario, checker, pose_bounds):
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

    visualize_animated_results(scenario, checker, poses, pose_bounds)


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


def sample_poses(count, pose_bounds):
    lower_bounds, upper_bounds = pose_bounds
    return [Pose((x, y), rotation) for x, y, rotation in np.random.uniform(lower_bounds, upper_bounds, (count, 3))]


def timed(operation):
    start = time.perf_counter()
    result = operation()
    return result, time.perf_counter() - start


def count_collisions(results):
    return sum(result.collides for result in results)


def scenario_time_steps(scenario):
    time_steps = []
    for dynamic_obstacle in scenario.dynamic_obstacles:
        initial_time = dynamic_obstacle.initial_state.time_step
        if isinstance(dynamic_obstacle.prediction, TrajectoryPrediction):
            state_count = len(dynamic_obstacle.prediction.trajectory.state_list)
            time_steps.extend(range(initial_time, initial_time + state_count + 1))
        else:
            time_steps.append(initial_time)
    return sorted(set(time_steps)) or [0]


def update_collision_flags(collided_flags, collision_results):
    for index, collision_status in enumerate(collision_results):
        collided_flags[index] = collided_flags[index] or collision_status.collides
    return collided_flags


def visualize_animated_results(scenario, checker, poses, pose_bounds):
    samples = random.sample(poses, min(len(poses), VISUALIZATION_SAMPLE_COUNT))
    car = Rectangle(*CAR_SIZE)
    positioned_cars = [(car, pose) for pose in samples]
    collided_flags = [False] * len(samples)
    time_steps = scenario_time_steps(scenario)

    fig, ax = plt.subplots()
    plot_limits = pose_bounds_to_plot_limits(pose_bounds)

    def draw_frame(time_step):
        ax.clear()
        draw_params = MPDrawParams(time_begin=time_step, time_end=time_step)
        renderer = MPRenderer(draw_params=draw_params, plot_limits=plot_limits, ax=ax)
        scenario.draw(renderer, draw_params)
        for pose, collided in zip(samples, collided_flags, strict=True):
            rectangle = cr_shape.Rectangle(*CAR_SIZE, np.array(pose.translation), pose.rotation)
            color = COLOR_COLLIDED if collided else COLOR_CLEAR
            rectangle.draw(renderer, ShapeParams(facecolor=color, edgecolor=color, opacity=0.5))
        renderer.render()
        ax.set_title(f"Collision state at time step {time_step}")
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


def pose_bounds_to_plot_limits(pose_bounds):
    lower_bounds, upper_bounds = pose_bounds
    return [lower_bounds[0], upper_bounds[0], lower_bounds[1], upper_bounds[1]]


if __name__ == "__main__":
    main()
