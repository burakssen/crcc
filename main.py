import random
import time

import commonroad.geometry.shape as cr_shape
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_params import ShapeParams
from commonroad.visualization.mp_renderer import MPRenderer
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Circle, Polygon, Rectangle
from crcc.commonroad import create_collision_checker_from_scenario
from crcc.pose import Pose
from matplotlib import pyplot as plt

SCENARIO_PATH = "scenarios/ZAM_Merge-1_1_T-1.xml"
ENGINE = CollisionEngine.Parry
CAR_SIZE = (4.5, 2.0)
POSE_BOUNDS = ([-7.0, -15.0, -np.pi], [87.0, 10.0, np.pi])
BENCHMARK_SAMPLE_COUNT = 100_000
VISUALIZATION_SAMPLE_COUNT = 200


def main():
    scenario, checker = load_collision_checker(SCENARIO_PATH)
    run_smoke_checks(checker)
    run_geometry_examples()
    demo_parallel(scenario, checker)


def load_collision_checker(scenario_path: str):
    scenario, _ = CommonRoadFileReader(scenario_path).open()
    builder = CollisionCheckerBuilder(engine=ENGINE)
    checker = create_collision_checker_from_scenario(scenario, builder=builder).build()
    return scenario, checker


def run_smoke_checks(checker):
    car = Rectangle(*CAR_SIZE)
    print("Collides with road boundary", checker.collides_static(car, Pose((55.29, -1.99), 1.326)))
    print("Collides between step 26 and 27", checker.collides_static(car, Pose((37.33, 4.07), -2.207)))


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


def demo_parallel(scenario, checker):
    car = Rectangle(*CAR_SIZE)
    poses = sample_poses(BENCHMARK_SAMPLE_COUNT)
    positioned_cars = [(car, pose) for pose in poses]

    parallel_results, parallel_elapsed = timed(
        lambda: checker.par_collides_static(positioned_cars),
    )
    print(
        f"Parallel: {parallel_elapsed:.4f} seconds, "
        f"{count_collisions(parallel_results)} collisions"
    )

    sequential_results, sequential_elapsed = timed(
        lambda: [checker.collides_static(car, pose) for car, pose in positioned_cars],
    )
    print(
        f"Sequential: {sequential_elapsed:.4f} seconds, "
        f"{count_collisions(sequential_results)} collisions"
    )

    visualize_sampled_results(scenario, poses, sequential_results)


def sample_poses(count):
    lower_bounds, upper_bounds = POSE_BOUNDS
    return [Pose((x, y), rotation) for x, y, rotation in np.random.uniform(lower_bounds, upper_bounds, (count, 3))]


def timed(operation):
    start = time.perf_counter()
    result = operation()
    return result, time.perf_counter() - start


def count_collisions(results):
    return sum(result.collides for result in results)


def visualize_sampled_results(scenario, poses, results):
    samples = random.sample(list(zip(poses, results, strict=True)), min(len(poses), VISUALIZATION_SAMPLE_COUNT))
    renderer = MPRenderer()
    scenario.draw(renderer)
    for pose, collision_status in samples:
        rectangle = cr_shape.Rectangle(*CAR_SIZE, np.array(pose.translation), pose.rotation)
        params = ShapeParams(facecolor="red" if collision_status.collides else "green", opacity=0.5)
        rectangle.draw(renderer, params)
    renderer.render()
    plt.show()


if __name__ == "__main__":
    main()
