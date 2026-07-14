from pathlib import Path

import numpy as np
from crcc import Pose, Rectangle

from examples.presentation import ResultRow, print_results
from examples.utils import CAR_SIZE, scenario_time_steps


def deterministic_probes(scenario, pose_bounds) -> tuple[tuple[str, Pose], ...]:
    lanelets = scenario.lanelet_network.lanelets
    if not lanelets:
        return ()
    vertices = np.asarray(lanelets[0].polygon.vertices)
    inside = tuple(vertices[:, :2].mean(axis=0))
    outside = (pose_bounds[1][0] + 2 * CAR_SIZE[0], pose_bounds[1][1] + 2 * CAR_SIZE[0])
    return (("first lanelet centroid", Pose(inside, 0.0)), ("outside road bounds", Pose(outside, 0.0)))


def commonroad_results(scenario, checker, pose_bounds) -> tuple[ResultRow, ...]:
    car = Rectangle(*CAR_SIZE)
    first_time = scenario_time_steps(scenario)[0]
    results = []
    for name, pose in deterministic_probes(scenario, pose_bounds):
        status = checker.collides_static(car, pose, min_time=first_time, max_time=first_time)
        results.append((name, "hit" if status.collides else "clear", str(status)))
    return tuple(results)


def run(scenario, checker, scenario_path, pose_bounds):
    results = commonroad_results(scenario, checker, pose_bounds)
    print(f"CommonRoad scenario: {Path(scenario_path).name}")
    print(
        f"  lanelets={len(scenario.lanelet_network.lanelets)} static={len(scenario.static_obstacles)} dynamic={len(scenario.dynamic_obstacles)}"
    )
    print_results("Deterministic vehicle probes", results)
    return results
