import time
from typing import Any

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.prediction.prediction import TrajectoryPrediction
from crcc import CollisionCheckerBuilder, CollisionEngine
from crcc.commonroad import create_collision_checker_from_scenario

CAR_SIZE = (4.5, 2.0)
POSE_BOUNDS_PADDING = 5.0
MERGE_SCENARIO_NAME = "DEU_MerzenichRather-2_870_T-149.xml"


def load_collision_checker(scenario_path: str, engine: CollisionEngine):
    """Load a CommonRoad scenario and build a CollisionChecker with the given engine."""
    scenario, _ = CommonRoadFileReader(scenario_path).open()
    builder = CollisionCheckerBuilder(engine=engine)
    checker = create_collision_checker_from_scenario(scenario, builder=builder).build()
    return scenario, checker


def scenario_pose_bounds(scenario):
    """Compute pose sampling bounds based on the scenario's lanelet network vertices."""
    if not scenario.lanelet_network.lanelets:
        raise ValueError("scenario has no lanelets from which to derive pose bounds")
    vertices = np.concatenate([lanelet.polygon.vertices for lanelet in scenario.lanelet_network.lanelets])
    min_xy = vertices.min(axis=0) - POSE_BOUNDS_PADDING
    max_xy = vertices.max(axis=0) + POSE_BOUNDS_PADDING
    return [min_xy[0], min_xy[1], -np.pi], [max_xy[0], max_xy[1], np.pi]


def format_pose_bounds(pose_bounds):
    """Format pose bounds for console output."""
    lower_bounds, upper_bounds = pose_bounds
    return (
        f"x=[{lower_bounds[0]:.2f}, {upper_bounds[0]:.2f}], "
        f"y=[{lower_bounds[1]:.2f}, {upper_bounds[1]:.2f}], "
        f"theta=[{-np.pi:.2f}, {np.pi:.2f}]"
    )


def sample_poses(count, pose_bounds, rng: Any = np.random):
    """Sample random 2D Poses within the given bounds."""
    from crcc import Pose

    lower_bounds, upper_bounds = pose_bounds
    return [Pose((x, y), rotation) for x, y, rotation in rng.uniform(lower_bounds, upper_bounds, (count, 3))]


def count_collisions(results):
    """Count the number of collision results where collides is True."""
    return sum(result.collides for result in results)


def scenario_time_steps(scenario):
    """Gather all unique time steps present in a scenario's dynamic obstacles."""
    time_steps = []
    for obstacle in scenario.dynamic_obstacles:
        time_steps.append(obstacle.initial_state.time_step)
        if isinstance(obstacle.prediction, TrajectoryPrediction):
            time_steps.extend(state.time_step for state in obstacle.prediction.trajectory.state_list)
    return sorted(set(time_steps)) or [0]


def timed(operation):
    """Measure the time taken by an operation."""
    start = time.perf_counter()
    result = operation()
    return result, time.perf_counter() - start
