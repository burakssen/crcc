from pathlib import Path

from crcc.collision_object import Rectangle
from crcc.pose import Pose

from examples.utils import (
    CAR_SIZE,
    MERGE_SCENARIO_NAME,
    count_collisions,
    format_pose_bounds,
    sample_poses,
)

DIAGNOSTIC_SAMPLE_COUNT = 1_000


def run(scenario, checker, scenario_path, pose_bounds):
    """Run diagnostic smoke checks on a loaded scenario and checker."""
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
