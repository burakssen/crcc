from pathlib import Path

from crcc.collision_object import Rectangle
from crcc.pose import Pose

from examples.utils import CAR_SIZE, count_collisions, format_pose_bounds, sample_poses, scenario_time_steps

AUDIT_SAMPLE_COUNT = 1_000
PROBE_POSES = {
    "DEU_MerzenichRather-2_870_T-149.xml": (
        ("road_boundary", Pose((55.29, -1.99), 1.326)),
        ("dynamic_conflict", Pose((37.33, 4.07), -2.207)),
    )
}


def scenario_audit(scenario, checker, scenario_path: str, pose_bounds, sample_count: int = AUDIT_SAMPLE_COUNT):
    car = Rectangle(*CAR_SIZE)
    time_steps = scenario_time_steps(scenario)
    poses = sample_poses(sample_count, pose_bounds)
    statuses = [checker.collides_static(car, pose) for pose in poses]
    probes = {
        name: str(checker.collides_static(car, pose))
        for name, pose in PROBE_POSES.get(Path(scenario_path).name, ())
    }
    return {
        "scenario": Path(scenario_path).name,
        "lanelets": len(scenario.lanelet_network.lanelets),
        "static_obstacles": len(scenario.static_obstacles),
        "dynamic_obstacles": len(scenario.dynamic_obstacles),
        "first_time_step": min(time_steps),
        "last_time_step": max(time_steps),
        "sample_count": len(poses),
        "sample_collisions": count_collisions(statuses),
        "pose_bounds": format_pose_bounds(pose_bounds),
        "probes": probes,
    }


def run(scenario, checker, scenario_path, pose_bounds):
    """Print a reproducible CommonRoad scenario collision audit."""
    audit = scenario_audit(scenario, checker, scenario_path, pose_bounds)
    print(f"Scenario audit: {audit['scenario']}")
    for key in (
        "lanelets",
        "static_obstacles",
        "dynamic_obstacles",
        "first_time_step",
        "last_time_step",
        "sample_count",
        "sample_collisions",
        "pose_bounds",
    ):
        print(f"  {key}: {audit[key]}")
    for name, status in audit["probes"].items():
        print(f"  probe {name}: {status}")
    return audit
