import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Circle, Compound, Polygon, Rectangle, Triangle
from crcc.commonroad import create_collision_checker_from_scenario
from crcc.pose import Pose

from examples.utils import CAR_SIZE, sample_poses, scenario_pose_bounds


@dataclass(frozen=True)
class PairQuery:
    left: Any
    right: Any
    left_pose: Pose
    right_pose: Pose
    expected: bool | None
    left_end_pose: Pose | None = None
    right_end_pose: Pose | None = None


@dataclass(frozen=True)
class SyntheticWorkload:
    feature: str
    workload: str
    operation: str
    queries: tuple[PairQuery, ...]


@dataclass(frozen=True)
class SceneWorkload:
    objects: int
    density: float
    static_objects: tuple[Any, ...]
    positioned_queries: tuple[tuple[Any, Pose], ...]


@dataclass(frozen=True)
class ScenarioWorkload:
    name: str
    checker_by_backend: dict[str, Any]
    car: Rectangle
    poses: tuple[Pose, ...]
    positioned_queries: tuple[tuple[Rectangle, Pose], ...]


def synthetic_workloads(sample_count: int, seed: int):
    for workload in ("circle_circle", "circle_rectangle", "rectangle_rectangle", "convex_polygon", "compound_polygon"):
        yield SyntheticWorkload("pair", workload, "collides", tuple(primitive_queries(sample_count, workload, seed)))
    yield SyntheticWorkload("pair", "polygon_vertex_scaling", "collides", tuple(polygon_complexity_queries(sample_count)))
    yield SyntheticWorkload("pair", "boundary_robustness", "collides", tuple(robustness_queries()))
    for workload in ("tunneling", "moving_vs_moving", "rotation_wrap", "endpoint_touch"):
        yield SyntheticWorkload("continuous", workload, "continuous", tuple(continuous_queries(sample_count, workload)))
    yield SyntheticWorkload(
        "distance", "circle_rectangle", "distance", tuple(primitive_queries(sample_count, "circle_rectangle", seed))
    )


def scene_workload(objects: int, queries: int, density: float):
    grid_width = math.ceil(math.sqrt(objects))
    static_objects = tuple(
        Circle(0.75, ((index % grid_width) * 4.0, (index // grid_width) * 4.0)) for index in range(objects)
    )
    positioned_queries = []
    for index in range(queries):
        should_collide = index / max(1, queries) < density
        target = index % objects
        x = (target % grid_width) * 4.0
        y = (target // grid_width) * 4.0
        offset = (0.0, 0.0) if should_collide else (1.35 + 0.4 * (index % 5), 1.35 + 0.3 * (index % 7))
        pose = Pose.from_translation((x + offset[0], y + offset[1]))
        positioned_queries.append((Circle(0.5), pose))
    return SceneWorkload(objects, density, static_objects, tuple(positioned_queries))


def scenario_workload(path: Path, engines: tuple[tuple[str, CollisionEngine], ...], sample_count: int, seed: int):
    scenario, _ = CommonRoadFileReader(str(path)).open()
    bounds = scenario_pose_bounds(scenario)
    poses = tuple(sample_poses(sample_count, bounds, rng(seed + stable_hash(path.name))))
    car = Rectangle(*CAR_SIZE)
    checkers = {
        name: create_collision_checker_from_scenario(scenario, CollisionCheckerBuilder(engine=engine)).build()
        for name, engine in engines
    }
    return ScenarioWorkload(path.stem, checkers, car, poses, tuple((car, pose) for pose in poses))


def primitive_queries(sample_count: int, kind: str, seed: int):
    generator = rng(seed + stable_hash(kind))
    queries = []
    for index in range(sample_count):
        colliding = index % 2 == 0
        offset = float(generator.uniform(0.0, 0.75) if colliding else generator.uniform(4.0, 10.0))
        angle = float(generator.uniform(-math.pi, math.pi))
        if kind == "circle_circle":
            left, right = Circle(1.0), Circle(1.0)
        elif kind == "circle_rectangle":
            left, right = Circle(1.0), Rectangle(2.0, 1.0, angle)
        elif kind == "rectangle_rectangle":
            left, right = Rectangle(2.0, 1.0, angle), Rectangle(2.0, 1.0, -angle)
        elif kind == "convex_polygon":
            left, right = regular_polygon(6, 1.0), regular_polygon(8, 1.0)
        elif kind == "compound_polygon":
            left = Compound(
                [
                    Triangle((-1.0, -0.5), (0.5, -0.5), (-0.2, 0.7)),
                    Triangle((0.5, -0.5), (1.0, 0.7), (-0.2, 0.7)),
                ]
            )
            right = regular_polygon(7, 0.9)
        else:
            raise ValueError(f"unknown primitive workload: {kind}")
        queries.append(PairQuery(left, right, Pose.identity(), Pose.from_translation((offset, 0.0)), expected=colliding))
    return queries


def polygon_complexity_queries(sample_count: int):
    vertex_counts = (8, 32, 128)
    per_size = max(1, sample_count // len(vertex_counts))
    return [
        PairQuery(
            regular_polygon(vertices, 1.0),
            regular_polygon(vertices, 1.0),
            Pose.identity(),
            Pose.from_translation((0.5 if index % 2 == 0 else 4.0, 0.0)),
            expected=index % 2 == 0,
        )
        for vertices in vertex_counts
        for index in range(per_size)
    ]


def robustness_queries():
    return [
        PairQuery(Circle(1.0), Circle(1.0), Pose.identity(), Pose.from_translation((2.0, 0.0)), expected=True),
        PairQuery(Circle(1.0), Circle(1.0), Pose.identity(), Pose.from_translation((2.0 + 1e-9, 0.0)), expected=False),
        PairQuery(
            Rectangle(1e-6, 1e-6),
            Rectangle(1e-6, 1e-6, 1e-12),
            Pose.from_translation((1e-6, 1e-6)),
            Pose.from_translation((1.5e-6, 1e-6)),
            expected=True,
        ),
        PairQuery(
            Rectangle(10.0, 0.01, 1e-9),
            Rectangle(10.0, 0.01, -1e-9),
            Pose.from_translation((1e9, 1e9)),
            Pose.from_translation((1e9, 1e9 + 0.02)),
            expected=False,
        ),
    ]


def continuous_queries(sample_count: int, kind: str):
    queries = []
    for index in range(sample_count):
        hit = index % 2 == 0
        if kind == "tunneling":
            queries.append(
                PairQuery(
                    Circle(0.5),
                    Rectangle(0.25, 3.0),
                    Pose.from_translation((-4.0, 0.0 if hit else 3.0)),
                    Pose.identity(),
                    hit,
                    Pose.from_translation((4.0, 0.0 if hit else 3.0)),
                    Pose.identity(),
                )
            )
        elif kind == "moving_vs_moving":
            queries.append(
                PairQuery(
                    Circle(0.5),
                    Circle(0.5),
                    Pose.from_translation((-3.0, 0.0 if hit else 2.0)),
                    Pose.from_translation((3.0, 0.0)),
                    hit,
                    Pose.from_translation((3.0, 0.0 if hit else 2.0)),
                    Pose.from_translation((-3.0, 0.0)),
                )
            )
        elif kind == "rotation_wrap":
            queries.append(
                PairQuery(
                    Rectangle(3.0, 0.4),
                    Circle(0.5),
                    Pose((0.0, 0.0), math.pi - 0.1),
                    Pose.from_translation((0.0, 2.5 if hit else 4.0)),
                    hit,
                    Pose((0.0, 0.0), -math.pi + 0.1),
                    Pose.from_translation((0.0, 2.5 if hit else 4.0)),
                )
            )
        elif kind == "endpoint_touch":
            queries.append(
                PairQuery(
                    Circle(0.5),
                    Circle(0.5),
                    Pose.from_translation((-3.0, 0.0)),
                    Pose.from_translation((1.0 if hit else 1.01, 0.0)),
                    hit,
                    Pose.from_translation((0.0, 0.0)),
                    Pose.from_translation((1.0 if hit else 1.01, 0.0)),
                )
            )
        else:
            raise ValueError(f"unknown continuous workload: {kind}")
    return queries


def regular_polygon(vertices: int, radius: float):
    points = [
        (radius * math.cos(index / vertices * math.tau), radius * math.sin(index / vertices * math.tau))
        for index in range(vertices)
    ]
    points.append(points[0])
    return Polygon(points)


def rng(seed: int):
    return np.random.default_rng(seed)


def stable_hash(value: str):
    hash_value = 14_695_981_039_346_656_037
    for byte in value.encode():
        hash_value = (hash_value ^ byte) * 1_099_511_628_211
    return hash_value % 1_000_000
