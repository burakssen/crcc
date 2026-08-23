import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from crcc import (
    Circle,
    CollisionCheckerBuilder,
    CollisionEngine,
    Compound,
    DynamicObstacle,
    Polygon,
    Pose,
    Rectangle,
    Triangle,
)
from crcc.commonroad import scenario_builder

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
    expected_by_backend: dict[str, bool] | None = None


@dataclass(frozen=True)
class SyntheticWorkload:
    feature: str
    workload: str
    operation: str
    queries: tuple[PairQuery, ...]
    shape_family: str = ""
    scene_mode: str = "pair"
    ccd_mode: str = "discrete"


@dataclass(frozen=True)
class SceneWorkload:
    objects: int
    density: float
    static_objects: tuple[Any, ...]
    positioned_queries: tuple[tuple[Any, Pose], ...]
    density_label: str = ""
    shape_family: str = "circle"
    scene_mode: str = "static_static"
    ccd_mode: str = "discrete"


@dataclass(frozen=True)
class ScenarioWorkload:
    name: str
    checker_by_backend: dict[str, Any]
    car: Rectangle
    poses: tuple[Pose, ...]
    positioned_queries: tuple[tuple[Rectangle, Pose], ...]


@dataclass(frozen=True)
class PlanningWorkload:
    static_objects: tuple[Any, ...]
    predicted_obstacles: tuple[DynamicObstacle, ...]
    candidate_trajectories: tuple[DynamicObstacle, ...]
    static_count: int
    dynamic_count: int
    candidate_count: int
    trajectory_steps: int
    shape_family: str = "circle"


def synthetic_workloads(sample_count: int, seed: int):
    for workload in ("circle_circle", "circle_rectangle", "rectangle_rectangle", "convex_polygon", "compound_polygon"):
        yield SyntheticWorkload("pair", workload, "collides", tuple(primitive_queries(sample_count, workload, seed)))
    yield SyntheticWorkload(
        "pair", "polygon_vertex_scaling", "collides", tuple(polygon_complexity_queries(sample_count))
    )
    yield SyntheticWorkload("pair", "boundary_robustness", "collides", tuple(robustness_queries()))
    for workload in ("tunneling", "moving_vs_moving", "rotation_wrap", "endpoint_touch"):
        yield SyntheticWorkload("continuous", workload, "continuous", tuple(continuous_queries(sample_count, workload)))
    yield SyntheticWorkload(
        "distance", "circle_rectangle", "distance", tuple(primitive_queries(sample_count, "circle_rectangle", seed))
    )


def spec_shape_workloads(sample_count: int, vertex_counts: tuple[int, ...], child_counts: tuple[int, ...]):
    for vertices in vertex_counts:
        yield SyntheticWorkload(
            "shape_complexity",
            f"convex_polygon_{vertices}",
            "collides",
            tuple(_same_shape_queries(regular_polygon(vertices, 1.0), sample_count)),
        )
    for children in child_counts:
        yield SyntheticWorkload(
            "shape_complexity",
            f"compound_{children}",
            "collides",
            tuple(_same_shape_queries(compound_grid(children), sample_count)),
        )


def coverage_matrix_workloads(sample_count: int):
    for family in ("circle", "rectangle", "polygon32", "compound16_polygon32"):
        left, right = matrix_pair(family)
        for ccd_mode in ("discrete", "stationary", "moving_static", "moving_moving"):
            operation = "collides" if ccd_mode == "discrete" else "continuous"
            yield SyntheticWorkload(
                "coverage_matrix",
                f"{family}_{ccd_mode}",
                operation,
                tuple(matrix_queries(left, right, sample_count, ccd_mode)),
                family,
                "pair",
                ccd_mode,
            )


def update_proxy_workload(objects: int, transform_kind: str, seed: int):
    generator = rng(seed + stable_hash(f"update:{objects}:{transform_kind}"))
    query = Circle(0.5)
    positioned_queries = []
    for index in range(objects):
        if transform_kind == "translation":
            pose = Pose.from_translation((float(index) * 1.5, 0.0))
        elif transform_kind == "rotation":
            pose = Pose((0.0, 0.0), index * 0.01)
        elif transform_kind == "translation_rotation":
            pose = Pose((float(index) * 1.5, 0.25), index * 0.01)
        elif transform_kind == "randomized":
            pose = Pose(
                (float(generator.uniform(-100.0, 100.0)), float(generator.uniform(-100.0, 100.0))),
                float(generator.uniform(-math.pi, math.pi)),
            )
        else:
            raise ValueError(f"unknown transform kind: {transform_kind}")
        positioned_queries.append((query, pose))
    return SceneWorkload(objects, 0.0, (Circle(0.75),), tuple(positioned_queries))


def rebuild_update_workload(objects: int, transform_kind: str, seed: int, shape_family: str = "rectangle"):
    generator = rng(seed + stable_hash(f"rebuild:{objects}:{transform_kind}"))
    updated = []
    width = max(1, math.ceil(math.sqrt(objects)))
    for index in range(objects):
        x = float(index % width) * 4.0
        y = float(index // width) * 4.0
        angle = 0.0
        if transform_kind in {"translation", "translation_rotation"}:
            x += 0.25
            y += 0.1
        if transform_kind in {"rotation", "translation_rotation"}:
            angle = 0.1 + index * 0.001
        if transform_kind == "randomized":
            x += float(generator.uniform(-0.5, 0.5))
            y += float(generator.uniform(-0.5, 0.5))
            angle = float(generator.uniform(-math.pi, math.pi))
        updated.append(matrix_shape(shape_family, (x, y), angle))
    return tuple(updated)


def api_batch_workload(batch_size: int):
    return tuple(
        (Circle(0.5), Pose.from_translation((0.0 if index % 2 == 0 else 4.0, 0.0))) for index in range(batch_size)
    )


def density_scene_workload(objects: int, queries: int, density_label: str):
    density_by_label = {"clear": 0.0, "medium": 0.10, "dense": 0.50, "worst_case": 1.0}
    density = density_by_label[density_label]
    if density_label != "worst_case":
        workload = scene_workload(objects, queries, density)
        return SceneWorkload(
            workload.objects,
            workload.density,
            workload.static_objects,
            workload.positioned_queries,
            density_label,
        )
    static_objects = tuple(Circle(0.75) for _ in range(objects))
    positioned_queries = tuple((Circle(0.5), Pose.identity()) for _ in range(queries))
    return SceneWorkload(objects, density, static_objects, positioned_queries, density_label)


def dynamic_scene_workload(
    static_count: int,
    dynamic_count: int,
    steps: int,
    *,
    x_offset: float = 0.0,
    shape_family: str = "circle",
):
    static_objects = tuple(matrix_shape(shape_family, (float(index) * 4.0, 0.0)) for index in range(static_count))
    query_shape = matrix_pair(shape_family)[1]
    dynamics = []
    for index in range(dynamic_count):
        poses = [Pose.from_translation((float(index) * 4.0 + x_offset, float(step) * 0.25)) for step in range(steps)]
        dynamics.append(DynamicObstacle(query_shape, poses, 0))
    return static_objects, tuple(dynamics)


def dynamic_query_batch(count: int, steps: int):
    return tuple(
        DynamicObstacle(
            Circle(0.5),
            [Pose.from_translation((0.25 if index % 2 == 0 else 10.0, step * 0.05)) for step in range(steps)],
            0,
        )
        for index in range(count)
    )


def planning_frame_workload(
    static_count: int,
    dynamic_count: int,
    candidate_count: int,
    trajectory_steps: int,
    seed: int,
    *,
    shape_family: str = "circle",
):
    """Build one deterministic planning frame with a prepared map and predictions."""
    generator = rng(seed + stable_hash(f"planning:{static_count}:{dynamic_count}:{candidate_count}:{trajectory_steps}"))
    width = max(1, math.ceil(math.sqrt(static_count)))
    static_objects = tuple(
        matrix_shape(shape_family, ((index % width) * 6.0, (index // width) * 6.0)) for index in range(static_count)
    )
    predicted_obstacles = []
    for index in range(dynamic_count):
        x = float(index % max(1, min(dynamic_count, 16))) * 3.0 + 1.5
        y = float(index // max(1, min(dynamic_count, 16))) * 3.0 + 1.5
        poses = [Pose.from_translation((x + step * 0.08, y + step * 0.03)) for step in range(trajectory_steps)]
        predicted_obstacles.append(DynamicObstacle(matrix_pair(shape_family)[1], poses, 0))

    candidate_trajectories = []
    for index in range(candidate_count):
        lane = index % 8
        x_offset = float(generator.uniform(-0.15, 0.15))
        y = (lane - 3.5) * 1.6 + float(generator.uniform(-0.1, 0.1))
        poses = [Pose.from_translation((x_offset + step * 0.45, y + step * 0.02)) for step in range(trajectory_steps)]
        candidate_trajectories.append(DynamicObstacle(matrix_pair(shape_family)[1], poses, 0))

    return PlanningWorkload(
        tuple(static_objects),
        tuple(predicted_obstacles),
        tuple(candidate_trajectories),
        static_count,
        dynamic_count,
        candidate_count,
        trajectory_steps,
        shape_family,
    )


def time_variant_query_batch(count: int, steps: int, variation: str):
    obstacles = []
    for index in range(count):
        if variation == "repeated_shape":
            shapes = [Circle(0.5) for _ in range(steps)]
        elif variation == "circle_radius":
            shapes = [Circle(0.25 + 0.5 * step / max(1, steps - 1)) for step in range(steps)]
        elif variation == "primitive_switch":
            shapes = [Circle(0.5) if step % 2 == 0 else Rectangle(1.0, 1.0) for step in range(steps)]
        else:
            raise ValueError(f"unknown shape variation: {variation}")
        x = 0.25 if index % 2 == 0 else 10.0
        positions = [Pose.from_translation((x, step * 0.05)) for step in range(steps)]
        obstacles.append(DynamicObstacle.from_time_variant(shapes, 0, positions))
    return tuple(obstacles)


def scene_workload(objects: int, queries: int, density: float, shape_family: str = "circle"):
    grid_width = math.ceil(math.sqrt(objects))
    static_objects = tuple(
        matrix_shape(shape_family, ((index % grid_width) * 6.0, (index // grid_width) * 6.0))
        for index in range(objects)
    )
    query_shape = matrix_pair(shape_family)[1]
    positioned_queries = []
    for index in range(queries):
        should_collide = index / max(1, queries) < density
        target = index % objects
        x = (target % grid_width) * 6.0
        y = (target // grid_width) * 6.0
        offset = (0.0, 0.0) if should_collide else (2.8 + 0.4 * (index % 5), 2.8 + 0.3 * (index % 7))
        pose = Pose.from_translation((x + offset[0], y + offset[1]))
        positioned_queries.append((query_shape, pose))
    return SceneWorkload(
        objects,
        density,
        static_objects,
        tuple(positioned_queries),
        shape_family=shape_family,
    )


def matrix_pair(family: str):
    if family == "circle":
        return Circle(1.0), Circle(1.0)
    if family == "rectangle":
        return Rectangle(2.0, 1.0), Rectangle(2.0, 1.0)
    if family == "polygon32":
        return regular_polygon(32, 1.0), regular_polygon(32, 1.0)
    if family == "compound16_polygon32":
        return compound_grid(16), regular_polygon(32, 1.0)
    raise ValueError(f"unknown matrix shape family: {family}")


def matrix_shape(family: str, center=(0.0, 0.0), angle: float = 0.0):
    if family == "circle":
        return Circle(0.75, center)
    if family == "rectangle":
        return Rectangle(1.5, 0.8, angle, center)
    if family == "polygon32":
        cosine, sine = math.cos(angle), math.sin(angle)
        points = [
            (
                center[0] + cosine * math.cos(index / 32 * math.tau) - sine * math.sin(index / 32 * math.tau),
                center[1] + sine * math.cos(index / 32 * math.tau) + cosine * math.sin(index / 32 * math.tau),
            )
            for index in range(32)
        ]
        points.append(points[0])
        return Polygon(points)
    if family == "compound16_polygon32":
        cosine, sine = math.cos(angle), math.sin(angle)
        children = []
        for index in range(16):
            local_x = (index % 4 - 1.5) * 0.4
            local_y = (index // 4 - 1.5) * 0.4
            children.append(
                Circle(
                    0.18,
                    (
                        center[0] + cosine * local_x - sine * local_y,
                        center[1] + sine * local_x + cosine * local_y,
                    ),
                )
            )
        return Compound(children)
    raise ValueError(f"unknown matrix shape family: {family}")


def matrix_queries(left, right, sample_count: int, ccd_mode: str):
    for index in range(sample_count):
        hit = index % 2 == 0
        clear_y = 0.0 if hit else 4.0
        if ccd_mode == "discrete":
            yield PairQuery(left, right, Pose.identity(), Pose.from_translation((0.25 if hit else 4.0, 0.0)), hit)
        elif ccd_mode == "stationary":
            right_pose = Pose.from_translation((0.25 if hit else 4.0, 0.0))
            yield PairQuery(left, right, Pose.identity(), right_pose, hit, Pose.identity(), right_pose)
        elif ccd_mode == "moving_static":
            yield PairQuery(
                left,
                right,
                Pose.from_translation((-4.0, clear_y)),
                Pose.identity(),
                hit,
                Pose.from_translation((4.0, clear_y)),
                Pose.identity(),
            )
        elif ccd_mode == "moving_moving":
            yield PairQuery(
                left,
                right,
                Pose.from_translation((-4.0, clear_y)),
                Pose.from_translation((4.0, 0.0)),
                hit,
                Pose.from_translation((4.0, clear_y)),
                Pose.from_translation((-4.0, 0.0)),
            )
        else:
            raise ValueError(f"unknown CCD mode: {ccd_mode}")


def _same_shape_queries(shape, sample_count: int):
    for index in range(sample_count):
        hit = index % 2 == 0
        yield PairQuery(shape, shape, Pose.identity(), Pose.from_translation((0.25 if hit else 4.0, 0.0)), expected=hit)


def compound_grid(children: int):
    width = max(1, math.ceil(math.sqrt(children)))
    return Compound([Circle(0.2, ((index % width) * 0.55, (index // width) * 0.55)) for index in range(children)])


def scenario_workload(path: Path, engines: tuple[tuple[str, CollisionEngine], ...], sample_count: int, seed: int):
    scenario, _ = CommonRoadFileReader(str(path)).open()
    bounds = scenario_pose_bounds(scenario)
    poses = tuple(sample_poses(sample_count, bounds, rng(seed + stable_hash(path.name))))
    car = Rectangle(*CAR_SIZE)
    checkers = {
        name: scenario_builder(scenario, CollisionCheckerBuilder(engine=engine)).build() for name, engine in engines
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
        queries.append(
            PairQuery(left, right, Pose.identity(), Pose.from_translation((offset, 0.0)), expected=colliding)
        )
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
        PairQuery(
            Circle(1.0),
            Circle(1.0),
            Pose.identity(),
            Pose.from_translation((2.0, 0.0)),
            expected=True,
            expected_by_backend={"rhusics": False},
        ),
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
                    Pose.from_translation((0.0, 0.0 if hit else 4.0)),
                    hit,
                    Pose((0.0, 0.0), -math.pi + 0.1),
                    Pose.from_translation((0.0, 0.0 if hit else 4.0)),
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
