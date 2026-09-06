"""Backend-neutral, deterministic benchmark workload contract.

This module deliberately has no CRCC binding imports. External harnesses load
it directly so both benchmark implementations serialize identical inputs.
"""

import hashlib
import json
import math
from typing import Any, Iterable

import numpy as np

SCHEMA_VERSION = "12"
CONTRACT_VERSION = "crcc-workload-contract-v1"
SUITES = ("pair", "continuous", "distance", "scene_scaling", "planning")

_PROFILE_DIMENSIONS = {
    "smoke": {
        "scene_sizes": (100, 1_000),
        "polygon_vertices": (16, 64),
        "compound_children": (1, 4),
        "api_batch_sizes": (0, 1, 8, 31, 32, 33, 128, 1_024),
        "dynamic_batch_steps": (1, 4, 16),
        "dynamic_batch_sizes": (1, 8, 31, 32, 33, 128),
        "time_variant_steps": (1, 2, 4, 16),
        "planning_static_counts": (100, 1_000),
        "planning_dynamic_counts": (4, 16),
        "planning_candidates": (16, 64),
        "planning_steps": (8, 16),
        "native_layer_iterations": (10_000,),
        "reusable_iterations": (3,),
    },
    "spec": {
        "scene_sizes": (100, 1_000, 5_000, 10_000, 25_000, 50_000),
        "polygon_vertices": (16, 64, 256, 1_024),
        "compound_children": (1, 4, 16, 64, 256),
        "api_batch_sizes": (0, 1, 8, 31, 32, 33, 128, 1_024, 10_000, 65_536),
        "dynamic_batch_steps": (1, 2, 4, 16, 64, 256),
        "dynamic_batch_sizes": (1, 8, 31, 32, 33, 128, 1_024),
        "time_variant_steps": (1, 2, 4, 16, 64, 256),
        "planning_static_counts": (100, 1_000, 5_000),
        "planning_dynamic_counts": (4, 16, 64),
        "planning_candidates": (16, 64, 256),
        "planning_steps": (8, 16, 32),
        "native_layer_iterations": (100_000,),
        "reusable_iterations": (10,),
    },
}


def stable_hash(value: str) -> int:
    result = 14_695_981_039_346_656_037
    for byte in value.encode("utf-8"):
        result = (result ^ byte) * 1_099_511_628_211
    return result % 1_000_000


def profile_dimensions(profile: str) -> dict[str, tuple[int, ...]]:
    try:
        return {key: tuple(values) for key, values in _PROFILE_DIMENSIONS[profile].items()}
    except KeyError as error:
        raise ValueError(f"unknown benchmark profile: {profile}") from error


def synthetic_workloads(sample_count: int, seed: int) -> dict[str, list[dict[str, Any]]]:
    if sample_count < 1:
        raise ValueError("sample_count must be positive")
    shape_pairs = {
        "circle_circle": ("circle", "circle"),
        "circle_rectangle": ("circle", "rectangle"),
        "rectangle_rectangle": ("rectangle", "rectangle"),
        "convex_polygon": ("polygon6", "polygon8"),
        "compound_polygon": ("compound2", "polygon7"),
    }
    workloads: dict[str, list[dict[str, Any]]] = {}
    for name, (left_shape, right_shape) in shape_pairs.items():
        rng = np.random.default_rng(seed + stable_hash(name))
        workloads[name] = [
            {
                "index": index,
                "left": {"shape": left_shape, "pose": [0.0, 0.0, 0.0]},
                "right": {
                    "shape": right_shape,
                    "pose": [
                        float(rng.uniform(0.0, 0.75) if index % 2 == 0 else rng.uniform(4.0, 10.0)),
                        0.0,
                        float(rng.uniform(-math.pi, math.pi)),
                    ],
                },
                "expected": index % 2 == 0,
            }
            for index in range(sample_count)
        ]
    return workloads


def scene_workload(objects: int, queries: int, density: float, shape_family: str = "circle") -> dict[str, Any]:
    if objects < 1 or queries < 0 or not 0.0 <= density <= 1.0:
        raise ValueError("invalid scene workload dimensions")
    static_objects = [
        {"index": index, "shape": shape_family, "pose": [*_grid_position(index, 6.0, objects), 0.0]}
        for index in range(objects)
    ]
    query_items = []
    for index in range(queries):
        target = static_objects[index % objects]
        expected = index / max(1, queries) < density
        x, y, angle = target["pose"]
        if not expected:
            x += 2.8 + 0.4 * (index % 5)
            y += 2.8 + 0.3 * (index % 7)
        query_items.append({"index": index, "shape": shape_family, "pose": [x, y, angle], "expected": expected})
    return {
        "objects": objects,
        "query_count": queries,
        "density": float(density),
        "shape_family": shape_family,
        "static_objects": static_objects,
        "queries": query_items,
    }


def planning_frame_workload(
    static_objects: int, dynamic_objects: int, candidate_count: int, trajectory_steps: int, seed: int
) -> dict[str, Any]:
    if min(static_objects, dynamic_objects, candidate_count, trajectory_steps) < 1:
        raise ValueError("planning dimensions must be positive")
    rng = np.random.default_rng(
        seed + stable_hash(f"planning:{static_objects}:{dynamic_objects}:{candidate_count}:{trajectory_steps}")
    )
    static_map = [
        {"index": index, "shape": "circle", "pose": [*_grid_position(index, 6.0, static_objects), 0.0]}
        for index in range(static_objects)
    ]
    dynamic_width = max(1, min(dynamic_objects, 16))
    predicted = [
        {
            "index": index,
            "shape": "circle",
            "poses": [
                [
                    (index % dynamic_width) * 3.0 + 1.5 + step * 0.08,
                    (index // dynamic_width) * 3.0 + 1.5 + step * 0.03,
                    0.0,
                ]
                for step in range(trajectory_steps)
            ],
        }
        for index in range(dynamic_objects)
    ]
    candidates = []
    for index in range(candidate_count):
        x = float(rng.uniform(-0.15, 0.15))
        y = (index % 8 - 3.5) * 1.6 + float(rng.uniform(-0.1, 0.1))
        candidates.append(
            {
                "index": index,
                "shape": "rectangle",
                "poses": [[x + step * 0.45, y + step * 0.02, 0.0] for step in range(trajectory_steps)],
            }
        )
    return {
        "static_count": static_objects,
        "dynamic_count": dynamic_objects,
        "static_scene_objects": static_objects,
        "dynamic_scene_objects": dynamic_objects,
        "candidate_count": candidate_count,
        "trajectory_steps": trajectory_steps,
        "static_map": static_map,
        "predicted": predicted,
        "candidates": candidates,
        "predicted_obstacles": predicted,
        "candidate_trajectories": candidates,
    }


def canonical_bundle(profile: str, sample_count: int, seed: int, suites: Iterable[str] = SUITES) -> dict[str, Any]:
    dimensions = profile_dimensions(profile)
    selected = tuple(dict.fromkeys(str(suite) for suite in suites))
    unknown = sorted(set(selected) - set(SUITES))
    if unknown:
        raise ValueError(f"unknown canonical workload suite(s): {', '.join(unknown)}")
    workloads: dict[str, Any] = {}
    if {"pair", "continuous", "distance"} & set(selected):
        workloads["synthetic"] = synthetic_workloads(sample_count, seed)
    if "scene_scaling" in selected:
        workloads["scene_scaling"] = [
            scene_workload(objects, min(sample_count, 1_000 if objects <= 10_000 else 200), density, shape)
            for objects in dimensions["scene_sizes"]
            for density in (0.0, 0.5)
            for shape in ("circle", "rectangle", "polygon32", "compound16_polygon32")
        ]
    if "planning" in selected:
        workloads["planning"] = [
            planning_frame_workload(static, dynamic, candidates, steps, seed)
            for static in dimensions["planning_static_counts"]
            for dynamic in dimensions["planning_dynamic_counts"]
            for candidates in dimensions["planning_candidates"]
            for steps in dimensions["planning_steps"]
        ]
    payload = {
        "metadata": {
            "schema_version": SCHEMA_VERSION,
            "contract_version": CONTRACT_VERSION,
            "profile": profile,
            "sample_count": sample_count,
            "seed": seed,
            "suites": selected,
            "dimensions": dimensions,
        },
        "workloads": workloads,
    }
    encoded = json.dumps(payload, ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return {**payload, "bytes": encoded, "sha256": hashlib.sha256(encoded).hexdigest()}


def _grid_position(index: int, spacing: float, count: int) -> tuple[float, float]:
    width = max(1, math.ceil(math.sqrt(count)))
    return float(index % width) * spacing, float(index // width) * spacing
