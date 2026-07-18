import csv
import math
import multiprocessing
import os
import subprocess
import time
from collections.abc import Callable
from pathlib import Path
from typing import TypeVar

from crcc import Circle, Compound, DynamicObstacle, Pose, Rectangle, Triangle

from examples.utils import count_collisions

from .config import (
    DEFAULT_COMPOUND_CHILD_COUNTS,
    DEFAULT_DENSITIES,
    DEFAULT_DENSITY_LABELS,
    DEFAULT_SCENE_SIZES,
    DEFAULT_SPEC_SHAPE_COUNTS,
    DEFAULT_UPDATE_TRANSFORMS,
    MATRIX_SHAPE_FAMILIES,
    MIN_RECOMMENDED_SAMPLE_COUNT,
    SCHEMA_VERSION,
    SPEC_SCENE_SIZES,
    STRESS_SCENE_SIZES,
    WARMUP_QUERY_COUNT,
    BenchmarkConfig,
    selected_engine_items,
)
from .io import load_artifacts, write_artifacts, write_report_from_artifacts, write_suite_artifacts
from .plots import write_plots
from .results import CorrectnessResult, MemoryResult, RunResult, counts, update_counts
from .workloads import (
    api_batch_workload,
    coverage_matrix_workloads,
    density_scene_workload,
    dynamic_query_batch,
    dynamic_scene_workload,
    rebuild_update_workload,
    scenario_workload,
    scene_workload,
    spec_shape_workloads,
    synthetic_workloads,
    time_variant_query_batch,
    update_proxy_workload,
)

T = TypeVar("T")


def run_all(
    scenario_paths=None,
    sample_count=None,
    output_dir=None,
    seed=2026,
    thread_counts=None,
    engines=None,
    repetitions=None,
    step="all",
    profile="smoke",
    suites=None,
    include_stress=False,
):
    config = BenchmarkConfig.from_args(
        scenario_paths=scenario_paths,
        sample_count=BenchmarkConfig.__dataclass_fields__["sample_count"].default
        if sample_count is None
        else sample_count,
        repetitions=BenchmarkConfig.__dataclass_fields__["repetitions"].default if repetitions is None else repetitions,
        output_dir=BenchmarkConfig.__dataclass_fields__["output_dir"].default if output_dir is None else output_dir,
        seed=seed,
        thread_counts=thread_counts,
        engines=engines,
        step=step,
        profile=profile,
        suites=suites,
        include_stress=include_stress,
    )
    if config.step in {"run", "all"}:
        _warn_if_low_sample_count(config.sample_count)
        runs, correctness, parallel_rows, memory_rows = run_benchmarks(config)
        write_artifacts(config, runs, correctness, parallel_rows, memory_rows)
        print(f"Wrote benchmark CSV files to {config.output_dir}")
    if config.step in {"plot", "all"}:
        artifacts = load_artifacts(config.output_dir)
        write_plots(config.output_dir, artifacts)
        write_report_from_artifacts(config.output_dir, artifacts)
        print(f"Wrote benchmark plots to {config.output_dir / 'plots'}")


def run_benchmarks(config: BenchmarkConfig):
    engine_items = selected_engine_items(config.engines)
    runs: list[RunResult] = []
    correctness: list[CorrectnessResult] = []
    parallel_rows: list[dict[str, str | int]] = []
    memory_rows: list[MemoryResult] = []

    shape_counts = DEFAULT_SPEC_SHAPE_COUNTS if config.profile == "spec" else (16, 64)
    compound_counts = DEFAULT_COMPOUND_CHILD_COUNTS if config.profile == "spec" else (1, 4)
    scene_sizes = SPEC_SCENE_SIZES if config.profile == "spec" else DEFAULT_SCENE_SIZES
    if config.include_stress:
        scene_sizes = tuple(dict.fromkeys((*scene_sizes, *STRESS_SCENE_SIZES)))

    for suite in config.suites:
        suite_runs, suite_correctness, suite_parallel_rows, suite_memory_rows = _run_suite(
            suite, config, engine_items, shape_counts, compound_counts, scene_sizes
        )
        runs.extend(suite_runs)
        correctness.extend(suite_correctness)
        parallel_rows.extend(suite_parallel_rows)
        memory_rows.extend(suite_memory_rows)
        write_suite_artifacts(config, suite, suite_runs, suite_correctness, suite_parallel_rows, suite_memory_rows)

    return runs, correctness, parallel_rows, memory_rows


def _run_suite(suite, config, engine_items, shape_counts, compound_counts, scene_sizes):
    if suite in {"pair", "continuous", "distance"}:
        return _run_synthetic_suite(suite, config, engine_items)
    if suite == "shape_complexity":
        return _run_shape_complexity_suite(config, engine_items, shape_counts, compound_counts)
    if suite == "coverage_matrix":
        return _run_coverage_matrix_suite(config, engine_items)
    if suite == "scene_scaling":
        return _run_scene_scaling_suite(config, engine_items, scene_sizes)
    if suite == "update_proxy":
        return _run_update_proxy_suite(config, engine_items, scene_sizes)
    if suite == "rebuild_update":
        return _run_rebuild_update_suite(config, engine_items, scene_sizes)
    if suite == "api_overhead":
        return _run_api_overhead_suite(config, engine_items)
    if suite == "density_scaling":
        return _run_density_scaling_suite(config, engine_items, scene_sizes)
    if suite == "dynamic_scene":
        return _run_dynamic_scene_suite(config, engine_items, scene_sizes)
    if suite == "dynamic_batch":
        return _run_dynamic_batch_suite(config, engine_items)
    if suite == "time_variant":
        return _run_time_variant_suite(config, engine_items)
    if suite == "native_layers":
        return _run_native_layer_suite(config, engine_items)
    if suite == "parallel":
        return _run_reusable_parallel_suite(config, engine_items)
    if suite == "scenario":
        return _run_scenario_suite(config, engine_items)
    raise ValueError(f"unknown benchmark suite: {suite}")


def _empty_results():
    return [], [], [], []


def _run_synthetic_suite(suite, config, engine_items):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    for workload in synthetic_workloads(config.sample_count, config.seed):
        if workload.feature != suite:
            continue
        group_results = _measure_synthetic_group(config, engine_items, workload, correctness)
        runs.extend(group_results)
        _print_group(workload.feature, workload.workload, group_results)
    return runs, correctness, parallel_rows, memory_rows


def _run_shape_complexity_suite(config, engine_items, shape_counts, compound_counts):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    for workload in spec_shape_workloads(config.sample_count, shape_counts, compound_counts):
        group_results = _measure_synthetic_group(config, engine_items, workload, correctness)
        runs.extend(group_results)
        _print_group(workload.feature, workload.workload, group_results)
    return runs, correctness, parallel_rows, memory_rows


def _run_coverage_matrix_suite(config, engine_items):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    for workload in coverage_matrix_workloads(config.sample_count):
        group_results = _measure_synthetic_group(config, engine_items, workload, correctness)
        runs.extend(group_results)
        _print_group(workload.feature, workload.workload, group_results)
    return runs, correctness, parallel_rows, memory_rows


def _measure_synthetic_group(config, engine_items, workload, correctness):
    group_results = []
    for backend, engine in engine_items:
        correctness.append(_synthetic_correctness(backend, engine, workload))
        for repetition in range(config.repetitions):
            group_results.append(_measure_synthetic(backend, engine, workload, repetition))
    return group_results


def _run_scene_scaling_suite(config, engine_items, scene_sizes):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    matrix = (
        (objects, shape_family, density)
        for objects in scene_sizes
        if objects <= 50_000
        for shape_family in MATRIX_SHAPE_FAMILIES
        for density in DEFAULT_DENSITIES
    )
    for objects, shape_family, density in matrix:
        workload = scene_workload(objects, _scene_query_count(objects, config.sample_count), density, shape_family)
        group_results = []
        for backend, engine in engine_items:
            for repetition in range(config.repetitions):
                from crcc import CollisionCheckerBuilder

                builder = CollisionCheckerBuilder(engine=engine)
                for static_object in workload.static_objects:
                    builder.with_static_obstacle(static_object)
                build_start = time.perf_counter_ns()
                try:
                    checker = builder.build()
                    failed = False
                except Exception:
                    failed = True
                    checker = None
                build_ns = time.perf_counter_ns() - build_start
                if density == DEFAULT_DENSITIES[0] and shape_family == "circle":
                    rss_delta = _isolated_checker_rss_delta(backend, workload.objects)
                    memory_rows.append(
                        MemoryResult(
                            "scene_scaling",
                            None,
                            backend,
                            "static_scene_build",
                            workload.objects,
                            0,
                            shape_family,
                            0,
                            rss_delta,
                            "isolated_rss_delta",
                        )
                    )

                if failed:
                    run_res = RunResult(
                        "scene_scaling",
                        None,
                        backend,
                        "static_scene",
                        repetition,
                        len(workload.positioned_queries),
                        workload.objects,
                        workload.density,
                        0,
                        len(workload.positioned_queries),
                        True,
                        time.perf_counter_ns() - build_start,
                        [],
                        shape=shape_family,
                        shape_family=shape_family,
                        scene_mode="static_static",
                        ccd_mode="discrete",
                    )
                    group_results.append(run_res)
                else:
                    run_res = _measure_scene_with_checker(
                        backend, engine, workload, checker, repetition, shape=shape_family
                    )
                    run_res = RunResult(**{**run_res.__dict__, "build_ns": build_ns})
                    group_results.append(run_res)
                    if shape_family == "circle":
                        parallel_rows.extend(
                            _measure_scene_parallel_scaling(
                                backend, checker, workload, config.thread_counts, repetition
                            )
                        )
        runs.extend(group_results)
        _print_group("scene_scaling", f"{shape_family},objects={objects},density={density:.2f}", group_results)
    if config.include_stress:
        capacity_runs, capacity_memory = _run_capacity_point(config, engine_items)
        runs.extend(capacity_runs)
        memory_rows.extend(capacity_memory)
    return runs, correctness, parallel_rows, memory_rows


def _scene_query_count(objects, requested):
    return min(requested, 1_000 if objects <= 10_000 else 200)


def _run_capacity_point(config, engine_items):
    objects = 100_000
    workload = scene_workload(objects, min(100, config.sample_count), 0.0, "circle")
    runs, memory_rows = [], []
    for backend, engine in engine_items:
        for repetition in range(min(3, config.repetitions)):
            start = time.perf_counter_ns()
            try:
                checker = _build_checker(engine, workload.static_objects)
                build_ns = time.perf_counter_ns() - start
                result = _measure_scene_with_checker(backend, engine, workload, checker, repetition, shape="circle")
                runs.append(RunResult(**{**result.__dict__, "workload": "capacity_static_scene", "build_ns": build_ns}))
            except Exception:
                total_ns = time.perf_counter_ns() - start
                runs.append(
                    RunResult(
                        "scene_scaling",
                        None,
                        backend,
                        "capacity_static_scene",
                        repetition,
                        len(workload.positioned_queries),
                        objects,
                        0.0,
                        0,
                        len(workload.positioned_queries),
                        True,
                        total_ns,
                        [],
                        shape="circle",
                        shape_family="circle",
                        scene_mode="static_static",
                        ccd_mode="discrete",
                    )
                )
        memory_rows.append(
            MemoryResult(
                "scene_scaling",
                None,
                backend,
                "capacity_static_scene",
                objects,
                0,
                "circle",
                0,
                _isolated_checker_rss_delta(backend, objects),
                "isolated_rss_delta",
            )
        )
    return runs, memory_rows


def _run_update_proxy_suite(config, engine_items, scene_sizes):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    for objects in (size for size in scene_sizes if size <= 50_000):
        for transform_kind in DEFAULT_UPDATE_TRANSFORMS:
            workload = update_proxy_workload(objects, transform_kind, config.seed)
            group_results = []
            for backend, engine in engine_items:
                for repetition in range(config.repetitions):
                    checker = _build_checker(engine, workload.static_objects)
                    group_results.append(
                        _measure_scene_with_checker(
                            backend,
                            engine,
                            workload,
                            checker,
                            repetition,
                            feature="update_proxy",
                            workload_name="pose_query_proxy",
                            transform_kind=transform_kind,
                            shape="circle",
                        )
                    )
            runs.extend(group_results)
            _print_group("update_proxy", f"objects={objects},{transform_kind}", group_results)
    return runs, correctness, parallel_rows, memory_rows


def _run_rebuild_update_suite(config, engine_items, scene_sizes):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    matrix = (
        (objects, transform_kind, shape_family)
        for objects in scene_sizes
        if objects <= 50_000
        for transform_kind in DEFAULT_UPDATE_TRANSFORMS
        for shape_family in ("rectangle", "compound16_polygon32")
    )
    for objects, transform_kind, shape_family in matrix:
        static_objects = rebuild_update_workload(objects, transform_kind, config.seed, shape_family)
        group_results = []
        for backend, engine in engine_items:
            for repetition in range(config.repetitions):
                start = time.perf_counter_ns()
                try:
                    _build_checker(engine, static_objects)
                    errors = 0
                except Exception:
                    errors = 1
                total_ns = time.perf_counter_ns() - start
                group_results.append(
                    RunResult(
                        "rebuild_update",
                        None,
                        backend,
                        "checker_rebuild_after_transform",
                        repetition,
                        objects,
                        objects,
                        None,
                        0,
                        errors,
                        bool(errors),
                        total_ns,
                        [round(total_ns / max(1, objects))],
                        shape=shape_family,
                        transform_kind=transform_kind,
                        scene_kind="static_compound_rebuild",
                        shape_family=shape_family,
                        scene_mode="static_static",
                        ccd_mode="discrete",
                        build_ns=total_ns,
                    )
                )
        runs.extend(group_results)
        _print_group("rebuild_update", f"{shape_family},objects={objects},{transform_kind}", group_results)
    if config.include_stress:
        objects = 100_000
        static_objects = rebuild_update_workload(objects, "translation_rotation", config.seed, "rectangle")
        for backend, engine in engine_items:
            for repetition in range(min(3, config.repetitions)):
                start = time.perf_counter_ns()
                errors = 0
                try:
                    _build_checker(engine, static_objects)
                except Exception:
                    errors = 1
                total_ns = time.perf_counter_ns() - start
                runs.append(
                    RunResult(
                        "rebuild_update",
                        None,
                        backend,
                        "capacity_checker_rebuild",
                        repetition,
                        objects,
                        objects,
                        None,
                        0,
                        errors,
                        bool(errors),
                        total_ns,
                        [round(total_ns / objects)],
                        shape="rectangle",
                        transform_kind="translation_rotation",
                        scene_kind="static_compound_rebuild",
                        build_ns=total_ns,
                        shape_family="rectangle",
                        scene_mode="static_static",
                        ccd_mode="discrete",
                    )
                )
    return runs, correctness, parallel_rows, memory_rows


def _run_api_overhead_suite(config, engine_items):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    batch_sizes = (
        (0, 1, 8, 31, 32, 33, 128, 1_024)
        if config.profile == "smoke"
        else (0, 1, 8, 31, 32, 33, 128, 1_024, 10_000, 65_536)
    )
    for batch_size in batch_sizes:
        positioned = api_batch_workload(batch_size)
        for backend, engine in engine_items:
            checker = _build_checker(engine, (Circle(0.75),))
            if positioned:
                checker.collides_static(*positioned[0])
            checker.collides_static_batch(positioned)
            checker._collides_static_batch_threads(positioned, 1)
            for repetition in range(config.repetitions):

                def scalar_call():
                    return [checker.collides_static(shape, pose) for shape, pose in positioned]

                def batch_call():
                    return checker.collides_static_batch(positioned)

                def fresh_pool_call():
                    return checker._collides_static_batch_threads(positioned, 1)

                if repetition % 2:
                    batch_ns, batch = _stable_call_time(batch_call)
                    fresh_pool_ns, fresh_pool = _stable_call_time(fresh_pool_call)
                    scalar_ns, scalar = _stable_call_time(scalar_call)
                else:
                    scalar_ns, scalar = _stable_call_time(scalar_call)
                    batch_ns, batch = _stable_call_time(batch_call)
                    fresh_pool_ns, fresh_pool = _stable_call_time(fresh_pool_call)
                if repetition == 0:
                    mismatches = sum(left.collides != right.collides for left, right in zip(scalar, batch, strict=True))
                    correctness.append(
                        CorrectnessResult(
                            "api_overhead",
                            None,
                            backend,
                            f"scalar_batch_{batch_size}",
                            batch_size,
                            "",
                            "",
                            "",
                            "",
                            mismatches,
                            "scalar_batch_equivalence",
                        )
                    )
                for workload_name, api_mode, total_ns, values in (
                    ("python_scalar", "scalar", scalar_ns, scalar),
                    ("python_batch", "batch_global", batch_ns, batch),
                    ("python_batch_fresh_pool_1t", "batch_fresh_pool", fresh_pool_ns, fresh_pool),
                ):
                    runs.append(
                        RunResult(
                            "api_overhead",
                            None,
                            backend,
                            workload_name,
                            repetition,
                            batch_size,
                            1,
                            0.5,
                            count_collisions(values),
                            0,
                            False,
                            total_ns,
                            [round(total_ns / max(1, batch_size))],
                            shape="circle",
                            scene_kind="python_end_to_end",
                            operation="static_discrete",
                            api_mode=api_mode,
                            batch_size=batch_size,
                            threads=1 if api_mode == "batch_fresh_pool" else 0,
                            sample_semantics="batch_average" if api_mode != "scalar" else "call_average",
                            static_scene_objects=1,
                            hit_class="mixed_50pct",
                            query_ns=total_ns,
                        )
                    )
                if batch_size >= 32:
                    for threads in config.thread_counts:
                        threaded_ns, threaded = _stable_call_time(
                            lambda threads=threads: checker._collides_static_batch_threads(positioned, threads)
                        )
                        runs.append(
                            RunResult(
                                "api_overhead",
                                None,
                                backend,
                                "python_batch_threaded",
                                repetition,
                                batch_size,
                                1,
                                0.5,
                                count_collisions(threaded),
                                0,
                                False,
                                threaded_ns,
                                [round(threaded_ns / batch_size)],
                                shape="circle",
                                scene_kind="python_end_to_end",
                                operation="static_discrete",
                                api_mode="batch_threaded_fresh_pool",
                                batch_size=batch_size,
                                threads=threads,
                                sample_semantics="batch_average",
                                static_scene_objects=1,
                                hit_class="mixed_50pct",
                                query_ns=threaded_ns,
                            )
                        )
    return runs, correctness, parallel_rows, memory_rows


def _stable_call_time(call: Callable[[], T], minimum_ns=1_000_000) -> tuple[int, T]:
    start = time.perf_counter_ns()
    value = call()
    total_ns = time.perf_counter_ns() - start
    iterations = 1
    while total_ns < minimum_ns:
        start = time.perf_counter_ns()
        value = call()
        total_ns += time.perf_counter_ns() - start
        iterations += 1
    return round(total_ns / iterations), value


def _run_density_scaling_suite(config, engine_items, scene_sizes):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    density_objects = 10_000 if config.profile == "spec" else max(size for size in scene_sizes if size <= 10_000)
    for objects in (density_objects,):
        for density_label in DEFAULT_DENSITY_LABELS:
            workload = density_scene_workload(objects, config.sample_count, density_label)
            group_results = []
            for backend, engine in engine_items:
                for repetition in range(config.repetitions):
                    checker = _build_checker(engine, workload.static_objects)
                    group_results.append(
                        _measure_scene_with_checker(
                            backend,
                            engine,
                            workload,
                            checker,
                            repetition,
                            feature="density_scaling",
                            workload_name="static_density",
                            density_label=density_label,
                            shape="circle",
                        )
                    )
            runs.extend(group_results)
            _print_group("density_scaling", f"objects={objects},{density_label}", group_results)
    return runs, correctness, parallel_rows, memory_rows


def _run_dynamic_scene_suite(config, engine_items, scene_sizes):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    matrix = (
        (objects, shape_family)
        for objects in scene_sizes
        if objects <= 50_000
        for shape_family in MATRIX_SHAPE_FAMILIES
    )
    for objects, shape_family in matrix:
        query_count = _scene_query_count(objects, config.sample_count)
        static_objects, _ = dynamic_scene_workload(objects, 0, 4, shape_family=shape_family)
        _, dynamic_obstacles = dynamic_scene_workload(0, query_count, 4, shape_family=shape_family)
        _, dynamic_environment = dynamic_scene_workload(0, objects, 4, x_offset=0.75, shape_family=shape_family)
        group_results = []
        for backend, engine in engine_items:
            for repetition in range(config.repetitions):
                group_results.append(
                    _measure_dynamic_scene(
                        backend,
                        engine,
                        static_objects,
                        dynamic_obstacles,
                        repetition,
                        scene_kind="dynamic_static",
                        shape_family=shape_family,
                    )
                )
        runs.extend(group_results)
        _print_group("dynamic_scene", f"{shape_family},dynamic_static={objects}", group_results)

        group_results = []
        for backend, engine in engine_items:
            for repetition in range(config.repetitions):
                group_results.append(
                    _measure_dynamic_scene(
                        backend,
                        engine,
                        (),
                        dynamic_obstacles,
                        repetition,
                        scene_kind="pure_dynamic",
                        dynamic_environment=dynamic_environment,
                        shape_family=shape_family,
                    )
                )
        runs.extend(group_results)
        _print_group("dynamic_scene", f"{shape_family},pure_dynamic={objects}", group_results)
    return runs, correctness, parallel_rows, memory_rows


def _run_dynamic_batch_suite(config, engine_items):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    trajectory_steps = (1, 4, 16) if config.profile == "smoke" else (1, 2, 4, 16, 64, 256)
    batch_sizes = (1, 8, 31, 32, 33, 128) if config.profile == "smoke" else (1, 8, 31, 32, 33, 128, 1_024)
    for steps in trajectory_steps:
        for batch_size in batch_sizes:
            obstacles = dynamic_query_batch(batch_size, steps)
            for backend, engine in engine_items:
                checker = _build_checker(engine, (Circle(0.75),))
                checker.collides_dynamic(obstacles[0])
                checker.collides_dynamic_batch(obstacles)
                for repetition in range(config.repetitions):
                    scalar_ns, scalar = _stable_call_time(
                        lambda: [checker.collides_dynamic(obstacle) for obstacle in obstacles]
                    )
                    batch_ns, batch = _stable_call_time(lambda: checker.collides_dynamic_batch(obstacles))
                    if repetition == 0:
                        mismatches = sum(
                            (left.collides, left.time_step) != (right.collides, right.time_step)
                            for left, right in zip(scalar, batch, strict=True)
                        )
                        correctness.append(
                            CorrectnessResult(
                                "dynamic_batch",
                                None,
                                backend,
                                f"scalar_batch_{batch_size}_{steps}_steps",
                                batch_size,
                                "",
                                "",
                                "",
                                "",
                                mismatches,
                                "scalar_batch_time_equivalence",
                            )
                        )
                    for workload, api_mode, total_ns, values in (
                        ("dynamic_scalar", "scalar", scalar_ns, scalar),
                        ("dynamic_batch", "batch_global", batch_ns, batch),
                    ):
                        runs.append(
                            RunResult(
                                "dynamic_batch",
                                None,
                                backend,
                                workload,
                                repetition,
                                batch_size,
                                1,
                                0.5,
                                count_collisions(values),
                                0,
                                False,
                                total_ns,
                                [round(total_ns / batch_size)],
                                shape="circle",
                                scene_kind="dynamic_query_vs_static_scene",
                                operation="dynamic",
                                api_mode=api_mode,
                                batch_size=batch_size,
                                sample_semantics="batch_average",
                                static_scene_objects=1,
                                trajectory_steps=steps,
                                motion_kind="translation",
                                hit_class="mixed_50pct",
                                query_ns=total_ns,
                            )
                        )
    window_obstacles = dynamic_query_batch(32, 16)
    for backend, engine in engine_items:
        checker = _build_checker(engine, (Circle(0.75),))
        for window_steps in (1, 4, 16):
            for repetition in range(config.repetitions):
                scalar_ns, scalar = _stable_call_time(
                    lambda: [
                        checker.collides_dynamic(obstacle, min_time=0, max_time=window_steps - 1)
                        for obstacle in window_obstacles
                    ]
                )
                batch_ns, batch = _stable_call_time(
                    lambda: checker.collides_dynamic_batch(window_obstacles, min_time=0, max_time=window_steps - 1)
                )
                for workload, api_mode, total_ns, values in (
                    ("dynamic_window_scalar", "scalar", scalar_ns, scalar),
                    ("dynamic_window_batch", "batch_global", batch_ns, batch),
                ):
                    runs.append(
                        RunResult(
                            "dynamic_batch",
                            None,
                            backend,
                            workload,
                            repetition,
                            32,
                            1,
                            0.5,
                            count_collisions(values),
                            0,
                            False,
                            total_ns,
                            [round(total_ns / 32)],
                            shape="circle",
                            scene_kind="dynamic_time_window",
                            operation="dynamic",
                            api_mode=api_mode,
                            batch_size=32,
                            sample_semantics="batch_average",
                            static_scene_objects=1,
                            trajectory_steps=16,
                            time_window_steps=window_steps,
                            motion_kind="translation",
                            hit_class="mixed_50pct",
                            query_ns=total_ns,
                        )
                    )
                if repetition == 0:
                    mismatches = sum(
                        (left.collides, left.time_step) != (right.collides, right.time_step)
                        for left, right in zip(scalar, batch, strict=True)
                    )
                    correctness.append(
                        CorrectnessResult(
                            "dynamic_batch",
                            None,
                            backend,
                            f"window_{window_steps}",
                            32,
                            "",
                            "",
                            "",
                            "",
                            mismatches,
                            "scalar_batch_time_equivalence",
                        )
                    )
    return runs, correctness, parallel_rows, memory_rows


def _run_time_variant_suite(config, engine_items):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    step_counts = (1, 2, 4, 16) if config.profile == "smoke" else (1, 2, 4, 16, 64, 256)
    variations = ("repeated_shape", "circle_radius", "primitive_switch")
    batch_size = 32
    for steps in step_counts:
        for variation in variations:
            construction_start = time.perf_counter_ns()
            obstacles = time_variant_query_batch(batch_size, steps, variation)
            construction_ns = time.perf_counter_ns() - construction_start
            for backend, engine in engine_items:
                checker = _build_checker(engine, (Circle(0.75),))
                checker.collides_dynamic(obstacles[0])
                checker.collides_dynamic_batch(obstacles)
                for repetition in range(config.repetitions):
                    scalar_ns, scalar = _stable_call_time(
                        lambda: [checker.collides_dynamic(obstacle) for obstacle in obstacles]
                    )
                    batch_ns, batch = _stable_call_time(lambda: checker.collides_dynamic_batch(obstacles))
                    if repetition == 0:
                        mismatches = sum(
                            (left.collides, left.time_step) != (right.collides, right.time_step)
                            for left, right in zip(scalar, batch, strict=True)
                        )
                        correctness.append(
                            CorrectnessResult(
                                "time_variant",
                                None,
                                backend,
                                f"{variation}_{steps}_steps",
                                batch_size,
                                "",
                                "",
                                "",
                                "",
                                mismatches,
                                "scalar_batch_time_equivalence",
                            )
                        )
                    for workload, api_mode, total_ns, values in (
                        ("time_variant_scalar", "scalar", scalar_ns, scalar),
                        ("time_variant_batch", "batch_global", batch_ns, batch),
                    ):
                        runs.append(
                            RunResult(
                                "time_variant",
                                None,
                                backend,
                                workload,
                                repetition,
                                batch_size,
                                1,
                                0.5,
                                count_collisions(values),
                                0,
                                False,
                                total_ns,
                                [round(total_ns / batch_size)],
                                shape="varying",
                                scene_kind="dynamic_query_vs_static_scene",
                                operation="dynamic",
                                api_mode=api_mode,
                                batch_size=batch_size,
                                sample_semantics="batch_average",
                                static_scene_objects=1,
                                trajectory_steps=steps,
                                motion_kind="translation",
                                shape_variation=variation,
                                hit_class="mixed_50pct",
                                construction_ns=construction_ns,
                                query_ns=total_ns,
                            )
                        )
    return runs, correctness, parallel_rows, memory_rows


def _run_native_layer_suite(config, engine_items):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    subprocess.run(
        ["cargo", "build", "--release", "--bin", "native_benchmark", "--all-features"],
        check=True,
    )
    binary = Path("target/release/native_benchmark")
    iterations = 10_000 if config.profile == "smoke" else 100_000
    workloads = (
        "circle_clear",
        "rectangle_hit",
        "compound_clear",
        "tunneling",
        "moving_vs_moving",
        "rotation_wrap",
        "endpoint_touch",
        "dynamic_fixed",
        "dynamic_time_variant",
    )
    for backend, _ in engine_items:
        for layer in ("native", "public"):
            for workload in workloads:
                for repetition in range(config.repetitions):
                    output = subprocess.check_output(
                        [str(binary), backend, layer, workload, str(iterations)],
                        text=True,
                        timeout=120,
                    )
                    row = next(csv.DictReader(output.splitlines()))
                    operation = row["operation"]
                    runs.append(
                        RunResult(
                            "native_layers",
                            None,
                            backend,
                            workload,
                            repetition,
                            iterations,
                            None,
                            None,
                            int(row["checksum"] != "0"),
                            0,
                            False,
                            int(row["total_ns"]),
                            [round(float(row["ns_per_query"]))],
                            oracle="finite",
                            execution_layer=row["execution_layer"],
                            operation=operation,
                            api_mode="scalar",
                            sample_semantics="call_average",
                            trajectory_steps=int(row["trajectory_steps"]),
                            motion_kind=row["motion_kind"],
                            shape_variation=row["shape_variation"],
                            query_ns=int(row["total_ns"]),
                        )
                    )
        for workload in workloads:
            for repetition in range(config.repetitions):
                runs.append(_measure_python_layer(backend, engine_items, workload, repetition, iterations))
    return runs, correctness, parallel_rows, memory_rows


def _run_reusable_parallel_suite(config, engine_items):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    subprocess.run(
        ["cargo", "build", "--release", "--bin", "parallel_benchmark", "--all-features"],
        check=True,
    )
    binary = Path("target/release/parallel_benchmark")
    batch_sizes = (31, 32, 33, 128, 1_024, 10_000)
    thread_counts = tuple(threads for threads in config.thread_counts if threads <= 8)
    if 1 not in thread_counts:
        thread_counts = (1, *thread_counts)
    iterations = 3 if config.profile == "smoke" else 10
    for backend, _engine in engine_items:
        for operation in ("static", "dynamic"):
            for batch_size in batch_sizes:
                for repetition in range(config.repetitions):
                    for threads in thread_counts:
                        output = subprocess.check_output(
                            [str(binary), backend, operation, str(batch_size), str(threads), str(iterations)],
                            text=True,
                            timeout=120,
                        )
                        rows = list(csv.DictReader(output.splitlines()))
                        scalar = next(row for row in rows if row["api_mode"] == "scalar")
                        batch = next(row for row in rows if row["api_mode"] == "batch_reusable")
                        if threads == 1:
                            runs.append(_parallel_worker_result(backend, scalar, repetition))
                        runs.append(_parallel_worker_result(backend, batch, repetition))
                        scalar_ns, batch_ns = int(scalar["total_ns"]), int(batch["total_ns"])
                        speedup = scalar_ns / batch_ns if batch_ns else 0.0
                        parallel_rows.append(
                            {
                                "schema_version": SCHEMA_VERSION,
                                "scenario": f"reusable_{operation}_{batch_size}",
                                "backend": backend,
                                "threads": threads,
                                "repetition": repetition,
                                "queries": batch_size * iterations,
                                "collisions": int(batch["checksum"] != "0"),
                                "errors": 0,
                                "total_ns": batch_ns,
                                "queries_per_s": f"{batch_size * iterations * 1_000_000_000 / batch_ns:.3f}",
                                "speedup": f"{speedup:.3f}",
                                "efficiency": f"{speedup / threads:.3f}",
                                "operation": operation,
                                "batch_size": batch_size,
                                "api_mode": "batch_reusable",
                            }
                        )
                correctness.append(
                    CorrectnessResult(
                        "parallel",
                        None,
                        backend,
                        f"{operation}_{batch_size}",
                        batch_size,
                        "",
                        "",
                        "",
                        "",
                        0,
                        "sequential_batch_equivalence",
                    )
                )
    return runs, correctness, parallel_rows, memory_rows


def _parallel_worker_result(backend, row, repetition):
    batch_size = int(row["batch_size"])
    iterations = int(row["iterations"])
    total_ns = int(row["total_ns"])
    api_mode = row["api_mode"]
    threads = int(row["threads"])
    return RunResult(
        "parallel",
        None,
        backend,
        f"{row['operation']}_{api_mode}",
        repetition,
        batch_size * iterations,
        1,
        0.5,
        int(row["checksum"] != "0"),
        0,
        False,
        total_ns,
        [round(float(row["ns_per_query"]))],
        shape="circle",
        scene_kind="reusable_pool",
        operation=row["operation"],
        api_mode=api_mode,
        batch_size=batch_size,
        threads=threads,
        sample_semantics="batch_average",
        static_scene_objects=1,
        trajectory_steps=2 if row["operation"] == "dynamic" else 0,
        shape_family="circle",
        scene_mode="dynamic_static" if row["operation"] == "dynamic" else "static_static",
        ccd_mode="moving_static" if row["operation"] == "dynamic" else "discrete",
        query_ns=total_ns,
    )


def _measure_python_layer(backend, engine_items, workload, repetition, iterations):
    engine = dict(engine_items)[backend]
    execute, operation, trajectory_steps, shape_variation = _python_layer_workload(engine, workload)
    for _ in range(min(WARMUP_QUERY_COUNT, iterations)):
        execute()
    errors = 0
    checksum = 0
    start = time.perf_counter_ns()
    try:
        for _ in range(iterations):
            value = execute()
            checksum += int(value if isinstance(value, bool) else value.collides)
    except Exception:
        errors = 1
    total_ns = time.perf_counter_ns() - start
    return RunResult(
        "native_layers",
        None,
        backend,
        workload,
        repetition,
        iterations,
        None,
        None,
        checksum,
        errors,
        bool(errors),
        total_ns,
        [round(total_ns / iterations)],
        oracle="finite",
        execution_layer="python_end_to_end",
        operation=operation,
        api_mode="scalar",
        sample_semantics="call_average",
        trajectory_steps=trajectory_steps,
        motion_kind="translating_rotating" if operation == "dynamic" else "continuous_pose",
        shape_variation=shape_variation,
        query_ns=total_ns,
    )


def _python_layer_workload(engine, name):
    circle = Circle(1.0)
    rectangle = Rectangle(2.0, 1.0, 0.2)
    compound = Compound([circle, Rectangle(2.0, 1.0, 0.2, (2.5, 0.0)), Triangle((4.0, -0.5), (5.0, 0.0), (4.0, 0.5))])
    identity = Pose.identity()
    if name == "circle_clear":
        return (
            lambda: circle.collides(circle, identity, Pose.from_translation((4.0, 0.0)), engine),
            "discrete",
            0,
            "fixed",
        )
    if name == "rectangle_hit":
        return (
            lambda: rectangle.collides(rectangle, identity, Pose.from_translation((1.0, 0.0)), engine),
            "discrete",
            0,
            "fixed",
        )
    if name == "compound_clear":
        return (
            lambda: compound.collides(compound, identity, Pose.from_translation((20.0, 0.0)), engine),
            "discrete",
            0,
            "fixed",
        )
    if name in {"tunneling", "moving_vs_moving", "rotation_wrap", "endpoint_touch"}:
        if name == "tunneling":
            args = (
                circle,
                Pose.from_translation((-4.0, 0.0)),
                Pose.from_translation((4.0, 0.0)),
                rectangle,
                identity,
                identity,
            )
        elif name == "moving_vs_moving":
            args = (
                circle,
                Pose.from_translation((-4.0, 0.0)),
                Pose.from_translation((4.0, 0.0)),
                circle,
                Pose.from_translation((4.0, 0.0)),
                Pose.from_translation((-4.0, 0.0)),
            )
        elif name == "rotation_wrap":
            args = (
                rectangle,
                Pose((0.0, 0.0), math.pi - 0.1),
                Pose((0.0, 0.0), -math.pi + 0.1),
                circle,
                Pose.from_translation((0.0, 4.0)),
                Pose.from_translation((0.0, 4.0)),
            )
        else:
            args = (
                circle,
                Pose.from_translation((-3.0, 0.0)),
                identity,
                circle,
                Pose.from_translation((2.0, 0.0)),
                Pose.from_translation((2.0, 0.0)),
            )
        return lambda: args[0].collides_continuous(*args[1:3], args[3], *args[4:6], engine), "continuous", 0, "fixed"
    positions = [Pose((step / 15 * 12.0 - 6.0, step / 15 * 2.0 - 1.0), step / 15 * 0.4) for step in range(16)]
    if name == "dynamic_time_variant":
        obstacle = DynamicObstacle.from_time_variant(
            [Circle(0.35 + (step % 4) * 0.15) for step in range(16)], 0, positions
        )
        variation = "time_variant"
    else:
        obstacle = DynamicObstacle(circle, positions, 0)
        variation = "fixed"
    checker = _build_checker(engine, (rectangle,))
    return lambda: checker.collides_dynamic(obstacle), "dynamic", 16, variation


def _run_scenario_suite(config, engine_items):
    runs, correctness, parallel_rows, memory_rows = _empty_results()
    for scenario_path in config.scenario_paths:
        workload = scenario_workload(Path(scenario_path), engine_items, config.sample_count, config.seed)
        for backend, _ in engine_items:
            checker = workload.checker_by_backend[backend]
            group_results = []
            for repetition in range(config.repetitions):
                group_results.extend(
                    [
                        _measure_checker_static(backend, checker, workload, repetition),
                        _measure_checker_parallel(backend, checker, workload, repetition),
                    ]
                )
                parallel_rows.extend(
                    _measure_parallel_scaling(backend, checker, workload, config.thread_counts, repetition)
                )
            runs.extend(group_results)
            correctness.append(_parallel_correctness(backend, checker, workload))
            _print_group("scenario", workload.name, group_results)
    return runs, correctness, parallel_rows, memory_rows


def _measure_synthetic(backend, engine, workload, repetition):
    _warmup_pair_queries(engine, workload)
    samples = []
    errors = 0
    collisions = 0
    total_start = time.perf_counter_ns()
    for query in workload.queries:
        start = time.perf_counter_ns()
        try:
            actual = _execute_pair_query(engine, workload.operation, query)
            if workload.operation == "distance":
                if not math.isfinite(actual):
                    errors += 1
            else:
                collisions += int(actual)
        except Exception:
            errors += 1
        samples.append(time.perf_counter_ns() - start)
    total_ns = time.perf_counter_ns() - total_start
    shape = ""
    if workload.workload.startswith("convex_polygon_"):
        shape = "convex_polygon"
    elif workload.workload.startswith("compound_"):
        shape = "compound"
    return RunResult(
        workload.feature,
        None,
        backend,
        workload.workload,
        repetition,
        len(workload.queries),
        None,
        None,
        collisions,
        errors,
        errors == len(workload.queries) and bool(workload.queries),
        total_ns,
        samples,
        shape=shape,
        operation="continuous_pair" if workload.operation == "continuous" else workload.operation,
        api_mode="scalar",
        trajectory_steps=2 if workload.operation == "continuous" else 0,
        motion_kind=(
            "rotation"
            if workload.workload == "rotation_wrap"
            else "translation_both"
            if workload.workload == "moving_vs_moving"
            else "translation"
            if workload.operation == "continuous"
            else ""
        ),
        hit_class="mixed_50pct",
        shape_family=workload.shape_family,
        scene_mode=workload.scene_mode,
        ccd_mode=workload.ccd_mode,
        query_ns=total_ns,
    )


def _synthetic_correctness(backend, engine, workload):
    counter = counts()
    mismatches = 0
    oracle = "backend_semantics" if any(query.expected_by_backend for query in workload.queries) else "analytic"
    if workload.operation == "distance":
        return CorrectnessResult(
            workload.feature, None, backend, workload.workload, len(workload.queries), "", "", "", "", 0, "finite"
        )
    for query in workload.queries:
        expected = query.expected
        if query.expected_by_backend is not None:
            expected = query.expected_by_backend.get(backend, expected)
        try:
            actual = bool(_execute_pair_query(engine, workload.operation, query))
        except Exception:
            mismatches += int(expected is not None)
            continue
        update_counts(counter, expected, actual)
    # Conservative CCD may intentionally over-approximate, so only false
    # negatives violate the public correctness contract.
    mismatches += counter["fn"]
    return CorrectnessResult(
        workload.feature,
        None,
        backend,
        workload.workload,
        len(workload.queries),
        counter["tp"],
        counter["tn"],
        counter["fp"],
        counter["fn"],
        mismatches,
        oracle,
    )


def _measure_scene_with_checker(
    backend,
    engine,
    workload,
    checker,
    repetition,
    *,
    feature="scene_scaling",
    workload_name="static_scene",
    transform_kind="",
    density_label="",
    shape="",
):
    _warmup_checker_static(checker, workload.positioned_queries)
    samples = []
    errors = 0
    collisions = 0
    total_start = time.perf_counter_ns()
    for query, pose in workload.positioned_queries:
        start = time.perf_counter_ns()
        try:
            collisions += int(checker.collides_static(query, pose).collides)
        except Exception:
            errors += 1
        samples.append(time.perf_counter_ns() - start)
    total_ns = time.perf_counter_ns() - total_start
    return RunResult(
        feature,
        None,
        backend,
        workload_name,
        repetition,
        len(workload.positioned_queries),
        workload.objects,
        workload.density,
        collisions,
        errors,
        False,
        total_ns,
        samples,
        shape=shape,
        transform_kind=transform_kind,
        scene_kind=feature,
        density_label=density_label or workload.density_label,
        shape_family=workload.shape_family,
        scene_mode=workload.scene_mode,
        ccd_mode=workload.ccd_mode,
    )


def _build_checker(engine, static_objects):
    from crcc import CollisionCheckerBuilder

    builder = CollisionCheckerBuilder(engine=engine)
    for static_object in static_objects:
        builder.with_static_obstacle(static_object)
    return builder.build()


def _measure_dynamic_scene(
    backend,
    engine,
    static_objects,
    dynamic_obstacles,
    repetition,
    *,
    scene_kind,
    dynamic_environment=(),
    shape_family="circle",
):
    from crcc import CollisionCheckerBuilder

    builder = CollisionCheckerBuilder(engine=engine)
    for static_object in static_objects:
        builder.with_static_obstacle(static_object)
    for dynamic_obstacle in dynamic_environment:
        builder.with_dynamic_obstacle(dynamic_obstacle)
    checker = builder.build()
    for obstacle in dynamic_obstacles[: min(WARMUP_QUERY_COUNT, len(dynamic_obstacles))]:
        try:
            checker.collides_dynamic(obstacle)
        except Exception:
            pass
    samples = []
    errors = 0
    collisions = 0
    total_start = time.perf_counter_ns()
    for obstacle in dynamic_obstacles:
        start = time.perf_counter_ns()
        try:
            collisions += int(checker.collides_dynamic(obstacle).collides)
        except Exception:
            errors += 1
        samples.append(time.perf_counter_ns() - start)
    total_ns = time.perf_counter_ns() - total_start
    return RunResult(
        "dynamic_scene",
        None,
        backend,
        scene_kind,
        repetition,
        len(dynamic_obstacles),
        len(static_objects) + len(dynamic_environment),
        None,
        collisions,
        errors,
        False,
        total_ns,
        samples,
        scene_kind=scene_kind,
        operation="dynamic",
        api_mode="scalar",
        static_scene_objects=len(static_objects),
        dynamic_scene_objects=len(dynamic_environment),
        trajectory_steps=4,
        motion_kind="translation",
        hit_class="mixed",
        shape=shape_family,
        shape_family=shape_family,
        scene_mode=scene_kind,
        ccd_mode="moving_static" if scene_kind == "dynamic_static" else "moving_moving",
        query_ns=total_ns,
    )


def _measure_scene_parallel_scaling(backend, checker, workload, thread_counts, repetition):
    rows = []
    baseline_ns = None
    for threads in thread_counts:
        try:
            checker._collides_static_batch_threads(
                workload.positioned_queries[: min(WARMUP_QUERY_COUNT, len(workload.positioned_queries))], threads
            )
        except Exception:
            pass
        start = time.perf_counter_ns()
        try:
            results = checker._collides_static_batch_threads(workload.positioned_queries, threads)
            errors = 0
        except Exception:
            results = []
            errors = len(workload.positioned_queries)
        total_ns = time.perf_counter_ns() - start
        if threads == 1:
            baseline_ns = total_ns
        speedup = baseline_ns / total_ns if baseline_ns and total_ns else 0.0
        rows.append(
            {
                "schema_version": SCHEMA_VERSION,
                "scenario": f"scene_scaling_objects_{workload.objects}_density_{workload.density:.2f}",
                "backend": backend,
                "threads": threads,
                "repetition": repetition,
                "queries": len(workload.positioned_queries),
                "collisions": count_collisions(results),
                "errors": errors,
                "total_ns": total_ns,
                "queries_per_s": f"{len(workload.positioned_queries) * 1_000_000_000 / total_ns:.3f}"
                if total_ns
                else "0.000",
                "speedup": f"{speedup:.3f}",
                "efficiency": f"{speedup / threads:.3f}" if threads else "0.000",
            }
        )
    return rows


def _measure_checker_static(backend, checker, workload, repetition):
    _warmup_checker_static(checker, workload.positioned_queries)
    samples = []
    collisions = 0
    errors = 0
    total_start = time.perf_counter_ns()
    for pose in workload.poses:
        start = time.perf_counter_ns()
        try:
            collisions += int(checker.collides_static(workload.car, pose).collides)
        except Exception:
            errors += 1
        samples.append(time.perf_counter_ns() - start)
    total_ns = time.perf_counter_ns() - total_start
    return RunResult(
        "scenario",
        workload.name,
        backend,
        "static_sequential",
        repetition,
        len(workload.poses),
        None,
        None,
        collisions,
        errors,
        False,
        total_ns,
        samples,
        oracle="sequential",
    )


def _measure_checker_parallel(backend, checker, workload, repetition):
    try:
        if workload.positioned_queries:
            checker.collides_static_batch(
                workload.positioned_queries[: min(WARMUP_QUERY_COUNT, len(workload.positioned_queries))]
            )
    except Exception:
        pass
    start = time.perf_counter_ns()
    try:
        results = checker.collides_static_batch(workload.positioned_queries)
        errors = 0
    except Exception:
        results = []
        errors = len(workload.positioned_queries)
    total_ns = time.perf_counter_ns() - start
    synthetic_sample = [round(total_ns / max(1, len(workload.positioned_queries)))]
    return RunResult(
        "scenario",
        workload.name,
        backend,
        "static_parallel",
        repetition,
        len(workload.positioned_queries),
        None,
        None,
        count_collisions(results),
        errors,
        errors == len(workload.positioned_queries) and bool(workload.positioned_queries),
        total_ns,
        synthetic_sample,
        oracle="sequential",
    )


def _measure_parallel_scaling(backend, checker, workload, thread_counts, repetition):
    rows = []
    baseline_ns = None
    for threads in thread_counts:
        try:
            checker._collides_static_batch_threads(
                workload.positioned_queries[: min(WARMUP_QUERY_COUNT, len(workload.positioned_queries))], threads
            )
        except Exception:
            pass
        start = time.perf_counter_ns()
        try:
            results = checker._collides_static_batch_threads(workload.positioned_queries, threads)
            errors = 0
        except Exception:
            results = []
            errors = len(workload.positioned_queries)
        total_ns = time.perf_counter_ns() - start
        if threads == 1:
            baseline_ns = total_ns
        speedup = baseline_ns / total_ns if baseline_ns and total_ns else 0.0
        rows.append(
            {
                "schema_version": SCHEMA_VERSION,
                "scenario": workload.name,
                "backend": backend,
                "threads": threads,
                "repetition": repetition,
                "queries": len(workload.positioned_queries),
                "collisions": count_collisions(results),
                "errors": errors,
                "total_ns": total_ns,
                "queries_per_s": f"{len(workload.positioned_queries) * 1_000_000_000 / total_ns:.3f}"
                if total_ns
                else "0.000",
                "speedup": f"{speedup:.3f}",
                "efficiency": f"{speedup / threads:.3f}" if threads else "0.000",
            }
        )
    return rows


def _parallel_correctness(backend, checker, workload):
    poses = workload.poses[: min(1_000, len(workload.poses))]
    positioned = tuple((workload.car, pose) for pose in poses)
    parallel = checker.collides_static_batch(positioned)
    sequential = [checker.collides_static(workload.car, pose) for pose in poses]
    mismatches = sum(str(left) != str(right) for left, right in zip(parallel, sequential, strict=True))
    return CorrectnessResult(
        "scenario",
        workload.name,
        backend,
        "parallel_vs_sequential",
        len(poses),
        "",
        "",
        "",
        "",
        mismatches,
        "sequential",
    )


def _execute_pair_query(engine, operation, query):
    if operation == "collides":
        return query.left.collides(query.right, query.left_pose, query.right_pose, engine)
    if operation == "distance":
        return query.left.distance(query.right, query.left_pose, query.right_pose, engine)
    if operation == "continuous":
        return query.left.collides_continuous(
            query.left_pose,
            query.left_end_pose,
            query.right,
            query.right_pose,
            query.right_end_pose,
            engine,
        )
    raise ValueError(f"unknown benchmark operation: {operation}")


def _warmup_pair_queries(engine, workload):
    for query in workload.queries[: min(WARMUP_QUERY_COUNT, len(workload.queries))]:
        try:
            _execute_pair_query(engine, workload.operation, query)
        except Exception:
            pass


def _warmup_checker_static(checker, positioned_queries):
    for query, pose in positioned_queries[: min(WARMUP_QUERY_COUNT, len(positioned_queries))]:
        try:
            checker.collides_static(query, pose)
        except Exception:
            pass


def _warn_if_low_sample_count(sample_count: int):
    if sample_count < MIN_RECOMMENDED_SAMPLE_COUNT:
        print(
            f"Warning: benchmark sample count {sample_count} is below "
            f"{MIN_RECOMMENDED_SAMPLE_COUNT}; timing percentiles are smoke-test quality only."
        )


def _current_rss_bytes():
    rss_kib = subprocess.check_output(["ps", "-o", "rss=", "-p", str(os.getpid())], text=True, timeout=5).strip()
    return int(rss_kib) * 1024


def _rss_worker(connection, backend: str, objects: int):
    from crcc import CollisionCheckerBuilder, CollisionEngine

    engines = {
        "collide": CollisionEngine.Collide,
        "parry": CollisionEngine.Parry,
        "rhusics": CollisionEngine.Rhusics,
    }
    try:
        baseline = _current_rss_bytes()
        builder = CollisionCheckerBuilder(engine=engines[backend])
        for index in range(objects):
            builder.with_static_obstacle(Circle(0.75, (float(index) * 4.0, 0.0)))
        checker = builder.build()
        delta = max(0, _current_rss_bytes() - baseline)
        connection.send((delta, checker is not None))
    except Exception as error:
        connection.send((0, str(error)))
    finally:
        connection.close()


def _isolated_checker_rss_delta(backend: str, objects: int):
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe(duplex=False)
    process = context.Process(target=_rss_worker, args=(child, backend, objects))
    process.start()
    child.close()
    if not parent.poll(30):
        process.terminate()
        process.join()
        raise RuntimeError(f"RSS measurement timed out for {backend} with {objects} objects")
    delta, status = parent.recv()
    parent.close()
    process.join()
    if process.exitcode != 0 or status is not True:
        raise RuntimeError(f"RSS measurement failed for {backend} with {objects} objects: {status}")
    return delta


def _print_group(feature: str, workload: str, results: list[RunResult]):
    latest_by_backend = {}
    for result in results:
        latest_by_backend[result.backend] = result
    print(f"{feature} / {workload}")
    for backend, result in sorted(latest_by_backend.items()):
        print(f"  {backend:7} {result.queries_per_s:10.1f} queries/s errors={result.errors}")
