import math
import time
from pathlib import Path

from examples.utils import count_collisions

from .config import (
    DEFAULT_DENSITIES,
    DEFAULT_SCENE_SIZES,
    MIN_RECOMMENDED_SAMPLE_COUNT,
    WARMUP_QUERY_COUNT,
    BenchmarkConfig,
    selected_engine_items,
)
from .io import write_artifacts
from .plots import write_plots
from .results import CorrectnessResult, RunResult, counts, update_counts
from .workloads import scenario_workload, scene_workload, synthetic_workloads


def run_all(
    scenario_paths=None,
    sample_count=None,
    output_dir=None,
    seed=2026,
    thread_counts=None,
    engines=None,
    repetitions=None,
    step="all",
):
    config = BenchmarkConfig.from_args(
        scenario_paths=scenario_paths,
        sample_count=BenchmarkConfig.__dataclass_fields__["sample_count"].default if sample_count is None else sample_count,
        repetitions=BenchmarkConfig.__dataclass_fields__["repetitions"].default if repetitions is None else repetitions,
        output_dir=BenchmarkConfig.__dataclass_fields__["output_dir"].default if output_dir is None else output_dir,
        seed=seed,
        thread_counts=thread_counts,
        engines=engines,
        step=step,
    )
    if config.step in {"run", "all"}:
        _warn_if_low_sample_count(config.sample_count)
        runs, correctness, parallel_rows = run_benchmarks(config)
        write_artifacts(config, runs, correctness, parallel_rows)
        print(f"Wrote benchmark CSV files to {config.output_dir}")
    if config.step in {"plot", "all"}:
        write_plots(config.output_dir)
        print(f"Wrote benchmark plots to {config.output_dir / 'plots'}")


def run_benchmarks(config: BenchmarkConfig):
    engine_items = selected_engine_items(config.engines)
    runs: list[RunResult] = []
    correctness: list[CorrectnessResult] = []
    parallel_rows: list[dict[str, str | int]] = []

    for workload in synthetic_workloads(config.sample_count, config.seed):
        group_results = []
        for backend, engine in engine_items:
            correctness.append(_synthetic_correctness(backend, engine, workload))
            for repetition in range(config.repetitions):
                group_results.append(_measure_synthetic(backend, engine, workload, repetition))
        runs.extend(group_results)
        _print_group(workload.feature, workload.workload, group_results)

    for objects in DEFAULT_SCENE_SIZES:
        for density in DEFAULT_DENSITIES:
            workload = scene_workload(objects, config.sample_count, density)
            group_results = []
            for backend, engine in engine_items:
                for repetition in range(config.repetitions):
                    from crcc.collision_checker import CollisionCheckerBuilder
                    builder = CollisionCheckerBuilder(engine=engine)
                    for static_object in workload.static_objects:
                        builder.with_static_obstacle(static_object)
                    build_start = time.perf_counter_ns()
                    try:
                        checker = builder.build()
                        failed = False
                    except Exception:
                        failed = True

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
                        )
                        group_results.append(run_res)
                    else:
                        run_res = _measure_scene_with_checker(backend, engine, workload, checker, repetition)
                        group_results.append(run_res)
                        parallel_rows.extend(_measure_scene_parallel_scaling(backend, checker, workload, config.thread_counts, repetition))
            runs.extend(group_results)
            _print_group("scene_scaling", f"objects={objects},density={density:.2f}", group_results)

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
                parallel_rows.extend(_measure_parallel_scaling(backend, checker, workload, config.thread_counts, repetition))
            runs.extend(group_results)
            correctness.append(_parallel_correctness(backend, checker, workload))
            _print_group("scenario", workload.name, group_results)
    return runs, correctness, parallel_rows


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
    )


def _synthetic_correctness(backend, engine, workload):
    counter = counts()
    mismatches = 0
    if workload.operation == "distance":
        return CorrectnessResult(workload.feature, None, backend, workload.workload, len(workload.queries), "", "", "", "", 0, "finite")
    for query in workload.queries:
        try:
            actual = bool(_execute_pair_query(engine, workload.operation, query))
        except Exception:
            actual = False
            mismatches += int(query.expected is not None)
        update_counts(counter, query.expected, actual)
    mismatches += counter["fp"] + counter["fn"]
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
        "analytic",
    )


def _measure_scene_with_checker(backend, engine, workload, checker, repetition):
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
        "scene_scaling",
        None,
        backend,
        "static_scene",
        repetition,
        len(workload.positioned_queries),
        workload.objects,
        workload.density,
        collisions,
        errors,
        False,
        total_ns,
        samples,
    )


def _measure_scene_parallel_scaling(backend, checker, workload, thread_counts, repetition):
    rows = []
    baseline_ns = None
    for threads in thread_counts:
        try:
            checker.par_static_threads(workload.positioned_queries[: min(WARMUP_QUERY_COUNT, len(workload.positioned_queries))], threads)
        except Exception:
            pass
        start = time.perf_counter_ns()
        try:
            results = checker.par_static_threads(workload.positioned_queries, threads)
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
                "schema_version": "2",
                "scenario": f"scene_scaling_objects_{workload.objects}_density_{workload.density:.2f}",
                "backend": backend,
                "threads": threads,
                "repetition": repetition,
                "queries": len(workload.positioned_queries),
                "collisions": count_collisions(results),
                "errors": errors,
                "total_ns": total_ns,
                "queries_per_s": f"{len(workload.positioned_queries) * 1_000_000_000 / total_ns:.3f}" if total_ns else "0.000",
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
            checker.par_static(workload.positioned_queries[: min(WARMUP_QUERY_COUNT, len(workload.positioned_queries))])
    except Exception:
        pass
    start = time.perf_counter_ns()
    try:
        results = checker.par_static(workload.positioned_queries)
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
            checker.par_static_threads(workload.positioned_queries[: min(WARMUP_QUERY_COUNT, len(workload.positioned_queries))], threads)
        except Exception:
            pass
        start = time.perf_counter_ns()
        try:
            results = checker.par_static_threads(workload.positioned_queries, threads)
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
                "schema_version": "2",
                "scenario": workload.name,
                "backend": backend,
                "threads": threads,
                "repetition": repetition,
                "queries": len(workload.positioned_queries),
                "collisions": count_collisions(results),
                "errors": errors,
                "total_ns": total_ns,
                "queries_per_s": f"{len(workload.positioned_queries) * 1_000_000_000 / total_ns:.3f}" if total_ns else "0.000",
                "speedup": f"{speedup:.3f}",
                "efficiency": f"{speedup / threads:.3f}" if threads else "0.000",
            }
        )
    return rows


def _parallel_correctness(backend, checker, workload):
    poses = workload.poses[: min(1_000, len(workload.poses))]
    positioned = tuple((workload.car, pose) for pose in poses)
    parallel = checker.par_static(positioned)
    sequential = [checker.collides_static(workload.car, pose) for pose in poses]
    mismatches = sum(str(left) != str(right) for left, right in zip(parallel, sequential, strict=True))
    return CorrectnessResult("scenario", workload.name, backend, "parallel_vs_sequential", len(poses), "", "", "", "", mismatches, "sequential")


def _execute_pair_query(engine, operation, query):
    if operation == "collides":
        return query.left.collides(query.right, query.left_pose, query.right_pose, engine)
    if operation == "distance":
        return query.left.distance(query.right, query.left_pose, query.right_pose, engine)
    if operation == "ccd":
        return query.left.collides_sweep(
            query.left_pose,
            type(query.left_pose)(
                (query.left_pose.translation[0] + 8.0, query.left_pose.translation[1]),
                query.left_pose.rotation,
            ),
            query.right,
            query.right_pose,
            query.right_pose,
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


def _print_group(feature: str, workload: str, results: list[RunResult]):
    latest_by_backend = {}
    for result in results:
        latest_by_backend[result.backend] = result
    print(f"{feature} / {workload}")
    for backend, result in sorted(latest_by_backend.items()):
        print(f"  {backend:7} {result.queries_per_s:10.1f} queries/s errors={result.errors}")
