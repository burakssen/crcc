import random
import statistics
from dataclasses import dataclass
from typing import Any

from .config import SCHEMA_VERSION


@dataclass(frozen=True)
class RunResult:
    feature: str
    scenario: str | None
    backend: str
    workload: str
    repetition: int
    queries: int
    objects: int | None
    density: float | None
    collisions: int
    errors: int
    unsupported: bool
    total_ns: int
    samples_ns: list[int]
    oracle: str = "analytic"
    shape: str = ""
    transform_kind: str = ""
    scene_kind: str = ""
    density_label: str = ""
    build_ns: int = 0
    execution_layer: str = "python_end_to_end"
    operation: str = ""
    api_mode: str = "scalar"
    batch_size: int = 0
    threads: int = 0
    sample_semantics: str = "per_query"
    static_scene_objects: int = 0
    dynamic_scene_objects: int = 0
    trajectory_steps: int = 0
    time_window_steps: int = 0
    motion_kind: str = ""
    shape_variation: str = "fixed"
    hit_class: str = ""
    shape_family: str = ""
    scene_mode: str = ""
    ccd_mode: str = ""
    construction_ns: int = 0
    query_ns: int = 0
    deadline_ns: int = 0
    deadline_misses: int = 0
    cache_state: str = ""
    candidate_count: int = 0

    @property
    def queries_per_s(self) -> float:
        return self.queries * 1_000_000_000 / self.total_ns if self.total_ns else 0.0

    @property
    def ns_per_query(self) -> float:
        return self.total_ns / self.queries if self.queries else 0.0


@dataclass(frozen=True)
class CorrectnessResult:
    feature: str
    scenario: str | None
    backend: str
    workload: str
    queries: int
    true_positive: int | str
    true_negative: int | str
    false_positive: int | str
    false_negative: int | str
    mismatches: int
    oracle: str
    errors: int = 0


@dataclass(frozen=True)
class MemoryResult:
    feature: str
    scenario: str | None
    backend: str
    workload: str
    objects: int | None
    queries: int
    shape: str
    current_bytes: int
    peak_bytes: int
    measurement: str = "python_heap"

    @property
    def bytes_per_object(self) -> float:
        return self.peak_bytes / self.objects if self.objects else 0.0


RUN_FIELDS = [
    "schema_version",
    "feature",
    "scenario",
    "backend",
    "workload",
    "repetition",
    "queries",
    "objects",
    "density",
    "collisions",
    "errors",
    "unsupported",
    "total_ns",
    "queries_per_s",
    "ns_per_query",
    "min_ns",
    "p50_ns",
    "p90_ns",
    "p95_ns",
    "p99_ns",
    "max_ns",
    "oracle",
    "shape",
    "transform_kind",
    "scene_kind",
    "density_label",
    "build_ns",
    "execution_layer",
    "operation",
    "api_mode",
    "batch_size",
    "threads",
    "sample_semantics",
    "static_scene_objects",
    "dynamic_scene_objects",
    "trajectory_steps",
    "time_window_steps",
    "motion_kind",
    "shape_variation",
    "hit_class",
    "shape_family",
    "scene_mode",
    "ccd_mode",
    "construction_ns",
    "query_ns",
    "deadline_ns",
    "deadline_misses",
    "cache_state",
    "candidate_count",
]

SUMMARY_FIELDS = [
    "schema_version",
    "feature",
    "scenario",
    "backend",
    "workload",
    "queries",
    "objects",
    "density",
    "repetitions",
    "collisions_median",
    "errors_total",
    "unsupported",
    "throughput_median",
    "throughput_mean",
    "ns_per_query_median",
    "p50_ns_median",
    "p90_ns_median",
    "p95_ns_median",
    "p99_ns_median",
    "oracle",
    "shape",
    "transform_kind",
    "scene_kind",
    "density_label",
    "total_ns_mean",
    "total_ns_min",
    "total_ns_max",
    "total_ns_stddev",
    "build_ns_median",
    "execution_layer",
    "operation",
    "api_mode",
    "batch_size",
    "threads",
    "sample_semantics",
    "static_scene_objects",
    "dynamic_scene_objects",
    "trajectory_steps",
    "time_window_steps",
    "motion_kind",
    "shape_variation",
    "hit_class",
    "shape_family",
    "scene_mode",
    "ccd_mode",
    "construction_ns_median",
    "query_ns_median",
    "deadline_ns",
    "deadline_misses",
    "deadline_miss_rate",
    "cache_state",
    "candidate_count",
]

COMPARISON_FIELDS = [
    "schema_version",
    "feature",
    "scenario",
    "workload",
    "queries",
    "objects",
    "density",
    "baseline_backend",
    "backend",
    "paired_repetitions",
    "baseline_throughput_median",
    "backend_throughput_median",
    "speedup_median",
    "speedup_q25",
    "speedup_q75",
    "speedup_ci_low",
    "speedup_ci_high",
    "verdict",
    "execution_layer",
    "operation",
    "api_mode",
    "batch_size",
    "threads",
    "trajectory_steps",
    "motion_kind",
    "shape_variation",
    "shape_family",
    "scene_mode",
    "ccd_mode",
    "deadline_ns",
    "cache_state",
    "candidate_count",
]

MODE_COMPARISON_FIELDS = [
    "schema_version",
    "feature",
    "backend",
    "execution_layer",
    "baseline_mode",
    "candidate_mode",
    "batch_size",
    "threads",
    "trajectory_steps",
    "time_window_steps",
    "shape_variation",
    "paired_repetitions",
    "ratio_median",
    "ratio_q25",
    "ratio_q75",
    "ratio_ci_low",
    "ratio_ci_high",
    "verdict",
    "static_scene_objects",
    "dynamic_scene_objects",
    "cache_state",
    "candidate_count",
]

LAYER_COMPARISON_FIELDS = [
    "schema_version",
    "feature",
    "backend",
    "workload",
    "operation",
    "trajectory_steps",
    "shape_variation",
    "queries",
    "paired_repetitions",
    "native_ns_median",
    "public_ns_median",
    "public_native_ratio_median",
    "python_ns_median",
    "python_public_ratio_median",
    "python_ratio_q25",
    "python_ratio_q75",
    "python_ratio_ci_low",
    "python_ratio_ci_high",
    "ratio_q25",
    "ratio_q75",
    "ratio_ci_low",
    "ratio_ci_high",
    "verdict",
]

CORRECTNESS_FIELDS = [
    "schema_version",
    "feature",
    "scenario",
    "backend",
    "workload",
    "queries",
    "true_positive",
    "true_negative",
    "false_positive",
    "false_negative",
    "mismatches",
    "errors",
    "oracle",
]

PARALLEL_SCALING_FIELDS = [
    "schema_version",
    "scenario",
    "backend",
    "threads",
    "repetition",
    "queries",
    "collisions",
    "errors",
    "total_ns",
    "queries_per_s",
    "speedup",
    "efficiency",
    "operation",
    "batch_size",
    "api_mode",
]

MEMORY_FIELDS = [
    "schema_version",
    "feature",
    "scenario",
    "backend",
    "workload",
    "objects",
    "queries",
    "shape",
    "current_bytes",
    "peak_bytes",
    "bytes_per_object",
    "measurement",
]


def run_row(result: RunResult):
    sample_percentiles = _sample_percentiles(result.samples_ns)
    return {
        "schema_version": SCHEMA_VERSION,
        "feature": result.feature,
        "scenario": result.scenario or "",
        "backend": result.backend,
        "workload": result.workload,
        "repetition": result.repetition,
        "queries": result.queries,
        "objects": "" if result.objects is None else result.objects,
        "density": "" if result.density is None else f"{result.density:.3f}",
        "collisions": result.collisions,
        "errors": result.errors,
        "unsupported": result.unsupported,
        "total_ns": result.total_ns,
        "queries_per_s": f"{result.queries_per_s:.3f}",
        "ns_per_query": f"{result.ns_per_query:.3f}",
        **sample_percentiles,
        "oracle": result.oracle,
        "shape": result.shape,
        "transform_kind": result.transform_kind,
        "scene_kind": result.scene_kind,
        "density_label": result.density_label,
        "build_ns": result.build_ns,
        "execution_layer": result.execution_layer,
        "operation": result.operation,
        "api_mode": result.api_mode,
        "batch_size": result.batch_size,
        "threads": result.threads,
        "sample_semantics": result.sample_semantics,
        "static_scene_objects": result.static_scene_objects,
        "dynamic_scene_objects": result.dynamic_scene_objects,
        "trajectory_steps": result.trajectory_steps,
        "time_window_steps": result.time_window_steps,
        "motion_kind": result.motion_kind,
        "shape_variation": result.shape_variation,
        "hit_class": result.hit_class,
        "shape_family": result.shape_family,
        "scene_mode": result.scene_mode,
        "ccd_mode": result.ccd_mode,
        "construction_ns": result.construction_ns,
        "query_ns": result.query_ns,
        "deadline_ns": result.deadline_ns,
        "deadline_misses": result.deadline_misses,
        "cache_state": result.cache_state,
        "candidate_count": result.candidate_count,
    }


def correctness_row(result: CorrectnessResult):
    return {
        "schema_version": SCHEMA_VERSION,
        "feature": result.feature,
        "scenario": result.scenario or "",
        "backend": result.backend,
        "workload": result.workload,
        "queries": result.queries,
        "true_positive": result.true_positive,
        "true_negative": result.true_negative,
        "false_positive": result.false_positive,
        "false_negative": result.false_negative,
        "mismatches": result.mismatches,
        "errors": result.errors,
        "oracle": result.oracle,
    }


def memory_row(result: MemoryResult):
    return {
        "schema_version": SCHEMA_VERSION,
        "feature": result.feature,
        "scenario": result.scenario or "",
        "backend": result.backend,
        "workload": result.workload,
        "objects": "" if result.objects is None else result.objects,
        "queries": result.queries,
        "shape": result.shape,
        "current_bytes": result.current_bytes,
        "peak_bytes": result.peak_bytes,
        "bytes_per_object": f"{result.bytes_per_object:.3f}",
        "measurement": result.measurement,
    }


def summarize_runs(results: list[RunResult]):
    rows = []
    grouped: dict[tuple[Any, ...], list[RunResult]] = {}
    for result in results:
        key = (
            result.feature,
            result.scenario,
            result.backend,
            result.workload,
            result.queries,
            result.objects,
            result.density,
            result.oracle,
            result.shape,
            result.transform_kind,
            result.scene_kind,
            result.density_label,
            result.execution_layer,
            result.operation,
            result.api_mode,
            result.batch_size,
            result.threads,
            result.sample_semantics,
            result.static_scene_objects,
            result.dynamic_scene_objects,
            result.trajectory_steps,
            result.time_window_steps,
            result.motion_kind,
            result.shape_variation,
            result.hit_class,
            result.shape_family,
            result.scene_mode,
            result.ccd_mode,
            result.deadline_ns,
            result.cache_state,
            result.candidate_count,
        )
        grouped.setdefault(key, []).append(result)

    for (
        feature,
        scenario,
        backend,
        workload,
        queries,
        objects,
        density,
        oracle,
        shape,
        transform_kind,
        scene_kind,
        density_label,
        execution_layer,
        operation,
        api_mode,
        batch_size,
        threads,
        sample_semantics,
        static_scene_objects,
        dynamic_scene_objects,
        trajectory_steps,
        time_window_steps,
        motion_kind,
        shape_variation,
        hit_class,
        shape_family,
        scene_mode,
        ccd_mode,
        deadline_ns,
        cache_state,
        candidate_count,
    ), group in sorted(grouped.items()):
        throughputs = [result.queries_per_s for result in group]
        totals = [result.total_ns for result in group]
        deadline_misses = sum(result.deadline_misses for result in group)
        rows.append(
            {
                "schema_version": SCHEMA_VERSION,
                "feature": feature,
                "scenario": scenario or "",
                "backend": backend,
                "workload": workload,
                "queries": queries,
                "objects": "" if objects is None else objects,
                "density": "" if density is None else f"{density:.3f}",
                "repetitions": len(group),
                "collisions_median": median([result.collisions for result in group]),
                "errors_total": sum(result.errors for result in group),
                "unsupported": all(result.unsupported for result in group),
                "throughput_median": f"{median(throughputs):.3f}",
                "throughput_mean": f"{(sum(throughputs) / len(throughputs)):.3f}",
                "ns_per_query_median": f"{median([result.ns_per_query for result in group]):.3f}",
                "p50_ns_median": _sample_median(group, 50),
                "p90_ns_median": _sample_median(group, 90),
                "p95_ns_median": _sample_median(group, 95),
                "p99_ns_median": _sample_median(group, 99),
                "oracle": oracle,
                "shape": shape,
                "transform_kind": transform_kind,
                "scene_kind": scene_kind,
                "density_label": density_label,
                "total_ns_mean": f"{(sum(totals) / len(totals)):.3f}",
                "total_ns_min": min(totals),
                "total_ns_max": max(totals),
                "total_ns_stddev": f"{statistics.pstdev(totals):.3f}" if len(totals) > 1 else "0.000",
                "build_ns_median": median([result.build_ns for result in group]),
                "execution_layer": execution_layer,
                "operation": operation,
                "api_mode": api_mode,
                "batch_size": batch_size,
                "threads": threads,
                "sample_semantics": sample_semantics,
                "static_scene_objects": static_scene_objects,
                "dynamic_scene_objects": dynamic_scene_objects,
                "trajectory_steps": trajectory_steps,
                "time_window_steps": time_window_steps,
                "motion_kind": motion_kind,
                "shape_variation": shape_variation,
                "hit_class": hit_class,
                "shape_family": shape_family,
                "scene_mode": scene_mode,
                "ccd_mode": ccd_mode,
                "construction_ns_median": median([result.construction_ns for result in group]),
                "query_ns_median": median([result.query_ns for result in group]),
                "deadline_ns": deadline_ns,
                "deadline_misses": deadline_misses,
                "deadline_miss_rate": f"{deadline_misses / len(group):.6f}" if group else "0.000000",
                "cache_state": cache_state,
                "candidate_count": candidate_count,
            }
        )
    return rows


def compare_runs(results: list[RunResult], baseline_backend: str = "parry"):
    rows = []
    grouped: dict[tuple[Any, ...], list[RunResult]] = {}
    for result in results:
        if result.unsupported or result.errors:
            continue
        key = (
            result.feature,
            result.scenario,
            result.workload,
            result.queries,
            result.objects,
            result.density,
            result.shape,
            result.transform_kind,
            result.scene_kind,
            result.density_label,
            result.execution_layer,
            result.operation,
            result.api_mode,
            result.batch_size,
            result.threads,
            result.trajectory_steps,
            result.motion_kind,
            result.shape_variation,
            result.shape_family,
            result.scene_mode,
            result.ccd_mode,
            result.deadline_ns,
            result.cache_state,
            result.candidate_count,
        )
        grouped.setdefault(key, []).append(result)

    for key, group in sorted(grouped.items()):
        feature, scenario, workload, queries, objects, density = key[:6]
        by_backend: dict[str, dict[int, RunResult]] = {}
        for result in group:
            by_backend.setdefault(result.backend, {})[result.repetition] = result
        baseline_runs = by_backend.get(baseline_backend)
        if not baseline_runs:
            continue
        for backend, backend_runs in sorted(by_backend.items()):
            if backend == baseline_backend:
                continue
            paired_repetitions = sorted(set(baseline_runs) & set(backend_runs))
            speedups = []
            baseline_throughputs = []
            backend_throughputs = []
            for repetition in paired_repetitions:
                baseline = baseline_runs[repetition]
                candidate = backend_runs[repetition]
                if baseline.queries_per_s <= 0 or candidate.queries_per_s <= 0:
                    continue
                baseline_throughputs.append(baseline.queries_per_s)
                backend_throughputs.append(candidate.queries_per_s)
                speedups.append(candidate.queries_per_s / baseline.queries_per_s)
            if not speedups:
                continue
            low, high = bootstrap_ci(speedups)
            rows.append(
                {
                    "schema_version": SCHEMA_VERSION,
                    "feature": feature,
                    "scenario": scenario or "",
                    "workload": workload,
                    "queries": queries,
                    "objects": "" if objects is None else objects,
                    "density": "" if density is None else f"{density:.3f}",
                    "baseline_backend": baseline_backend,
                    "backend": backend,
                    "paired_repetitions": len(speedups),
                    "baseline_throughput_median": f"{median(baseline_throughputs):.3f}",
                    "backend_throughput_median": f"{median(backend_throughputs):.3f}",
                    "speedup_median": f"{median(speedups):.6f}",
                    "speedup_q25": f"{percentile_float(speedups, 25):.6f}",
                    "speedup_q75": f"{percentile_float(speedups, 75):.6f}",
                    "speedup_ci_low": f"{low:.6f}",
                    "speedup_ci_high": f"{high:.6f}",
                    "verdict": comparison_verdict(low, high) if len(speedups) >= 2 else "inconclusive",
                    "execution_layer": group[0].execution_layer,
                    "operation": group[0].operation,
                    "api_mode": group[0].api_mode,
                    "batch_size": group[0].batch_size,
                    "threads": group[0].threads,
                    "trajectory_steps": group[0].trajectory_steps,
                    "motion_kind": group[0].motion_kind,
                    "shape_variation": group[0].shape_variation,
                    "shape_family": group[0].shape_family,
                    "scene_mode": group[0].scene_mode,
                    "ccd_mode": group[0].ccd_mode,
                    "deadline_ns": group[0].deadline_ns,
                    "cache_state": group[0].cache_state,
                    "candidate_count": group[0].candidate_count,
                }
            )
    return rows


def compare_modes(results: list[RunResult]):
    grouped = {}
    for result in results:
        if result.unsupported or result.errors or result.api_mode == "":
            continue
        key = (
            result.feature,
            result.backend,
            result.execution_layer,
            result.batch_size,
            result.trajectory_steps,
            result.time_window_steps,
            result.shape_variation,
            result.static_scene_objects,
            result.dynamic_scene_objects,
            result.cache_state,
            result.candidate_count,
        )
        grouped.setdefault(key, []).append(result)

    rows = []
    for key, group in sorted(grouped.items()):
        baseline = {result.repetition: result for result in group if result.api_mode == "scalar"}
        candidates = {}
        for result in group:
            if result.api_mode != "scalar":
                candidates.setdefault((result.api_mode, result.threads), {})[result.repetition] = result
        for (candidate_mode, threads), candidate_runs in sorted(candidates.items()):
            ratios = []
            for repetition in sorted(set(baseline) & set(candidate_runs)):
                scalar = baseline[repetition]
                candidate = candidate_runs[repetition]
                if scalar.ns_per_query > 0 and candidate.ns_per_query > 0:
                    ratios.append(scalar.ns_per_query / candidate.ns_per_query)
            if not ratios:
                continue
            low, high = bootstrap_ci(ratios)
            (
                feature,
                backend,
                execution_layer,
                batch_size,
                trajectory_steps,
                time_window_steps,
                shape_variation,
                static_scene_objects,
                dynamic_scene_objects,
                cache_state,
                candidate_count,
            ) = key
            row = {
                "schema_version": SCHEMA_VERSION,
                "feature": feature,
                "backend": backend,
                "execution_layer": execution_layer,
                "baseline_mode": "scalar",
                "candidate_mode": candidate_mode,
                "batch_size": batch_size,
                "threads": threads,
                "trajectory_steps": trajectory_steps,
                "time_window_steps": time_window_steps,
                "shape_variation": shape_variation,
                "paired_repetitions": len(ratios),
                "ratio_median": f"{median(ratios):.6f}",
                "ratio_q25": f"{percentile_float(ratios, 25):.6f}",
                "ratio_q75": f"{percentile_float(ratios, 75):.6f}",
                "ratio_ci_low": f"{low:.6f}",
                "ratio_ci_high": f"{high:.6f}",
                "verdict": comparison_verdict(low, high) if len(ratios) >= 2 else "inconclusive",
            }
            if feature == "planning":
                row.update(
                    {
                        "static_scene_objects": static_scene_objects,
                        "dynamic_scene_objects": dynamic_scene_objects,
                        "cache_state": cache_state,
                        "candidate_count": candidate_count,
                    }
                )
            rows.append(row)
    return rows


def compare_layers(results: list[RunResult]):
    grouped = {}
    for result in results:
        if result.feature != "native_layers" or result.unsupported or result.errors:
            continue
        key = (
            result.feature,
            result.backend,
            result.workload,
            result.operation,
            result.trajectory_steps,
            result.shape_variation,
            result.queries,
        )
        grouped.setdefault(key, []).append(result)

    rows = []
    for key, group in sorted(grouped.items()):
        native = {result.repetition: result for result in group if result.execution_layer == "engine_native"}
        public = {
            result.repetition: result for result in group if result.execution_layer == "rust_public_convert_and_query"
        }
        python = {result.repetition: result for result in group if result.execution_layer == "python_end_to_end"}
        if set(native) != set(public) or set(public) != set(python):
            raise ValueError(f"unmatched execution-layer repetitions for {key}")
        public_ratios = []
        python_ratios = []
        native_costs = []
        public_costs = []
        python_costs = []
        for repetition in sorted(native):
            native_cost = native[repetition].ns_per_query
            public_cost = public[repetition].ns_per_query
            python_cost = python[repetition].ns_per_query
            if native_cost > 0 and public_cost > 0 and python_cost > 0:
                native_costs.append(native_cost)
                public_costs.append(public_cost)
                python_costs.append(python_cost)
                public_ratios.append(public_cost / native_cost)
                python_ratios.append(python_cost / public_cost)
        if not public_ratios:
            continue
        low, high = bootstrap_ci(public_ratios)
        python_low, python_high = bootstrap_ci(python_ratios)
        feature, backend, workload, operation, trajectory_steps, shape_variation, queries = key
        rows.append(
            {
                "schema_version": SCHEMA_VERSION,
                "feature": feature,
                "backend": backend,
                "workload": workload,
                "operation": operation,
                "trajectory_steps": trajectory_steps,
                "shape_variation": shape_variation,
                "queries": queries,
                "paired_repetitions": len(public_ratios),
                "native_ns_median": f"{median(native_costs):.6f}",
                "public_ns_median": f"{median(public_costs):.6f}",
                "public_native_ratio_median": f"{median(public_ratios):.6f}",
                "python_ns_median": f"{median(python_costs):.6f}",
                "python_public_ratio_median": f"{median(python_ratios):.6f}",
                "python_ratio_q25": f"{percentile_float(python_ratios, 25):.6f}",
                "python_ratio_q75": f"{percentile_float(python_ratios, 75):.6f}",
                "python_ratio_ci_low": f"{python_low:.6f}",
                "python_ratio_ci_high": f"{python_high:.6f}",
                "ratio_q25": f"{percentile_float(public_ratios, 25):.6f}",
                "ratio_q75": f"{percentile_float(public_ratios, 75):.6f}",
                "ratio_ci_low": f"{low:.6f}",
                "ratio_ci_high": f"{high:.6f}",
                "verdict": "public_overhead" if low > 1.0 else "public_lower_cost" if high < 1.0 else "inconclusive",
            }
        )
    return rows


def percentile(values: list[int], percentile_value: int):
    if not values:
        return 0
    sorted_values = sorted(values)
    index = round((len(sorted_values) - 1) * percentile_value / 100)
    return sorted_values[index]


def _sample_percentiles(samples_ns: list[int]) -> dict[str, int | str]:
    if not samples_ns:
        return {
            "min_ns": "",
            "p50_ns": "",
            "p90_ns": "",
            "p95_ns": "",
            "p99_ns": "",
            "max_ns": "",
        }
    return {
        "min_ns": percentile(samples_ns, 0),
        "p50_ns": percentile(samples_ns, 50),
        "p90_ns": percentile(samples_ns, 90),
        "p95_ns": percentile(samples_ns, 95),
        "p99_ns": percentile(samples_ns, 99),
        "max_ns": percentile(samples_ns, 100),
    }


def _sample_median(results: list[RunResult], percentile_value: int) -> int | float | str:
    values = [percentile(result.samples_ns, percentile_value) for result in results if result.samples_ns]
    return median(values) if values else ""


def percentile_float(values: list[float], percentile_value: int):
    if not values:
        return 0.0
    sorted_values = sorted(values)
    index = round((len(sorted_values) - 1) * percentile_value / 100)
    return float(sorted_values[index])


def median(values):
    if not values:
        return 0
    sorted_values = sorted(values)
    middle = len(sorted_values) // 2
    if len(sorted_values) % 2:
        return sorted_values[middle]
    return (sorted_values[middle - 1] + sorted_values[middle]) / 2


def bootstrap_ci(values: list[float], confidence: float = 0.95, samples: int = 1_000):
    if len(values) < 2:
        value = float(values[0]) if values else 0.0
        return value, value
    # deterministic bootstrap is enough for benchmark reporting; no scipy dependency.
    rng = random.Random(20_260_621)
    medians = []
    for _ in range(samples):
        medians.append(median([values[rng.randrange(len(values))] for _ in values]))
    tail = (1.0 - confidence) / 2.0
    return percentile_float(medians, round(tail * 100)), percentile_float(medians, round((1.0 - tail) * 100))


def comparison_verdict(ci_low: float, ci_high: float):
    if ci_low > 1.0:
        return "faster"
    if ci_high < 1.0:
        return "slower"
    return "inconclusive"


def counts():
    return {"tp": 0, "tn": 0, "fp": 0, "fn": 0}


def update_counts(counter: dict[str, int], expected: bool | None, actual: bool):
    if expected is None:
        return
    if expected and actual:
        counter["tp"] += 1
    elif not expected and not actual:
        counter["tn"] += 1
    elif not expected and actual:
        counter["fp"] += 1
    else:
        counter["fn"] += 1
