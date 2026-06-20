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
]


def run_row(result: RunResult):
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
        "min_ns": percentile(result.samples_ns, 0),
        "p50_ns": percentile(result.samples_ns, 50),
        "p90_ns": percentile(result.samples_ns, 90),
        "p95_ns": percentile(result.samples_ns, 95),
        "p99_ns": percentile(result.samples_ns, 99),
        "max_ns": percentile(result.samples_ns, 100),
        "oracle": result.oracle,
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
        "oracle": result.oracle,
    }


def summarize_runs(results: list[RunResult]):
    rows = []
    grouped: dict[tuple[Any, ...], list[RunResult]] = {}
    for result in results:
        key = (result.feature, result.scenario, result.backend, result.workload, result.objects, result.density, result.oracle)
        grouped.setdefault(key, []).append(result)

    for (feature, scenario, backend, workload, objects, density, oracle), group in sorted(grouped.items()):
        throughputs = [result.queries_per_s for result in group]
        rows.append(
            {
                "schema_version": SCHEMA_VERSION,
                "feature": feature,
                "scenario": scenario or "",
                "backend": backend,
                "workload": workload,
                "queries": group[0].queries,
                "objects": "" if objects is None else objects,
                "density": "" if density is None else f"{density:.3f}",
                "repetitions": len(group),
                "collisions_median": median([result.collisions for result in group]),
                "errors_total": sum(result.errors for result in group),
                "unsupported": all(result.unsupported for result in group),
                "throughput_median": f"{median(throughputs):.3f}",
                "throughput_mean": f"{(sum(throughputs) / len(throughputs)):.3f}",
                "ns_per_query_median": f"{median([result.ns_per_query for result in group]):.3f}",
                "p50_ns_median": median([percentile(result.samples_ns, 50) for result in group]),
                "p90_ns_median": median([percentile(result.samples_ns, 90) for result in group]),
                "p95_ns_median": median([percentile(result.samples_ns, 95) for result in group]),
                "p99_ns_median": median([percentile(result.samples_ns, 99) for result in group]),
                "oracle": oracle,
            }
        )
    return rows


def percentile(values: list[int], percentile_value: int):
    if not values:
        return 0
    sorted_values = sorted(values)
    index = round((len(sorted_values) - 1) * percentile_value / 100)
    return sorted_values[index]


def median(values):
    if not values:
        return 0
    sorted_values = sorted(values)
    middle = len(sorted_values) // 2
    if len(sorted_values) % 2:
        return sorted_values[middle]
    return (sorted_values[middle - 1] + sorted_values[middle]) / 2


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
