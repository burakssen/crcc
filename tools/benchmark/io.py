import csv
import json
import platform
import subprocess
import sys
from dataclasses import dataclass
from importlib import metadata
from pathlib import Path
from typing import Any

from .config import SCHEMA_VERSION, BenchmarkConfig
from .results import (
    COMPARISON_FIELDS,
    CORRECTNESS_FIELDS,
    LAYER_COMPARISON_FIELDS,
    MEMORY_FIELDS,
    MODE_COMPARISON_FIELDS,
    PARALLEL_SCALING_FIELDS,
    RUN_FIELDS,
    SUMMARY_FIELDS,
    compare_layers,
    compare_modes,
    compare_runs,
    correctness_row,
    median,
    memory_row,
    run_row,
    summarize_runs,
)

ARTIFACT_FIELDS = {
    "runs.csv": RUN_FIELDS,
    "summary.csv": SUMMARY_FIELDS,
    "comparisons.csv": COMPARISON_FIELDS,
    "correctness.csv": CORRECTNESS_FIELDS,
    "parallel_scaling.csv": PARALLEL_SCALING_FIELDS,
    "memory.csv": MEMORY_FIELDS,
    "mode_comparisons.csv": MODE_COMPARISON_FIELDS,
    "layer_comparisons.csv": LAYER_COMPARISON_FIELDS,
}


class ArtifactError(ValueError):
    pass


@dataclass(frozen=True)
class ArtifactBundle:
    metadata: dict[str, Any]
    rows: dict[str, list[dict[str, str]]]
    source: str

    def get(self, filename: str):
        return self.rows[filename]


def write_artifacts(config: BenchmarkConfig, runs, correctness, parallel_rows, memory_rows):
    config.output_dir.mkdir(parents=True, exist_ok=True)
    summary_rows, comparison_rows, correctness_rows = write_result_csvs(
        config.output_dir, runs, correctness, parallel_rows, memory_rows
    )
    write_metadata(config.output_dir / "metadata.json", config)
    write_report(config.output_dir / "benchmark_report.md", config, summary_rows, comparison_rows, correctness_rows)


def write_suite_artifacts(config: BenchmarkConfig, suite: str, runs, correctness, parallel_rows, memory_rows):
    suite_dir = config.output_dir / "suites" / suite
    suite_dir.mkdir(parents=True, exist_ok=True)
    write_result_csvs(suite_dir, runs, correctness, parallel_rows, memory_rows)
    write_metadata(suite_dir / "metadata.json", config, suite=suite)


def write_result_csvs(output_dir: Path, runs, correctness, parallel_rows, memory_rows):
    output_dir.mkdir(parents=True, exist_ok=True)
    write_dicts(output_dir / "runs.csv", RUN_FIELDS, [run_row(result) for result in runs])
    summary_rows = summarize_runs(runs)
    comparison_rows = compare_runs(runs)
    mode_comparison_rows = compare_modes(runs)
    layer_comparison_rows = compare_layers(runs)
    correctness_rows = [correctness_row(result) for result in correctness]
    write_dicts(output_dir / "summary.csv", SUMMARY_FIELDS, summary_rows)
    write_dicts(output_dir / "comparisons.csv", COMPARISON_FIELDS, comparison_rows)
    write_dicts(output_dir / "mode_comparisons.csv", MODE_COMPARISON_FIELDS, mode_comparison_rows)
    write_dicts(output_dir / "layer_comparisons.csv", LAYER_COMPARISON_FIELDS, layer_comparison_rows)
    write_dicts(output_dir / "correctness.csv", CORRECTNESS_FIELDS, correctness_rows)
    write_dicts(output_dir / "parallel_scaling.csv", PARALLEL_SCALING_FIELDS, parallel_rows)
    write_dicts(output_dir / "memory.csv", MEMORY_FIELDS, [memory_row(result) for result in memory_rows])
    return summary_rows, comparison_rows, correctness_rows


def write_dicts(path: Path, fields: list[str], rows: list[dict[str, Any]]):
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def read_dicts(path: Path, required_fields: list[str] | None = None):
    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        if required_fields is not None:
            missing = [field for field in required_fields if field not in (reader.fieldnames or [])]
            if missing:
                raise ArtifactError(f"{path} is missing required column(s): {', '.join(missing)}")
        return list(reader)


def load_artifacts(output_dir: Path) -> ArtifactBundle:
    output_dir = Path(output_dir)
    if not output_dir.is_dir():
        raise ArtifactError(
            f"No benchmark artifacts found at {output_dir}. "
            "Run `uv run main.py study` first or specify --benchmark-output."
        )

    aggregate_present = [filename for filename in ARTIFACT_FIELDS if (output_dir / filename).exists()]
    if aggregate_present:
        missing = [filename for filename in ARTIFACT_FIELDS if filename not in aggregate_present]
        if missing:
            raise ArtifactError(f"Incomplete aggregate artifacts in {output_dir}; missing: {', '.join(missing)}")
        metadata = _read_metadata(output_dir / "metadata.json")
        rows = {
            filename: _read_validated_csv(output_dir / filename, fields) for filename, fields in ARTIFACT_FIELDS.items()
        }
        source = "aggregate"
    else:
        suite_dirs = sorted(path for path in (output_dir / "suites").glob("*") if path.is_dir())
        if not suite_dirs:
            raise ArtifactError(
                f"No benchmark artifacts found at {output_dir}. "
                "Run `uv run main.py study` first or specify --benchmark-output."
            )
        rows = {filename: [] for filename in ARTIFACT_FIELDS}
        metadata_items = []
        for suite_dir in suite_dirs:
            missing = [filename for filename in ARTIFACT_FIELDS if not (suite_dir / filename).exists()]
            if missing:
                raise ArtifactError(f"Incomplete suite artifacts in {suite_dir}; missing: {', '.join(missing)}")
            metadata_items.append(_read_metadata(suite_dir / "metadata.json"))
            for filename, fields in ARTIFACT_FIELDS.items():
                rows[filename].extend(_read_validated_csv(suite_dir / filename, fields))
        metadata = _merge_suite_metadata(metadata_items)
        source = "suites"

    if not rows["summary.csv"]:
        raise ArtifactError(f"Benchmark artifacts at {output_dir} contain no summary rows")
    return ArtifactBundle(metadata=metadata, rows=rows, source=source)


def _read_metadata(path: Path):
    if not path.exists():
        raise ArtifactError(f"Missing benchmark metadata: {path}")
    try:
        payload = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ArtifactError(f"Invalid benchmark metadata {path}: {error}") from error
    schema = str(payload.get("schema_version", ""))
    if schema != SCHEMA_VERSION:
        raise ArtifactError(
            f"Unsupported benchmark schema in {path}: expected {SCHEMA_VERSION}, found {schema or 'none'}"
        )
    return payload


def _read_validated_csv(path: Path, fields: list[str]):
    try:
        rows = read_dicts(path, fields)
    except (OSError, csv.Error) as error:
        raise ArtifactError(f"Invalid benchmark CSV {path}: {error}") from error
    for line, row in enumerate(rows, start=2):
        schema = str(row.get("schema_version", ""))
        if schema != SCHEMA_VERSION:
            raise ArtifactError(
                f"Unsupported benchmark schema in {path}:{line}: expected {SCHEMA_VERSION}, found {schema or 'none'}"
            )
    return rows


def _merge_suite_metadata(items):
    schemas = {str(item.get("schema_version", "")) for item in items}
    if schemas != {SCHEMA_VERSION}:
        raise ArtifactError(f"Suite metadata uses incompatible schemas: {', '.join(sorted(schemas))}")
    metadata = dict(items[0])
    command = dict(metadata.get("command", {}))
    command["suites"] = [suite for item in items for suite in item.get("command", {}).get("suites", [])]
    metadata["command"] = command
    return metadata


def write_report_from_artifacts(output_dir: Path, artifacts: ArtifactBundle | None = None):
    output_dir = Path(output_dir)
    artifacts = artifacts or load_artifacts(output_dir)
    write_report_rows(output_dir / "benchmark_report.md", artifacts)


def read_aggregate_or_suite_rows(output_dir: Path, filename: str):
    aggregate = output_dir / filename
    if aggregate.exists():
        return read_dicts(aggregate)
    rows = []
    for path in sorted((output_dir / "suites").glob(f"*/{filename}")):
        rows.extend(read_dicts(path))
    return rows


def write_report(path: Path, config: BenchmarkConfig, summary_rows, comparison_rows, correctness_rows):
    metadata = {
        "command": {
            "sample_count": config.sample_count,
            "repetitions": config.repetitions,
            "seed": config.seed,
            "thread_counts": list(config.thread_counts),
            "engines": list(config.engines),
            "scenarios": [str(path) for path in config.scenario_paths],
            "profile": config.profile,
            "suites": list(config.suites),
            "include_stress": config.include_stress,
        }
    }
    rows = {filename: [] for filename in ARTIFACT_FIELDS}
    rows["summary.csv"] = summary_rows
    rows["comparisons.csv"] = comparison_rows
    rows["correctness.csv"] = correctness_rows
    write_report_rows(path, ArtifactBundle(metadata, rows, "memory"))


def write_report_rows(path: Path, artifacts: ArtifactBundle):
    metadata = artifacts.metadata
    summary_rows = artifacts.get("summary.csv")
    comparison_rows = artifacts.get("comparisons.csv")
    correctness_rows = artifacts.get("correctness.csv")
    parallel_rows = artifacts.get("parallel_scaling.csv")
    layer_rows = artifacts.get("layer_comparisons.csv")
    memory_rows = artifacts.get("memory.csv")
    command = metadata.get("command", {})
    environment = metadata.get("environment", {})
    mismatches = sum(_int(row["mismatches"]) for row in correctness_rows)
    false_positives = sum(_int(row["false_positive"]) for row in correctness_rows)
    false_negatives = sum(_int(row["false_negative"]) for row in correctness_rows)
    errors = sum(_int(row["errors_total"]) for row in summary_rows)
    unsupported = [row for row in summary_rows if str(row["unsupported"]).lower() == "true"]
    comparisons = sorted(
        comparison_rows,
        key=lambda row: (row["feature"], row["workload"], row["scenario"], row["backend"]),
    )[:30]
    parallel_summary = _best_parallel_rows(parallel_rows)
    lines = [
        "# CRCC Benchmark Report",
        "",
        "## Executive Summary",
        "",
        f"- Validated artifact source: **{artifacts.source}**",
        f"- Correctness mismatches: **{mismatches}** (FP: {false_positives}, FN: {false_negatives})",
        f"- Query errors: **{errors}**",
        f"- Unsupported backend/workload groups: **{len(unsupported)}**",
        "",
        "## Method",
        "",
        f"- Schema: {metadata.get('schema_version', SCHEMA_VERSION)}",
        f"- Samples per workload: {_int(command.get('sample_count')):,}",
        f"- Repetitions: {_int(command.get('repetitions'))}",
        f"- Engines: {', '.join(command.get('engines', []))}",
        f"- Scenarios: {', '.join(command.get('scenarios', []))}",
        f"- Thread counts: {', '.join(str(item) for item in command.get('thread_counts', []))}",
        f"- Profile: {command.get('profile', 'smoke')}",
        f"- Suites: {', '.join(command.get('suites', []))}",
        f"- Extended stress sizes: {bool(command.get('include_stress', False))}",
        "",
        "The study uses paired backend comparisons, deterministic workload seeds, analytic correctness oracles where possible, and bootstrap confidence intervals for speedup medians.",
        "Correctness mismatches count false negatives and query errors. Conservative false positives are reported separately in the `fp` column.",
        "",
        "## Spec Coverage Notes",
        "",
        "- CRCC is a 2D collision library, so sphere/box/capsule/cylinder/mesh/BVH/contact-count benchmarks are reported as unsupported.",
        "- `update_proxy` is a pose-query cost only; `rebuild_update` is the supported immutable-checker reconstruction cost after transformed objects.",
        "- Memory rows use fresh subprocesses and report the current-RSS increase from baseline through checker construction; allocator granularity and retained pages still limit precision.",
        "- Static scene scaling measures CRCC's merged static compound, not an independently mutable broad-phase world.",
        "- 2D equivalents are circle, rectangle, convex polygon, and compound shapes.",
        "- Exact tangency follows each backend's native contact semantics: Rhusics GJK excludes touching-only contact, while Parry and Collide include it.",
        "- API amortization uses one native worker to isolate call and collection overhead from parallel speedup.",
        "",
        "## Environment And Provenance",
        "",
        "| property | value |",
        "|---|---|",
        f"| Git revision | {_md(metadata.get('git', {}).get('revision'))} |",
        f"| Python | {_md(environment.get('python'))} |",
        f"| Platform | {_md(environment.get('platform'))} |",
        f"| Processor | {_md(environment.get('processor'))} |",
        f"| Rust compiler | {_md(environment.get('rustc'))} |",
        f"| commonroad-io | {_md(environment.get('commonroad_io'))} |",
        f"| NumPy | {_md(environment.get('numpy'))} |",
        "",
        "## Correctness",
        "",
        "| feature | workload | scenario | backend | queries | FP | FN | mismatches | oracle |",
        "|---|---|---|---|---:|---:|---:|---:|---|",
    ]
    for row in sorted(correctness_rows, key=lambda item: (item["feature"], item["workload"], item["backend"])):
        lines.append(
            f"| {_md(row['feature'])} | {_md(row['workload'])} | {_md(row['scenario'])} | {_md(row['backend'])} | "
            f"{_int(row['queries'])} | {_int(row['false_positive'])} | {_int(row['false_negative'])} | "
            f"{_int(row['mismatches'])} | {_md(row['oracle'])} |"
        )
    lines.extend(
        [
            "",
            "## Paired Backend Comparisons",
            "",
            "| feature | workload | scenario | queries | objects | density | baseline | backend | pairs | speedup | 95% CI | verdict |",
            "|---|---|---|---:|---:|---:|---|---|---:|---:|---|---|",
        ]
    )
    for row in comparisons:
        lines.append(
            f"| {_md(row['feature'])} | {_md(row['workload'])} | {_md(row['scenario'])} | {_int(row['queries'])} | {_md(row['objects'])} | "
            f"{_md(row['density'])} | {_md(row['baseline_backend'])} | {_md(row['backend'])} | "
            f"{_int(row['paired_repetitions'])} | "
            f"{_float(row['speedup_median']):.3f} | "
            f"[{_float(row['speedup_ci_low']):.3f}, {_float(row['speedup_ci_high']):.3f}] | {row['verdict']} |"
        )
    lines.extend(
        [
            "",
            "## Parallel Scaling",
            "",
            "| scenario | backend | threads | speedup | efficiency |",
            "|---|---|---:|---:|---:|",
        ]
    )
    for row in parallel_summary:
        lines.append(
            f"| {_md(row['scenario'])} | {_md(row['backend'])} | {_int(row['threads'])} | "
            f"{_float(row['speedup']):.3f} | {_float(row['efficiency']):.3f} |"
        )
    lines.extend(
        [
            "",
            "## Execution Layer Overhead",
            "",
            "| backend | workload | queries | repetitions | native ns/query | Rust public/native | Python/Rust public |",
            "|---|---|---:|---:|---:|---:|---:|",
        ]
    )
    for row in sorted(layer_rows, key=lambda item: (item["backend"], item["workload"])):
        lines.append(
            f"| {_md(row['backend'])} | {_md(row['workload'])} | {_int(row['queries']):,} | {_int(row['paired_repetitions'])} | "
            f"{_float(row['native_ns_median']):.3f} | {_float(row['public_native_ratio_median']):.3f} | "
            f"{_float(row['python_public_ratio_median']):.3f} |"
        )
    lines.extend(
        [
            "",
            "## Memory",
            "",
            "| feature | workload | backend | objects | peak bytes | measurement |",
            "|---|---|---|---:|---:|---|",
        ]
    )
    for row in sorted(memory_rows, key=lambda item: (item["feature"], item["workload"], item["backend"]))[:30]:
        lines.append(
            f"| {_md(row['feature'])} | {_md(row['workload'])} | {_md(row['backend'])} | {_md(row['objects'])} | "
            f"{_int(row['peak_bytes']):,} | {_md(row['measurement'])} |"
        )
    lines.extend(
        [
            "",
            "## Plots",
            "",
            "The PNG figures below link to PDF equivalents in the same directory. CSV files remain the source for exact values.",
            "",
        ]
    )
    from .plots import PLOT_NAMES

    for name in sorted(PLOT_NAMES):
        title = name.replace("_", " ").title()
        lines.extend([f"### {title}", "", f"[![{title}](plots/{name}.png)](plots/{name}.pdf)", ""])
        lines.extend(
            _plot_explanation(name, summary_rows, comparison_rows, correctness_rows, parallel_rows, memory_rows)
        )
    lines.extend(
        [
            "## Threats To Validity",
            "",
            "- The smoke profile is intended for pipeline validation, not publication claims. Use the spec profile for research results.",
            "- Measurements share a process and can be affected by warm-up, thermal state, allocator behavior, and execution order.",
            "- Confidence intervals describe paired repetitions in this harness; they do not establish general hardware-independent performance.",
            "- Conservative CCD backends may report false positives by design; false negatives and query errors are correctness failures.",
            "- Python heap and process peak RSS measurements are not exact native allocation deltas.",
            "",
        ]
    )
    path.write_text("\n".join(lines))


def _best_parallel_rows(rows):
    grouped = {}
    for row in rows:
        key = (row["scenario"], row["backend"], row["threads"])
        grouped.setdefault(key, []).append(row)
    result = []
    for group in grouped.values():
        result.append(max(group, key=lambda row: _float(row["speedup"])))
    return sorted(result, key=lambda row: (row["scenario"], row["backend"], _int(row["threads"])))


PLOT_GUIDANCE = {
    "backend_throughput_iqr": (
        "Compare median query throughput across backends and synthetic workload families.",
        "Each workload is a bar group; color identifies the backend. The logarithmic y-axis reports queries per second, so higher bars are better.",
        "This aggregate mixes operations with different geometric complexity; compare backends within a workload, not bar heights across unrelated workloads.",
    ),
    "backend_speedup_forest": (
        "Show paired backend speedups relative to the Parry baseline with uncertainty.",
        "Points are median speedup ratios and whiskers are 95% bootstrap intervals. Values right of 1 favor the candidate backend; intervals crossing 1 are inconclusive.",
        "Bootstrap intervals describe only the recorded paired repetitions and are not hardware-independent confidence bounds.",
    ),
    "latency_percentiles": (
        "Compare tail-latency amplification across synthetic workloads.",
        "Bars report p99 divided by p50 latency on a logarithmic axis. A value near 1 is stable; larger values indicate a heavier latency tail.",
        "Per-query samples and batch-average samples are not equivalent; API overhead rows are excluded from this synthetic summary.",
    ),
    "scene_scaling_curves": (
        "Measure how throughput changes with environment object count across scene modes and shape families.",
        "Rows represent static/static, moving/static, and moving/moving scenes; columns represent shape families. Both axes are logarithmic, and higher throughput is better.",
        "CRCC builds one immutable static compound, so these curves do not represent a mutable broad-phase world.",
    ),
    "parallel_scaling_summary": (
        "Summarize throughput speedup as worker count increases.",
        "Lines show median speedup over scenarios with IQR bands; the dashed diagonal is ideal linear scaling. Higher is better.",
        "Superlinear points can arise from cache, scheduling, and measurement effects and should not be interpreted as guaranteed scaling.",
    ),
    "parallel_efficiency_summary": (
        "Show how effectively each backend converts additional workers into speedup.",
        "Efficiency is speedup divided by thread count. The dashed line at 1 is ideal; declining curves indicate diminishing returns.",
        "Efficiency aggregates heterogeneous scenarios and can hide scenario-specific contention or workload-size effects.",
    ),
    "commonroad_scenario_summary": (
        "Compare parallel versus sequential throughput on each CommonRoad scenario.",
        "Bars show the parallel/sequential throughput ratio by scenario and backend. Values above 1 indicate a parallel gain.",
        "Scenario geometry and query distributions differ, so ratios should not be treated as intrinsic backend constants.",
    ),
    "correctness_mismatch_matrix": (
        "Surface backend/workload groups with observed correctness mismatches.",
        "A status panel means no mismatches; otherwise horizontal bars count mismatches for each affected group.",
        "Exact tangency follows documented backend-native semantics, and conservative CCD false positives are reported separately from failures.",
    ),
    "update_time_scaling": (
        "Compare pose-query proxy cost across transform kinds and query counts.",
        "Each panel is one transform kind. Curves report median nanoseconds per query on shared logarithmic axes; lower is better.",
        "This benchmark varies query poses and does not mutate or incrementally update the static scene.",
    ),
    "density_scaling_curves": (
        "Show throughput scaling separately for sparse, medium, dense, and worst-case collision rates.",
        "Each panel fixes density while object count increases. Curves share logarithmic axes; higher throughput is better.",
        "Worst-case queries collide immediately and can therefore be faster than sparse misses that traverse more geometry.",
    ),
    "shape_complexity_throughput": (
        "Compare backend throughput as polygon vertex count or compound child count increases.",
        "Grouped bars use backend colors and a logarithmic throughput axis. Higher bars are better within the same shape workload.",
        "The selected shapes are representative synthetic cases, not a complete characterization of arbitrary geometry.",
    ),
    "memory_growth": (
        "Estimate incremental checker memory and per-object memory cost in isolated processes.",
        "The left panel shows median RSS delta in MiB; the right divides that delta by object count. Bands show the IQR across fresh-process repetitions.",
        "RSS is page-granular and includes allocator effects, Python wrappers, input objects, and checker construction; it is not an exact Rust allocation count.",
    ),
    "api_batch_amortization": (
        "Show how scalar, global-pool batch, and fresh-pool batch cost per query changes with batch size for each backend.",
        "Each panel fixes a backend. Dashed lines are repeated scalar calls, solid lines use the global Rayon pool, and dotted lines create a one-thread pool per call; lower is better.",
        "The global batch path changes dispatch behavior at the 32-query threshold, while the fresh-pool path includes pool construction. These are end-to-end API costs, not kernel-only collision times.",
    ),
    "api_batch_speedup": (
        "Summarize the relative cost of scalar and batch APIs by batch size.",
        "The ratio is scalar ns/query divided by global-pool batch ns/query. Values above 1 favor batching; values below 1 mean scalar calls remain cheaper.",
        "The global pool can execute in parallel at and above the dispatch threshold, so the ratio combines Python-call amortization with scheduling and parallel-execution effects.",
    ),
    "dynamic_batch_amortization": (
        "Compare scalar and batched dynamic-obstacle query cost as batch size and trajectory length increase.",
        "Panels fix trajectory length; dashed curves are scalar calls and solid curves are `collides_dynamic_batch` batches. Lower ns/query is better.",
        "The synthetic trajectories use circles, translation, and a 50% hit mix; other shapes and hit positions may scale differently.",
    ),
    "dynamic_time_window_scaling": (
        "Isolate how the requested dynamic-query time range affects scalar and batched query cost.",
        "The x-axis is the number of trajectory steps searched; dashed curves are scalar calls and solid curves are batches. Lower is better.",
        "All rows use 16-step source trajectories and vary only the inclusive query window beginning at step zero.",
    ),
    "time_variant_query_scaling": (
        "Measure query cost for dynamic obstacles whose shape changes over time.",
        "Panels represent shape-variation classes; x is trajectory steps and y is median ns/query. Lower curves are better.",
        "Construction cost is recorded separately in CSV fields and the plot focuses on warmed query execution.",
    ),
    "execution_layer_cost": (
        "Quantify Rust public conversion overhead and Python binding overhead on identical workloads.",
        "Solid bars show Rust public/native cost; hatched bars show Python/Rust-public cost. Values above 1 indicate overhead.",
        "All three layers reuse deterministic inputs with construction outside timed regions.",
    ),
}


def _plot_explanation(name, summary, comparisons, correctness, parallel, memory):
    purpose, reading, limitation = PLOT_GUIDANCE[name]
    observations = _plot_observations(name, summary, comparisons, correctness, parallel, memory)
    return [
        "**Purpose**",
        "",
        purpose,
        "",
        "**How to Read**",
        "",
        reading,
        "",
        "**Observed Results**",
        "",
        *[f"- {observation}" for observation in observations],
        "",
        "**Interpretation**",
        "",
        _plot_interpretation(name, observations),
        "",
        "**Limitations**",
        "",
        limitation,
        "",
    ]


def _plot_observations(name, summary, comparisons, correctness, parallel, memory):
    if name == "correctness_mismatch_matrix":
        mismatches = sum(_int(row["mismatches"]) for row in correctness)
        queries = sum(_int(row["queries"]) for row in correctness)
        return [f"{mismatches:,} mismatches were recorded across {queries:,} checked queries."]
    if name == "backend_speedup_forest":
        verdicts = {
            verdict: sum(row["verdict"] == verdict for row in comparisons)
            for verdict in ("faster", "slower", "inconclusive")
        }
        return [
            f"Paired comparisons classify {verdicts['faster']} as faster, {verdicts['slower']} as slower, and {verdicts['inconclusive']} as inconclusive."
        ]
    if name == "latency_percentiles":
        candidates = [
            row
            for row in summary
            if row["feature"] not in {"api_overhead", "scenario", "scene_scaling"} and _float(row["p50_ns_median"]) > 0
        ]
        if not candidates:
            return ["No latency-percentile rows are available."]
        row = max(candidates, key=lambda item: _float(item["p99_ns_median"]) / _float(item["p50_ns_median"]))
        ratio = _float(row["p99_ns_median"]) / _float(row["p50_ns_median"])
        return [
            f"The largest observed p99/p50 ratio is {ratio:.2f} for {_md(row['backend'])} on {_md(row['feature'])}:{_md(row['workload'])}."
        ]
    if name in {"parallel_scaling_summary", "parallel_efficiency_summary"}:
        metric = "speedup" if name == "parallel_scaling_summary" else "efficiency"
        observations = []
        for backend in sorted({row["backend"] for row in parallel}):
            backend_rows = [row for row in parallel if row["backend"] == backend]
            best = max(backend_rows, key=lambda row: _float(row[metric]))
            observations.append(
                f"{backend.title()} reaches a maximum recorded {metric} of {_float(best[metric]):.2f} at {_int(best['threads'])} threads."
            )
        return observations or ["No parallel-scaling rows are available."]
    if name == "commonroad_scenario_summary":
        return _scenario_parallel_observations(summary)
    if name in {"api_batch_amortization", "api_batch_speedup"}:
        return _api_observations(summary)
    if name == "dynamic_batch_amortization":
        return _mode_ratio_observations(summary, "dynamic_batch")
    if name == "dynamic_time_window_scaling":
        rows = [row for row in summary if row.get("scene_kind") == "dynamic_time_window"]
        return _range_observations(rows, "time_window_steps")
    if name == "time_variant_query_scaling":
        return _time_variant_observations(summary)
    if name == "execution_layer_cost":
        return _layer_observations(summary)
    if name == "memory_growth":
        return _memory_observations(memory)
    if name == "update_time_scaling":
        return _group_winner_observations(summary, "update_proxy", "transform_kind", "ns_per_query_median", lower=True)
    if name == "density_scaling_curves":
        return _group_winner_observations(summary, "density_scaling", "density_label", "throughput_median")
    if name == "shape_complexity_throughput":
        return _winner_count_observation([row for row in summary if row["feature"] == "shape_complexity"])
    if name == "scene_scaling_curves":
        largest = max((_int(row["objects"]) for row in summary if row["feature"] == "scene_scaling"), default=0)
        return _winner_count_observation(
            [row for row in summary if row["feature"] == "scene_scaling" and _int(row["objects"]) == largest],
            suffix=f" at {largest} objects",
            include_dimensions=True,
        )
    return _winner_count_observation(
        [row for row in summary if row["feature"] not in {"api_overhead", "scenario", "scene_scaling"}]
    )


def _winner_count_observation(rows, suffix="", include_dimensions=False):
    grouped = {}
    for row in rows:
        key = (row["feature"], row["workload"])
        if include_dimensions:
            key += (row.get("objects", ""), row.get("density", ""))
        grouped.setdefault(key, []).append(row)
    wins = {}
    for group in grouped.values():
        winner = max(group, key=lambda row: _float(row["throughput_median"]))["backend"]
        wins[winner] = wins.get(winner, 0) + 1
    if not wins:
        return ["No plottable rows are available."]
    return [
        f"{backend.title()} has the highest median throughput in {count} plotted workload groups{suffix}."
        for backend, count in sorted(wins.items())
    ]


def _group_winner_observations(summary, feature, group_field, metric, lower=False):
    rows = [row for row in summary if row["feature"] == feature]
    largest = max((_int(row["objects"]) for row in rows), default=0)
    observations = []
    for group in sorted({row[group_field] for row in rows}):
        candidates = [row for row in rows if row[group_field] == group and _int(row["objects"]) == largest]
        winner = (min if lower else max)(candidates, key=lambda row: _float(row[metric]))
        observations.append(
            f"At {largest} objects for {group.replace('_', ' ')}, {winner['backend'].title()} records {_float(winner[metric]):,.1f} {('ns/query' if lower else 'queries/s')}."
        )
    return observations or ["No plottable rows are available."]


def _scenario_parallel_observations(summary):
    rows = [row for row in summary if row["feature"] == "scenario"]
    ratios = []
    for scenario in {row["scenario"] for row in rows}:
        for backend in {row["backend"] for row in rows}:
            sequential = next(
                (
                    row
                    for row in rows
                    if row["scenario"] == scenario
                    and row["backend"] == backend
                    and row["workload"] == "static_sequential"
                ),
                None,
            )
            parallel = next(
                (
                    row
                    for row in rows
                    if row["scenario"] == scenario
                    and row["backend"] == backend
                    and row["workload"] == "static_parallel"
                ),
                None,
            )
            if sequential and parallel:
                ratios.append(
                    (_float(parallel["throughput_median"]) / _float(sequential["throughput_median"]), backend, scenario)
                )
    if not ratios:
        return ["No paired scenario rows are available."]
    best = max(ratios)
    return [
        f"The largest parallel/sequential throughput ratio is {best[0]:.2f} for {best[1].title()} on {_md(best[2])}."
    ]


def _api_observations(summary):
    rows = [row for row in summary if row["feature"] == "api_overhead"]
    if not rows:
        return ["No API-overhead rows are available."]
    observations = []
    for backend in sorted({row["backend"] for row in rows}):
        ratios = {}
        for batch_size in (31, 32, 1_024):
            scalar = next(
                (
                    row
                    for row in rows
                    if row["backend"] == backend
                    and row["workload"] == "python_scalar"
                    and _int(row["queries"]) == batch_size
                ),
                None,
            )
            batch = next(
                (
                    row
                    for row in rows
                    if row["backend"] == backend
                    and row["workload"] == "python_batch"
                    and _int(row["queries"]) == batch_size
                ),
                None,
            )
            if scalar and batch:
                ratios[batch_size] = _float(scalar["ns_per_query_median"]) / _float(batch["ns_per_query_median"])
        if ratios:
            observations.append(
                f"{backend.title()} scalar/global-batch ratios are "
                + ", ".join(f"{ratio:.2f} at {size:,}" for size, ratio in ratios.items())
                + "; the sharp change at 32 marks the parallel-dispatch threshold."
            )
    return observations


def _memory_observations(memory):
    rows = [row for row in memory if row["measurement"] == "isolated_rss_delta"]
    largest = max((_int(row["objects"]) for row in rows), default=0)
    if not largest:
        return ["No isolated RSS delta rows are available."]
    observations = []
    for backend in sorted({row["backend"] for row in rows}):
        values = [
            _float(row["peak_bytes"]) for row in rows if row["backend"] == backend and _int(row["objects"]) == largest
        ]
        median_value = sorted(values)[len(values) // 2]
        observations.append(
            f"At {largest:,} objects, {backend.title()} has a median isolated RSS delta of {median_value / 1024 / 1024:.2f} MiB ({median_value / largest:.0f} bytes/object)."
        )
    return observations


def _mode_ratio_observations(summary, feature):
    rows = [row for row in summary if row["feature"] == feature]
    observations = []
    for backend in sorted({row["backend"] for row in rows}):
        backend_rows = [row for row in rows if row["backend"] == backend]
        largest_steps = max((_int(row["trajectory_steps"]) for row in backend_rows), default=0)
        ratios = {}
        for batch_size in (31, 32, 128):
            scalar = next(
                (
                    row
                    for row in backend_rows
                    if row["api_mode"] == "scalar"
                    and _int(row["batch_size"]) == batch_size
                    and _int(row["trajectory_steps"]) == largest_steps
                    and not _int(row["time_window_steps"])
                ),
                None,
            )
            batch = next(
                (
                    row
                    for row in backend_rows
                    if row["api_mode"] == "batch_global"
                    and _int(row["batch_size"]) == batch_size
                    and _int(row["trajectory_steps"]) == largest_steps
                    and not _int(row["time_window_steps"])
                ),
                None,
            )
            if scalar and batch:
                ratios[batch_size] = _float(scalar["ns_per_query_median"]) / _float(batch["ns_per_query_median"])
        if ratios:
            observations.append(
                f"For {backend.title()} at {largest_steps} trajectory steps, scalar/batch ratios are "
                + ", ".join(f"{ratio:.2f} at batch {size}" for size, ratio in ratios.items())
                + "."
            )
    return observations or ["No paired scalar and batch rows are available."]


def _range_observations(rows, field):
    observations = []
    for backend in sorted({row["backend"] for row in rows}):
        selected = [row for row in rows if row["backend"] == backend and row["api_mode"] == "batch_global"]
        if not selected:
            continue
        smallest = min(selected, key=lambda row: _int(row[field]))
        largest = max(selected, key=lambda row: _int(row[field]))
        ratio = _float(largest["ns_per_query_median"]) / max(_float(smallest["ns_per_query_median"]), 1e-12)
        observations.append(
            f"For {backend.title()}, increasing {field.replace('_', ' ')} from {smallest[field]} to {largest[field]} changes batch cost by {ratio:.2f}x."
        )
    return observations or ["No time-window scaling rows are available."]


def _time_variant_observations(summary):
    rows = [row for row in summary if row["feature"] == "time_variant"]
    observations = []
    for backend in sorted({row["backend"] for row in rows}):
        selected = [row for row in rows if row["backend"] == backend and row["api_mode"] == "scalar"]
        smallest = [row for row in selected if _int(row["trajectory_steps"]) == 1]
        largest = [row for row in selected if _int(row["trajectory_steps"]) == 16]
        if smallest and largest:
            start = median([_float(row["ns_per_query_median"]) for row in smallest])
            end = median([_float(row["ns_per_query_median"]) for row in largest])
            observations.append(
                f"{backend.title()} scalar cost rises {end / start:.1f}x from 1 to 16 trajectory steps after taking the median across shape-variation classes."
            )
    return observations or ["No time-variant rows are available."]


def _layer_observations(summary):
    rows = [row for row in summary if row["feature"] == "native_layers"]
    public_ratios = []
    python_ratios = []
    for backend in {row["backend"] for row in rows}:
        for workload in {row["workload"] for row in rows}:
            native = next(
                (
                    row
                    for row in rows
                    if row["backend"] == backend
                    and row["workload"] == workload
                    and row["execution_layer"] == "engine_native"
                ),
                None,
            )
            public = next(
                (
                    row
                    for row in rows
                    if row["backend"] == backend
                    and row["workload"] == workload
                    and row["execution_layer"] == "rust_public_convert_and_query"
                ),
                None,
            )
            python = next(
                (
                    row
                    for row in rows
                    if row["backend"] == backend
                    and row["workload"] == workload
                    and row["execution_layer"] == "python_end_to_end"
                ),
                None,
            )
            if native and public:
                public_ratios.append(
                    (_float(public["ns_per_query_median"]) / _float(native["ns_per_query_median"]), backend, workload)
                )
            if python and public:
                python_ratios.append(
                    (_float(python["ns_per_query_median"]) / _float(public["ns_per_query_median"]), backend, workload)
                )
    if not public_ratios:
        return ["No matched three-layer rows are available."]
    largest = max(public_ratios)
    observations = [
        f"The largest public/native cost ratio is {largest[0]:.2f} for {largest[1].title()} on {largest[2].replace('_', ' ')}."
    ]
    if python_ratios:
        largest_python = max(python_ratios)
        observations.append(
            f"The largest Python/public cost ratio is {largest_python[0]:.2f} for {largest_python[1].title()} on {largest_python[2].replace('_', ' ')}."
        )
    return observations


def _plot_interpretation(name, observations):
    if not observations or observations == ["No plottable rows are available."]:
        return "The current artifacts do not support a comparative interpretation."
    return f"Artifact-derived result: {observations[0]}"


def _md(value):
    if value in (None, ""):
        return "—"
    return str(value).replace("|", "\\|").replace("\n", " ").replace("`", "\\`")


def write_metadata(path: Path, config: BenchmarkConfig, suite: str | None = None):
    metadata_payload = {
        "schema_version": SCHEMA_VERSION,
        "command": {
            "sample_count": config.sample_count,
            "repetitions": config.repetitions,
            "seed": config.seed,
            "thread_counts": list(config.thread_counts),
            "engines": list(config.engines),
            "scenarios": [str(path) for path in config.scenario_paths],
            "profile": config.profile,
            "suites": [suite] if suite else list(config.suites),
            "include_stress": config.include_stress,
        },
        "environment": {
            "python": sys.version.split()[0],
            "platform": platform.platform(),
            "processor": platform.processor(),
            "commonroad_io": _package_version("commonroad-io"),
            "numpy": _package_version("numpy"),
            "rustc": _rustc_version(),
            "compiler_flags": "release profile recommended: uv run main.py study",
            "memory_allocator": "platform default",
            "turbo_boost": "not detected by benchmark harness",
        },
        "git": {"revision": _git_revision()},
        "unsupported_spec_items": unsupported_spec_items(),
    }
    path.write_text(json.dumps(metadata_payload, indent=2, sort_keys=True) + "\n")


def _package_version(package: str):
    try:
        return metadata.version(package)
    except metadata.PackageNotFoundError:
        return None


def _git_revision():
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
    except Exception:
        return None
    return completed.stdout.strip()


def _rustc_version():
    try:
        completed = subprocess.run(
            ["rustc", "--version"],
            check=True,
            capture_output=True,
            text=True,
        )
    except Exception:
        return None
    return completed.stdout.strip()


def unsupported_spec_items():
    return [
        "3D sphere/box/capsule/cylinder primitives",
        "polygon mesh and BVH construction/update benchmarks",
        "narrowphase contact generation/contact count",
        "mutable internal CollisionObject update API",
    ]


def _float(value):
    if value in (None, ""):
        return 0.0
    return float(value)


def _int(value):
    if value in (None, ""):
        return 0
    return int(float(value))
