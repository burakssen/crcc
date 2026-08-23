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
    write_result_csvs(config.output_dir, runs, correctness, parallel_rows, memory_rows)
    write_metadata(config.output_dir / "metadata.json", config)
    # ponytail: report from the artifacts just written so step=run produces the
    # complete tables without a second plot step.
    write_report_from_artifacts(config.output_dir)


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
    correctness_errors = sum(_int(row.get("errors", 0)) for row in correctness_rows)
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
        f"- Correctness query errors: **{correctness_errors}**",
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
        "Correctness mismatches count false negatives only; backend query failures are reported separately as `errors`. Conservative false positives are reported in the `fp` column.",
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
        "| feature | workload | scenario | backend | queries | FP | FN | mismatches | errors | oracle |",
        "|---|---|---|---|---:|---:|---:|---:|---:|---|",
    ]
    for row in sorted(correctness_rows, key=lambda item: (item["feature"], item["workload"], item["backend"])):
        lines.append(
            f"| {_md(row['feature'])} | {_md(row['workload'])} | {_md(row['scenario'])} | {_md(row['backend'])} | "
            f"{_int(row['queries'])} | {_int(row['false_positive'])} | {_int(row['false_negative'])} | "
            f"{_int(row['mismatches'])} | {_int(row.get('errors', 0))} | {_md(row['oracle'])} |"
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
    reusable = [row for row in rows if row.get("api_mode") == "batch_reusable"]
    if not reusable:
        return []
    largest_batch_size = max(_int(row.get("batch_size")) for row in reusable)
    grouped = {}
    for row in reusable:
        if _int(row.get("batch_size")) != largest_batch_size:
            continue
        key = (row["scenario"], row["backend"], row["threads"])
        grouped.setdefault(key, []).append(row)
    result = []
    for group in grouped.values():
        representative = dict(group[0])
        representative["speedup"] = f"{median(_float(row['speedup']) for row in group):.3f}"
        representative["efficiency"] = f"{median(_float(row['efficiency']) for row in group):.3f}"
        result.append(representative)
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
    "rayon_scaling_summary": (
        "Summarize throughput speedup as the Rayon worker count increases.",
        "Lines show median speedup relative to one Rayon worker for the largest reusable batch, with separate static and dynamic panels; the dashed diagonal is ideal linear scaling.",
        "Small batches are intentionally excluded from this scaling view because scheduling overhead can dominate them; they remain available in the raw CSV.",
    ),
    "rayon_efficiency_summary": (
        "Show how effectively each backend converts additional Rayon workers into speedup.",
        "Efficiency is speedup divided by thread count for the largest reusable batch, with separate static and dynamic panels. The dashed line at 1 is ideal; declining curves indicate diminishing returns.",
        "Efficiency is derived from the same reusable-pool timing rows as the scaling plot; it is not a CPU-utilization measurement.",
    ),
    "commonroad_scenario_sequential": (
        "Show sequential throughput on each CommonRoad scenario.",
        "Bars report absolute sequential queries per second by scenario and backend; higher is better.",
        "Scenario geometry and query distributions differ, so throughput should be compared within each scenario.",
    ),
    "commonroad_scenario_rayon": (
        "Show Rayon batch throughput on each CommonRoad scenario.",
        "Bars report absolute Rayon-backed batch queries per second by scenario and backend; higher is better.",
        "This view includes Python-call amortization as well as Rayon execution and is not a kernel-only measurement.",
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
    "api_batch_amortization_sequential": (
        "Show the Python scalar baseline without Rayon dispatch.",
        "Each panel fixes a backend and reports median nanoseconds per query; lower is better.",
        "This view is the scalar baseline; batch execution is recorded separately with an explicit parallel mode.",
    ),
    "api_batch_amortization_rayon": (
        "Show Python batch cost after Rayon dispatch begins.",
        "Each panel fixes a backend; solid lines use the global pool and dotted lines create a one-thread pool per call.",
        "Fresh-pool rows include pool construction and should not be treated as steady-state worker throughput.",
    ),
    "dynamic_batch_amortization_sequential": (
        "Show sequential dynamic-obstacle query cost as batch size and trajectory length increase.",
        "Panels fix trajectory length and report median nanoseconds per query for scalar calls.",
        "The synthetic trajectories use circles, translation, and a 50% hit mix; other shapes and hit positions may scale differently.",
    ),
    "dynamic_batch_amortization_rayon": (
        "Show Rayon dynamic-batch cost as batch size and trajectory length increase.",
        "Panels fix trajectory length and contain only rows requested with explicit parallel execution.",
        "The synthetic trajectories use circles, translation, and a 50% hit mix; other shapes and hit positions may scale differently.",
    ),
    "dynamic_time_window_scaling_sequential": (
        "Isolate how the requested time range affects sequential dynamic-query cost.",
        "The x-axis is the number of trajectory steps searched and the y-axis is median nanoseconds per query.",
        "All rows use 16-step source trajectories and vary only the inclusive query window beginning at step zero.",
    ),
    "dynamic_time_window_scaling_rayon": (
        "Isolate how the requested time range affects Rayon dynamic-batch cost.",
        "The x-axis is the number of trajectory steps searched and the y-axis is median batch nanoseconds per query.",
        "All rows use a fixed 32-query batch, so this view does not characterize other batch sizes; execution is explicitly parallel.",
    ),
    "time_variant_query_scaling_sequential": (
        "Measure sequential query cost for dynamic obstacles whose shape changes over time.",
        "Panels represent shape-variation classes; x is trajectory steps and y is median nanoseconds per query.",
        "Construction cost is recorded separately in CSV fields and the plot focuses on warmed query execution.",
    ),
    "time_variant_query_scaling_rayon": (
        "Measure Rayon batch cost for dynamic obstacles whose shape changes over time.",
        "Panels represent shape-variation classes for the fixed 32-query batch; execution is explicitly parallel.",
        "Construction cost is recorded separately in CSV fields and the plot focuses on warmed query execution.",
    ),
    "execution_layer_cost": (
        "Compare native Rust, public Rust, and end-to-end Python binding cost on identical workloads.",
        "The top row reports absolute median nanoseconds per query; the bottom row reports the direct Python/native Rust cost ratio. Both use logarithmic axes and lower is better.",
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
    if name in {"rayon_scaling_summary", "rayon_efficiency_summary"}:
        metric = "speedup" if name == "rayon_scaling_summary" else "efficiency"
        reusable = [row for row in parallel if row.get("api_mode") == "batch_reusable"]
        largest_batch_size = max((_int(row.get("batch_size")) for row in reusable), default=0)
        reusable = [row for row in reusable if _int(row.get("batch_size")) == largest_batch_size]
        observations = []
        for backend in sorted({row["backend"] for row in parallel}):
            backend_rows = [row for row in reusable if row["backend"] == backend]
            for operation in sorted({row.get("operation") or "all" for row in backend_rows}):
                operation_rows = [row for row in backend_rows if (row.get("operation") or "all") == operation]
                if not operation_rows:
                    continue
                best = max(operation_rows, key=lambda row: _float(row[metric]))
                observations.append(
                    f"{backend.title()} {operation} reaches a maximum recorded {metric} of "
                    f"{_float(best[metric]):.2f} at {_int(best['threads'])} threads for batch {largest_batch_size:,}."
                )
        return observations or ["No parallel-scaling rows are available."]
    if name in {"commonroad_scenario_sequential", "commonroad_scenario_rayon"}:
        workload = "static_sequential" if name.endswith("sequential") else "static_parallel"
        return _throughput_observations(
            [row for row in summary if row["feature"] == "scenario" and row["workload"] == workload]
        )
    if name in {"api_batch_amortization_sequential", "api_batch_amortization_rayon"}:
        workload = "python_scalar" if name.endswith("sequential") else "python_batch"
        rows = [row for row in summary if row["feature"] == "api_overhead" and row["workload"] == workload]
        return _cost_observations(rows, "batch_size")
    if name in {"dynamic_batch_amortization_sequential", "dynamic_batch_amortization_rayon"}:
        workload = "dynamic_scalar" if name.endswith("sequential") else "dynamic_batch"
        rows = [row for row in summary if row["feature"] == "dynamic_batch" and row["workload"] == workload]
        return _cost_observations(rows, "batch_size")
    if name in {"dynamic_time_window_scaling_sequential", "dynamic_time_window_scaling_rayon"}:
        rows = [row for row in summary if row.get("scene_kind") == "dynamic_time_window"]
        mode = "scalar" if name.endswith("sequential") else "batch_parallel"
        return _cost_observations([row for row in rows if row["api_mode"] == mode], "time_window_steps")
    if name in {"time_variant_query_scaling_sequential", "time_variant_query_scaling_rayon"}:
        mode = "scalar" if name.endswith("sequential") else "batch_parallel"
        rows = [row for row in summary if row["feature"] == "time_variant" and row["api_mode"] == mode]
        return _cost_observations(rows, "trajectory_steps")
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


def _throughput_observations(rows):
    observations = []
    for backend in sorted({row["backend"] for row in rows}):
        values = [_float(row["throughput_median"]) for row in rows if row["backend"] == backend]
        observations.append(
            f"{backend.title()} records a median of {median(values):,.1f} queries/s across the plotted scenarios."
        )
    return observations or ["No throughput rows are available."]


def _cost_observations(rows, x_field):
    observations = []
    largest = max((_int(row[x_field]) for row in rows), default=0)
    for backend in sorted({row["backend"] for row in rows}):
        values = [
            _float(row["ns_per_query_median"])
            for row in rows
            if row["backend"] == backend and _int(row[x_field]) == largest
        ]
        observations.append(
            f"At {largest} {x_field.replace('_', ' ')}, {backend.title()} records a median cost of {median(values):,.1f} ns/query."
        )
    return observations or ["No cost rows are available."]


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


def _layer_observations(summary):
    rows = [row for row in summary if row["feature"] == "native_layers"]
    python_native_ratios = []
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
            if native and python:
                python_native_ratios.append(
                    (_float(python["ns_per_query_median"]) / _float(native["ns_per_query_median"]), backend, workload)
                )
    if not python_native_ratios:
        return ["No matched three-layer rows are available."]
    largest = max(python_native_ratios)
    return [
        f"The largest direct Python/native Rust cost ratio is {largest[0]:.2f} for {largest[1].title()} on {largest[2].replace('_', ' ')}."
    ]


def _plot_interpretation(_name, observations):
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
