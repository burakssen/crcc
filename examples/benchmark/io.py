import csv
import json
import platform
import subprocess
import sys
from importlib import metadata
from pathlib import Path
from typing import Any

from .config import SCHEMA_VERSION, BenchmarkConfig
from .results import (
    COMPARISON_FIELDS,
    CORRECTNESS_FIELDS,
    PARALLEL_SCALING_FIELDS,
    RUN_FIELDS,
    SUMMARY_FIELDS,
    compare_runs,
    correctness_row,
    run_row,
    summarize_runs,
)


def write_artifacts(config: BenchmarkConfig, runs, correctness, parallel_rows):
    config.output_dir.mkdir(parents=True, exist_ok=True)
    write_dicts(config.output_dir / "runs.csv", RUN_FIELDS, [run_row(result) for result in runs])
    summary_rows = summarize_runs(runs)
    comparison_rows = compare_runs(runs)
    correctness_rows = [correctness_row(result) for result in correctness]
    write_dicts(config.output_dir / "summary.csv", SUMMARY_FIELDS, summary_rows)
    write_dicts(config.output_dir / "comparisons.csv", COMPARISON_FIELDS, comparison_rows)
    write_dicts(
        config.output_dir / "correctness.csv", CORRECTNESS_FIELDS, correctness_rows
    )
    write_dicts(config.output_dir / "parallel_scaling.csv", PARALLEL_SCALING_FIELDS, parallel_rows)
    write_metadata(config.output_dir / "metadata.json", config)
    write_report(config.output_dir / "benchmark_report.md", config, summary_rows, comparison_rows, correctness_rows)


def write_dicts(path: Path, fields: list[str], rows: list[dict[str, Any]]):
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def read_dicts(path: Path):
    with path.open(newline="") as file:
        return list(csv.DictReader(file))


def write_report_from_artifacts(output_dir: Path):
    output_dir = Path(output_dir)
    metadata = json.loads((output_dir / "metadata.json").read_text()) if (output_dir / "metadata.json").exists() else {}
    summary_rows = read_dicts(output_dir / "summary.csv") if (output_dir / "summary.csv").exists() else []
    comparison_rows = read_dicts(output_dir / "comparisons.csv") if (output_dir / "comparisons.csv").exists() else []
    correctness_rows = read_dicts(output_dir / "correctness.csv") if (output_dir / "correctness.csv").exists() else []
    write_report_rows(output_dir / "benchmark_report.md", metadata, summary_rows, comparison_rows, correctness_rows)


def write_report(path: Path, config: BenchmarkConfig, summary_rows, comparison_rows, correctness_rows):
    metadata = {
        "command": {
            "sample_count": config.sample_count,
            "repetitions": config.repetitions,
            "seed": config.seed,
            "thread_counts": list(config.thread_counts),
            "engines": list(config.engines),
            "scenarios": [str(path) for path in config.scenario_paths],
        }
    }
    write_report_rows(path, metadata, summary_rows, comparison_rows, correctness_rows)


def write_report_rows(path: Path, metadata, summary_rows, comparison_rows, correctness_rows):
    command = metadata.get("command", {})
    mismatches = sum(_int(row["mismatches"]) for row in correctness_rows)
    unsupported = [row for row in summary_rows if str(row["unsupported"]).lower() == "true"]
    fastest = sorted(
        (row for row in summary_rows if _float(row["throughput_median"]) > 0),
        key=lambda row: _float(row["throughput_median"]),
        reverse=True,
    )[:12]
    decisive = [row for row in comparison_rows if row["verdict"] != "inconclusive"][:12]
    lines = [
        "# CRCC Benchmark Report",
        "",
        "## Method",
        "",
        f"- Schema: {SCHEMA_VERSION}",
        f"- Samples per workload: {_int(command.get('sample_count')):,}",
        f"- Repetitions: {_int(command.get('repetitions'))}",
        f"- Engines: {', '.join(command.get('engines', []))}",
        f"- Scenarios: {', '.join(command.get('scenarios', []))}",
        f"- Thread counts: {', '.join(str(item) for item in command.get('thread_counts', []))}",
        "",
        "The study uses paired backend comparisons, deterministic workload seeds, analytic correctness oracles where possible, and bootstrap confidence intervals for speedup medians.",
        "",
        "## Correctness",
        "",
        f"- Total mismatches: {mismatches}",
        f"- Unsupported backend/workload groups: {len(unsupported)}",
        "",
        "## Fastest Median Throughput Rows",
        "",
        "| feature | workload | scenario | backend | queries/s | ns/query |",
        "|---|---|---|---:|---:|---:|",
    ]
    for row in fastest:
        lines.append(
            f"| {row['feature']} | {row['workload']} | {row['scenario']} | {row['backend']} | "
            f"{_float(row['throughput_median']):.1f} | {_float(row['ns_per_query_median']):.1f} |"
        )
    lines.extend(
        [
            "",
            "## Decisive Backend Comparisons",
            "",
            "| feature | workload | backend | speedup median | 95% CI | verdict |",
            "|---|---|---:|---:|---:|---|",
        ]
    )
    for row in decisive:
        lines.append(
            f"| {row['feature']} | {row['workload']} | {row['backend']} | "
            f"{_float(row['speedup_median']):.3f} | "
            f"[{_float(row['speedup_ci_low']):.3f}, {_float(row['speedup_ci_high']):.3f}] | {row['verdict']} |"
        )
    lines.extend(
        [
            "",
            "## Plots",
            "",
            "Plots are written as PNG and PDF under `plots/`. Use the CSV files for exact values.",
            "",
        ]
    )
    path.write_text("\n".join(lines))


def write_metadata(path: Path, config: BenchmarkConfig):
    metadata_payload = {
        "schema_version": SCHEMA_VERSION,
        "command": {
            "sample_count": config.sample_count,
            "repetitions": config.repetitions,
            "seed": config.seed,
            "thread_counts": list(config.thread_counts),
            "engines": list(config.engines),
            "scenarios": [str(path) for path in config.scenario_paths],
        },
        "environment": {
            "python": sys.version.split()[0],
            "platform": platform.platform(),
            "processor": platform.processor(),
            "commonroad_io": _package_version("commonroad-io"),
            "numpy": _package_version("numpy"),
        },
        "git": {"revision": _git_revision()},
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


def _float(value):
    if value in (None, ""):
        return 0.0
    return float(value)


def _int(value):
    if value in (None, ""):
        return 0
    return int(float(value))
