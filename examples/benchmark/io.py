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
    write_dicts(config.output_dir / "summary.csv", SUMMARY_FIELDS, summarize_runs(runs))
    write_dicts(config.output_dir / "comparisons.csv", COMPARISON_FIELDS, compare_runs(runs))
    write_dicts(
        config.output_dir / "correctness.csv", CORRECTNESS_FIELDS, [correctness_row(result) for result in correctness]
    )
    write_dicts(config.output_dir / "parallel_scaling.csv", PARALLEL_SCALING_FIELDS, parallel_rows)
    write_metadata(config.output_dir / "metadata.json", config)


def write_dicts(path: Path, fields: list[str], rows: list[dict[str, Any]]):
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def read_dicts(path: Path):
    with path.open(newline="") as file:
        return list(csv.DictReader(file))


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
