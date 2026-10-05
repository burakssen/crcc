"""Matched raw-versus-prepared query ablation for CRCC's Python API."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import statistics
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from crcc import Circle, CollisionCheckerBuilder, DynamicObstacle, Pose

from .config import ENGINE_BY_NAME, ENGINE_ITEMS

STUDY_SCHEMA_VERSION = "1"
DEFAULT_REUSE_COUNTS = tuple(2**power for power in range(11))
DEFAULT_OUTPUT_DIR = Path("target/prepared-query-ablation")
DYNAMIC_TRAJECTORY_STEPS = 64
_STATIC_POSE = Pose.from_translation((3.0, 0.0))
_QUERY_RADIUS = 0.25
_SCENE_RADIUS = 1.0
_QUERY_DISTANCE = 3.0

RUN_FIELDS = [
    "study_schema_version",
    "operation",
    "backend",
    "repetition",
    "reuse_count",
    "trajectory_steps",
    "preparation_ns",
    "raw_query_total_ns",
    "prepared_query_total_ns",
    "prepared_amortized_total_ns",
    "raw_result",
    "prepared_result",
    "query_order",
]

SUMMARY_FIELDS = [
    "study_schema_version",
    "operation",
    "backend",
    "reuse_count",
    "trajectory_steps",
    "repetitions",
    "preparation_ns_median",
    "raw_total_ns_median",
    "prepared_query_total_ns_median",
    "prepared_amortized_total_ns_median",
    "raw_ns_per_query_median",
    "prepared_amortized_ns_per_query_median",
    "speedup_median",
    "prepared_faster_repetitions",
    "first_prepared_faster_reuse_count",
    "last_raw_faster_tested_reuse_count",
]


@dataclass(frozen=True)
class AblationConfig:
    output_dir: Path = DEFAULT_OUTPUT_DIR
    engines: tuple[str, ...] = tuple(name for name, _ in ENGINE_ITEMS)
    repetitions: int = 7
    reuse_counts: tuple[int, ...] = DEFAULT_REUSE_COUNTS
    warmup_queries: int = 100
    preparation_samples: int = 25

    def __post_init__(self):
        object.__setattr__(self, "output_dir", Path(self.output_dir))
        normalized_engines = tuple(str(engine).lower() for engine in self.engines)
        object.__setattr__(self, "engines", normalized_engines)
        object.__setattr__(self, "reuse_counts", tuple(int(count) for count in self.reuse_counts))
        if not self.engines or set(self.engines) - set(ENGINE_BY_NAME):
            unknown = sorted(set(self.engines) - set(ENGINE_BY_NAME))
            raise ValueError(f"unknown or empty engine selection: {', '.join(unknown) or 'no engines'}")
        if self.repetitions < 1:
            raise ValueError("repetitions must be positive")
        if not self.reuse_counts or any(count < 1 for count in self.reuse_counts):
            raise ValueError("reuse counts must be positive")
        if tuple(sorted(set(self.reuse_counts))) != self.reuse_counts:
            raise ValueError("reuse counts must be unique and strictly increasing")
        if self.warmup_queries < 0:
            raise ValueError("warmup query count must be non-negative")
        if self.preparation_samples < 1:
            raise ValueError("preparation samples must be positive")


@dataclass(frozen=True)
class AblationRun:
    operation: str
    backend: str
    repetition: int
    reuse_count: int
    preparation_ns: int
    raw_query_total_ns: int
    prepared_query_total_ns: int
    raw_result: str
    prepared_result: str
    query_order: str
    trajectory_steps: int

    @property
    def prepared_amortized_total_ns(self) -> int:
        return self.preparation_ns + self.prepared_query_total_ns


def run_ablation(config: AblationConfig) -> Path:
    """Run the matched ablation, write its dedicated artifacts, and return the output directory."""
    _prepare_output_directory(config.output_dir)
    runs: list[AblationRun] = []
    for engine_index, backend in enumerate(config.engines):
        checker = (
            CollisionCheckerBuilder(backend=ENGINE_BY_NAME[backend]).add_static_obstacle(Circle(_SCENE_RADIUS)).build()
        )
        cases = _build_cases(checker)
        for operation_index, case in enumerate(cases):
            for repetition in range(config.repetitions):
                prepared, preparation_ns = _measure_preparation(case.prepare, config.preparation_samples)
                raw_signature = _status_signature(case.raw_query())
                prepared_signature = _status_signature(case.prepared_query(prepared))
                if raw_signature != case.expected_result or prepared_signature != case.expected_result:
                    raise RuntimeError(
                        f"{backend} {case.operation} raw/prepared correctness mismatch: "
                        f"expected {case.expected_result}, got {raw_signature} and {prepared_signature}"
                    )

                for _ in range(config.warmup_queries):
                    case.raw_query()
                    case.prepared_query(prepared)

                for sweep_index, reuse_count in enumerate(config.reuse_counts):
                    query_order = (
                        "raw_first"
                        if (engine_index + operation_index + repetition + sweep_index) % 2 == 0
                        else "prepared_first"
                    )

                    def prepared_query():
                        return case.prepared_query(prepared)

                    first, second = (
                        (case.raw_query, prepared_query)
                        if query_order == "raw_first"
                        else (prepared_query, case.raw_query)
                    )
                    first_elapsed = _measure_repeated(first, reuse_count, case.expected_result)
                    second_elapsed = _measure_repeated(second, reuse_count, case.expected_result)
                    raw_elapsed, prepared_elapsed = (
                        (first_elapsed, second_elapsed)
                        if query_order == "raw_first"
                        else (second_elapsed, first_elapsed)
                    )
                    runs.append(
                        AblationRun(
                            operation=case.operation,
                            backend=backend,
                            repetition=repetition,
                            reuse_count=reuse_count,
                            preparation_ns=preparation_ns,
                            raw_query_total_ns=raw_elapsed,
                            prepared_query_total_ns=prepared_elapsed,
                            raw_result=repr(raw_signature),
                            prepared_result=repr(prepared_signature),
                            query_order=query_order,
                            trajectory_steps=case.trajectory_steps,
                        )
                    )

    summary = _summarize_runs(runs)
    _write_csv(config.output_dir / "runs.csv", RUN_FIELDS, (_run_row(run) for run in runs))
    _write_csv(config.output_dir / "summary.csv", SUMMARY_FIELDS, summary)
    _write_metadata(config.output_dir / "metadata.json", config)
    write_ablation_plots(config.output_dir)
    return config.output_dir


@dataclass(frozen=True)
class _QueryCase:
    operation: str
    raw_query: Any
    prepare: Any
    prepared_query: Any
    expected_result: tuple[bool, int | None]
    trajectory_steps: int


def _build_cases(checker) -> tuple[_QueryCase, _QueryCase]:
    query_shape = Circle(_QUERY_RADIUS)

    def static_query():
        return checker.collides_static(query_shape, _STATIC_POSE)

    def static_preparation():
        return checker.prepare_static(query_shape)

    static_case = _QueryCase(
        operation="static",
        raw_query=static_query,
        prepare=static_preparation,
        prepared_query=lambda prepared: checker.collides_static(prepared, _STATIC_POSE),
        expected_result=(False, None),
        trajectory_steps=0,
    )

    trajectory = DynamicObstacle(
        query_shape,
        [Pose.from_translation((_QUERY_DISTANCE, 0.0)) for _ in range(DYNAMIC_TRAJECTORY_STEPS)],
        0,
    )
    dynamic_case = _QueryCase(
        operation="dynamic",
        raw_query=lambda: checker.collides_dynamic(trajectory),
        prepare=lambda: checker.prepare_dynamic(trajectory),
        prepared_query=lambda prepared: checker.collides_dynamic(prepared),
        expected_result=(False, None),
        trajectory_steps=DYNAMIC_TRAJECTORY_STEPS,
    )
    return static_case, dynamic_case


def _measure_preparation(prepare, samples: int):
    elapsed_samples = []
    prepared = None
    for _ in range(samples):
        started = time.perf_counter_ns()
        candidate = prepare()
        elapsed = time.perf_counter_ns() - started
        prepared = candidate
        elapsed_samples.append(elapsed)
    return prepared, round(statistics.median(elapsed_samples))


def _measure_repeated(query, reuse_count: int, expected_result: tuple[bool, int | None]) -> int:
    started = time.perf_counter_ns()
    result = None
    for _ in range(reuse_count):
        result = query()
    elapsed = time.perf_counter_ns() - started
    if _status_signature(result) != expected_result:
        raise RuntimeError(f"query result changed during measurement: {_status_signature(result)}")
    return elapsed


def _status_signature(status) -> tuple[bool, int | None]:
    return bool(status.collides), status.time_step


def _run_row(run: AblationRun) -> dict[str, Any]:
    row = asdict(run)
    row["study_schema_version"] = STUDY_SCHEMA_VERSION
    row["prepared_amortized_total_ns"] = run.prepared_amortized_total_ns
    return row


def _summarize_runs(runs: list[AblationRun]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str, int], list[AblationRun]] = {}
    for run in runs:
        grouped.setdefault((run.operation, run.backend, run.reuse_count), []).append(run)

    grouped_keys = sorted(grouped, key=lambda key: (key[0], key[1], key[2]))
    summaries: list[dict[str, Any]] = []
    for operation, backend, reuse_count in grouped_keys:
        group = grouped[(operation, backend, reuse_count)]
        summaries.append(
            {
                "study_schema_version": STUDY_SCHEMA_VERSION,
                "operation": operation,
                "backend": backend,
                "reuse_count": reuse_count,
                "trajectory_steps": group[0].trajectory_steps,
                "repetitions": len(group),
                "preparation_ns_median": round(statistics.median(run.preparation_ns for run in group)),
                "raw_total_ns_median": round(statistics.median(run.raw_query_total_ns for run in group)),
                "prepared_query_total_ns_median": round(
                    statistics.median(run.prepared_query_total_ns for run in group)
                ),
                "prepared_amortized_total_ns_median": round(
                    statistics.median(run.prepared_amortized_total_ns for run in group)
                ),
                "raw_ns_per_query_median": statistics.median(run.raw_query_total_ns / reuse_count for run in group),
                "prepared_amortized_ns_per_query_median": statistics.median(
                    run.prepared_amortized_total_ns / reuse_count for run in group
                ),
                "speedup_median": statistics.median(
                    run.raw_query_total_ns / run.prepared_amortized_total_ns
                    for run in group
                    if run.prepared_amortized_total_ns > 0
                ),
                "prepared_faster_repetitions": sum(
                    run.prepared_amortized_total_ns < run.raw_query_total_ns for run in group
                ),
                "first_prepared_faster_reuse_count": "",
                "last_raw_faster_tested_reuse_count": "",
            }
        )

    by_operation_backend: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in summaries:
        by_operation_backend.setdefault((row["operation"], row["backend"]), []).append(row)
    for operation_rows in by_operation_backend.values():
        operation_rows.sort(key=lambda row: row["reuse_count"])
        first_faster = next(
            (
                index
                for index, row in enumerate(operation_rows)
                if row["prepared_amortized_total_ns_median"] < row["raw_total_ns_median"]
            ),
            None,
        )
        if first_faster is None:
            continue
        crossing = operation_rows[first_faster]["reuse_count"]
        lower_tested = operation_rows[first_faster - 1]["reuse_count"] if first_faster else ""
        for row in operation_rows:
            row["first_prepared_faster_reuse_count"] = crossing
            row["last_raw_faster_tested_reuse_count"] = lower_tested

    return summaries


def _write_csv(path: Path, fields: list[str], rows):
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _prepare_output_directory(output_dir: Path):
    output_dir = Path(output_dir)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"output directory must be empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)


def _write_metadata(path: Path, config: AblationConfig):
    import crcc._core
    from matplotlib import __version__ as matplotlib_version

    extension_path = Path(crcc._core.__file__).resolve()
    metadata = {
        "study": "prepared_query_ablation",
        "study_schema_version": STUDY_SCHEMA_VERSION,
        "command": {
            "output_dir": str(config.output_dir),
            "engines": list(config.engines),
            "repetitions": config.repetitions,
            "reuse_counts": list(config.reuse_counts),
            "warmup_queries_per_path": config.warmup_queries,
            "preparation_samples_per_repetition": config.preparation_samples,
        },
        "workload": {
            "scene": f"static Circle(radius={_SCENE_RADIUS}) at the origin",
            "static_query": f"Circle(radius={_QUERY_RADIUS}) at x={_QUERY_DISTANCE}",
            "dynamic_query": (
                f"Circle(radius={_QUERY_RADIUS}) at x={_QUERY_DISTANCE} for {DYNAMIC_TRAJECTORY_STEPS} time steps"
            ),
            "expected_result": [False, None],
        },
        "measurement": {
            "interface": "public CRCC Python scalar API",
            "preparation": "median wall-clock duration of prepare_static or prepare_dynamic calls",
            "query_total": "wall-clock duration of repeated public API calls, including the Python loop",
            "amortized_prepared_total": "measured repeated prepared-query total plus measured preparation median",
            "ordering": "raw-first and prepared-first are alternated across the reuse sweep",
            "correctness": "raw and prepared signatures must both match the known workload result before timing",
        },
        "environment": {
            "python": sys.version.split()[0],
            "platform": platform.platform(),
            "processor": platform.processor(),
            "rustc": _command_output(["rustc", "--version"]),
            "matplotlib": matplotlib_version,
            "extension_path": str(extension_path),
            "extension_sha256": _file_sha256(extension_path),
        },
        "git": {
            "revision": _command_output(["git", "rev-parse", "--short", "HEAD"]),
            "working_tree_dirty": bool(_command_output(["git", "status", "--porcelain"])),
            "source_sha256": _source_sha256(),
        },
    }
    path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")


def _source_sha256() -> str:
    repository = Path(__file__).resolve().parents[2]
    source_paths = (
        Path(__file__).resolve(),
        repository / "Cargo.toml",
        repository / "Cargo.lock",
        repository / "src/python/collision_checker.rs",
        repository / "src/python/collision_object.rs",
        repository / "src/python/dynamic_obstacle.rs",
        repository / "src/collision_checker/mod.rs",
    )
    digest = hashlib.sha256()
    for path in source_paths:
        digest.update(path.relative_to(repository).as_posix().encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _command_output(command: list[str]) -> str | None:
    try:
        result = subprocess.run(command, check=True, capture_output=True, text=True)
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip()


def write_ablation_plots(output_dir: Path):
    """Render thesis-style plots from the ablation summary CSV."""
    from matplotlib import pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    output_dir = Path(output_dir)
    plot_dir = output_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    rows = _read_summary(output_dir / "summary.csv")
    with plt.rc_context(
        {
            "font.family": "DejaVu Sans",
            "font.size": 8,
            "axes.titlesize": 9,
            "axes.titleweight": "bold",
            "axes.labelsize": 8,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "legend.fontsize": 7,
            "legend.title_fontsize": 7,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "svg.hashsalt": "benchmark-next-figures-v1",
        }
    ):
        _plot_preparation_cost(plt, Patch, plot_dir / "preparation_cost", rows)
        _plot_reuse_latency(plt, Line2D, plot_dir / "reuse_latency", rows)
        _plot_reuse_speedup(plt, Line2D, plot_dir / "reuse_speedup", rows)


_ABLATION_COLORS = {"parry": "#0072B2", "rhusics": "#D55E00", "collide": "#009E73"}
_ABLATION_MARKERS = {"parry": "^", "rhusics": "D", "collide": "X"}
_ABLATION_HATCHES = {"parry": "///", "rhusics": "\\\\", "collide": "xx"}


def _ablation_backends(rows):
    present = {row["backend"] for row in rows}
    return [backend for backend, _ in ENGINE_ITEMS if backend in present]


def _ablation_backend_label(backend):
    return backend.capitalize()


def _ablation_backend_handles(line_type, backends):
    return [
        line_type(
            [0],
            [0],
            color=_ABLATION_COLORS.get(backend, "#555555"),
            marker=_ABLATION_MARKERS.get(backend, "o"),
            markerfacecolor=_ABLATION_COLORS.get(backend, "#555555"),
            markeredgecolor="#303740",
            linestyle="-",
            linewidth=1.1,
            markersize=4,
            label=_ablation_backend_label(backend),
        )
        for backend in backends
    ]


def _style_ablation_axis(axis, *, grid="both"):
    axis.grid(True, which="major", axis=grid, color="#D9DEE5", linewidth=0.55)
    if axis.get_xscale() == "log" or axis.get_yscale() == "log":
        axis.grid(True, which="minor", axis=grid, color="#EEF0F3", linewidth=0.4, linestyle=":")
    axis.set_axisbelow(True)
    for side in ("top", "right"):
        axis.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        axis.spines[side].set_color("#89929C")
        axis.spines[side].set_linewidth(0.65)
    axis.tick_params(colors="#303740", width=0.6, length=3)


def _plot_preparation_cost(plt, patch_type, path_base: Path, rows):
    fig, axes = plt.subplots(1, 2, figsize=(5.85, 3.1))
    backends = _ablation_backends(rows)
    for axis, operation in zip(axes, ("static", "dynamic"), strict=True):
        operation_rows = [row for row in rows if row["operation"] == operation]
        positions = range(len(backends))
        for backend_index, backend in enumerate(backends):
            backend_rows = [row for row in operation_rows if row["backend"] == backend]
            value = int(backend_rows[0]["preparation_ns_median"]) if backend_rows else 0
            if value:
                axis.bar(
                    positions[backend_index],
                    value,
                    width=0.62,
                    color=_ABLATION_COLORS.get(backend, "#555555"),
                    edgecolor="#303740",
                    hatch=_ABLATION_HATCHES.get(backend, "///"),
                    linewidth=0.9,
                )
        axis.set_xticks(positions, [_ablation_backend_label(backend) for backend in backends])
        axis.set_title(f"{operation.capitalize()} query", loc="left")
        axis.set_ylabel("Preparation time (ns)")
        axis.set_yscale("log")
        _style_ablation_axis(axis, grid="y")
    handles = [
        patch_type(
            facecolor=_ABLATION_COLORS.get(backend, "#555555"),
            edgecolor="#303740",
            hatch=_ABLATION_HATCHES.get(backend, "///"),
            label=_ablation_backend_label(backend),
        )
        for backend in backends
    ]
    if handles:
        fig.legend(handles=handles, loc="lower center", ncol=len(handles), frameon=False)
    fig.tight_layout(rect=(0, 0.12, 1, 1))
    _save_figure(fig, path_base)


def _plot_reuse_latency(plt, line_type, path_base: Path, rows):
    fig, axes = plt.subplots(1, 2, figsize=(5.85, 3.8))
    backends = _ablation_backends(rows)
    for axis, operation in zip(axes, ("static", "dynamic"), strict=True):
        for backend in backends:
            selected = sorted(
                (row for row in rows if row["operation"] == operation and row["backend"] == backend),
                key=lambda row: int(row["reuse_count"]),
            )
            if not selected:
                continue
            reuse = [int(row["reuse_count"]) for row in selected]
            axis.plot(
                reuse,
                [float(row["raw_ns_per_query_median"]) for row in selected],
                color=_ABLATION_COLORS.get(backend, "#555555"),
                marker=_ABLATION_MARKERS.get(backend, "o"),
                linestyle="-",
                linewidth=1.1,
                markersize=3.5,
            )
            axis.plot(
                reuse,
                [float(row["prepared_amortized_ns_per_query_median"]) for row in selected],
                color=_ABLATION_COLORS.get(backend, "#555555"),
                marker=_ABLATION_MARKERS.get(backend, "o"),
                linestyle="--",
                linewidth=1.1,
                markersize=3.5,
            )
        axis.set_xscale("log", base=2)
        axis.set_yscale("log")
        axis.set_xlabel("Prepared-query reuse count")
        axis.set_ylabel("Amortized time (ns/query)")
        axis.set_title(f"{operation.capitalize()} query", loc="left")
        _style_ablation_axis(axis)
    backend_handles = _ablation_backend_handles(line_type, backends)
    style_handles = [
        line_type([0], [0], color="#303740", linestyle="-", linewidth=1.1, label="Raw queries"),
        line_type([0], [0], color="#303740", linestyle="--", linewidth=1.1, label="Preparation + prepared queries"),
    ]
    fig.legend(handles=backend_handles + style_handles, loc="lower center", ncol=3, frameon=False)
    fig.tight_layout(rect=(0, 0.18, 1, 1))
    _save_figure(fig, path_base)


def _plot_reuse_speedup(plt, line_type, path_base: Path, rows):
    fig, axes = plt.subplots(1, 2, figsize=(5.85, 3.5))
    backends = _ablation_backends(rows)
    for axis, operation in zip(axes, ("static", "dynamic"), strict=True):
        for backend in backends:
            selected = sorted(
                (row for row in rows if row["operation"] == operation and row["backend"] == backend),
                key=lambda row: int(row["reuse_count"]),
            )
            if selected:
                axis.plot(
                    [int(row["reuse_count"]) for row in selected],
                    [float(row["speedup_median"]) for row in selected],
                    color=_ABLATION_COLORS.get(backend, "#555555"),
                    marker=_ABLATION_MARKERS.get(backend, "o"),
                    linewidth=1.1,
                    markersize=3.5,
                )
        axis.axhline(1.0, color="#89929C", linewidth=0.9, linestyle="--")
        axis.set_xscale("log", base=2)
        axis.set_yscale("log")
        axis.set_xlabel("Prepared-query reuse count")
        axis.set_ylabel("Raw / amortized prepared time")
        axis.set_title(f"{operation.capitalize()} query", loc="left")
        _style_ablation_axis(axis)
    handles = _ablation_backend_handles(line_type, backends)
    handles.append(line_type([0], [0], color="#89929C", linestyle="--", linewidth=0.9, label="Parity (1×)"))
    fig.legend(handles=handles, loc="lower center", ncol=max(1, len(handles)), frameon=False)
    fig.tight_layout(rect=(0, 0.13, 1, 1))
    _save_figure(fig, path_base)


def _save_figure(fig, path_base: Path):
    from matplotlib import pyplot as plt

    fig.savefig(
        path_base.with_suffix(".pdf"),
        metadata={"Creator": "CRCC prepared-query ablation", "CreationDate": None, "ModDate": None},
    )
    fig.savefig(path_base.with_suffix(".svg"), metadata={"Creator": "CRCC prepared-query ablation", "Date": None})
    fig.savefig(path_base.with_suffix(".png"), dpi=220, metadata={"Software": "CRCC prepared-query ablation"})
    plt.close(fig)


def _read_summary(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise ValueError(f"ablation summary is empty: {path}")
    if any(row.get("study_schema_version") != STUDY_SCHEMA_VERSION for row in rows):
        raise ValueError(f"unsupported ablation summary schema in {path}")
    return rows


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--engines", nargs="+", choices=sorted(ENGINE_BY_NAME), default=list(ENGINE_BY_NAME))
    parser.add_argument("--repetitions", type=int, default=7)
    parser.add_argument("--reuse-counts", nargs="+", type=int, default=list(DEFAULT_REUSE_COUNTS))
    parser.add_argument("--warmup-queries", type=int, default=100)
    parser.add_argument("--preparation-samples", type=int, default=25)
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    config = AblationConfig(
        output_dir=args.output,
        engines=tuple(args.engines),
        repetitions=args.repetitions,
        reuse_counts=tuple(args.reuse_counts),
        warmup_queries=args.warmup_queries,
        preparation_samples=args.preparation_samples,
    )
    output_dir = run_ablation(config)
    print(f"Wrote prepared-query ablation artifacts to {output_dir}")


if __name__ == "__main__":
    main()
