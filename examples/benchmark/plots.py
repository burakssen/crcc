import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D

from .config import ENGINE_ITEMS
from .io import read_dicts

BACKEND_COLORS = {"parry": "#1f77b4", "rhusics": "#ff7f0e", "collide": "#2ca02c"}
BACKEND_MARKERS = {"parry": "o", "rhusics": "s", "collide": "^"}
PLOT_NAMES = {
    "backend_throughput_dotplot",
    "latency_tail_ratio",
    "scene_scaling_curves",
    "scenario_parallel_speedup_dotplot",
    "parallel_scaling_summary",
    "parallel_efficiency_summary",
    "correctness_summary",
    "throughput_variability_ratio",
    "parallel_scene_scaling",
}


def write_plots(output_dir: Path):
    output_dir = Path(output_dir)
    plot_dir = output_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    _clean_plot_dir(plot_dir)

    runs = _read_optional(output_dir / "runs.csv")
    summary = _read_optional(output_dir / "summary.csv")
    correctness = _read_optional(output_dir / "correctness.csv")
    parallel = _read_optional(output_dir / "parallel_scaling.csv")

    plottable_summary = [row for row in summary if not _is_true(row.get("unsupported"))]
    plottable_runs = [row for row in runs if not _is_true(row.get("unsupported"))]
    _plot_backend_throughput_dotplot(plot_dir / "backend_throughput_dotplot", plottable_summary)
    _plot_latency_tail_ratio(plot_dir / "latency_tail_ratio", plottable_summary)
    _plot_scene_scaling_curves(plot_dir / "scene_scaling_curves", plottable_summary)
    _plot_scenario_parallel_speedup_dotplot(plot_dir / "scenario_parallel_speedup_dotplot", plottable_summary)
    _plot_parallel_summary(plot_dir / "parallel_scaling_summary", parallel, "speedup", "speedup vs 1 thread")
    _plot_parallel_summary(plot_dir / "parallel_efficiency_summary", parallel, "efficiency", "parallel efficiency")
    _plot_correctness_summary(plot_dir / "correctness_summary", correctness)
    _plot_throughput_variability_ratio(plot_dir / "throughput_variability_ratio", plottable_runs)
    _plot_parallel_scene_scaling(plot_dir / "parallel_scene_scaling", parallel)


def _plot_backend_throughput_dotplot(path_base: Path, rows):
    synthetic_rows = _synthetic_summary_rows(rows)
    labels = _ordered_workload_labels(synthetic_rows)
    if not labels:
        _plot_status(path_base, "Backend Throughput", "No synthetic benchmark rows")
        return

    backends = _present_backends(synthetic_rows)
    x_base = np.arange(len(labels))
    bar_width = 0.22
    offsets = np.linspace(-bar_width * (len(backends) - 1) / 2, bar_width * (len(backends) - 1) / 2, len(backends)) if len(backends) > 1 else [0.0]
    
    fig, ax = plt.subplots(figsize=(10.2, 5.8))
    for offset, backend in zip(offsets, backends, strict=True):
        values = []
        positions = []
        for index, label in enumerate(labels):
            value = _summary_value(synthetic_rows, label, backend, "throughput_median")
            if value <= 0:
                value = 0.1
            values.append(value)
            positions.append(x_base[index] + offset)
        ax.bar(
            positions,
            values,
            width=bar_width,
            color=BACKEND_COLORS.get(backend),
            label=backend,
            edgecolor="none",
            zorder=3,
        )

    ax.set_title("Backend Throughput by Synthetic Workload")
    ax.set_ylabel("median throughput (queries/s, log)")
    ax.set_xticks(x_base)
    ax.set_xticklabels([_display_workload(label) for label in labels], rotation=45, ha="right")
    ax.set_yscale("log")
    _style_axis(ax, axis="y")
    _legend_outside(fig, ax, backends)
    fig.tight_layout(rect=(0, 0, 0.86, 1))
    _save_plot(fig, path_base)


def _plot_latency_tail_ratio(path_base: Path, rows):
    synthetic_rows = _synthetic_summary_rows(rows)
    labels = _ordered_workload_labels(synthetic_rows)
    if not labels:
        _plot_status(path_base, "Tail Latency Ratio", "No synthetic benchmark rows")
        return

    backends = _present_backends(synthetic_rows)
    x_base = np.arange(len(labels))
    bar_width = 0.22
    offsets = np.linspace(-bar_width * (len(backends) - 1) / 2, bar_width * (len(backends) - 1) / 2, len(backends)) if len(backends) > 1 else [0.0]
    
    fig, ax = plt.subplots(figsize=(10.2, 5.8))
    for offset, backend in zip(offsets, backends, strict=True):
        values = []
        positions = []
        for index, label in enumerate(labels):
            row = _summary_row(synthetic_rows, label, backend)
            if row is None:
                p50, p99 = 1.0, 1.0
            else:
                p50 = _float(row["p50_ns_median"])
                p99 = _float(row["p99_ns_median"])
            if p50 <= 0 or p99 <= 0:
                val = 1.0
            else:
                val = p99 / p50
            values.append(val)
            positions.append(x_base[index] + offset)
        ax.bar(
            positions,
            values,
            width=bar_width,
            color=BACKEND_COLORS.get(backend),
            label=backend,
            edgecolor="none",
            zorder=3,
        )

    ax.axhline(1.0, color="black", linestyle="--", alpha=0.35)
    ax.set_title("Tail Latency Ratio by Workload")
    ax.set_ylabel("p99 / p50 latency ratio (log)")
    ax.set_xticks(x_base)
    ax.set_xticklabels([_display_workload(label) for label in labels], rotation=45, ha="right")
    ax.set_yscale("log")
    _style_axis(ax, axis="y")
    _legend_outside(fig, ax, backends)
    fig.tight_layout(rect=(0, 0, 0.86, 1))
    _save_plot(fig, path_base)


def _plot_scene_scaling_curves(path_base: Path, rows):
    scene_rows = [row for row in rows if row["feature"] == "scene_scaling"]
    densities = sorted({_float(row["density"]) for row in scene_rows if row["density"] != ""})
    if not densities:
        _plot_status(path_base, "Scene Scaling", "No scene scaling rows")
        return

    backends = _present_backends(scene_rows)
    y_values = [_float(row["throughput_median"]) for row in scene_rows if _float(row["throughput_median"]) > 0]
    fig, axes = plt.subplots(
        len(densities),
        1,
        figsize=(8.8, max(4.8, 2.25 * len(densities))),
        sharex=True,
        sharey=True,
        squeeze=False,
    )
    for ax, density in zip(axes.ravel(), densities, strict=True):
        density_rows = [row for row in scene_rows if _float(row["density"]) == density]
        for backend in backends:
            backend_rows = sorted(
                [row for row in density_rows if row["backend"] == backend],
                key=lambda row: _int(row["objects"]),
            )
            if not backend_rows:
                continue
            ax.plot(
                [_int(row["objects"]) for row in backend_rows],
                [_float(row["throughput_median"]) for row in backend_rows],
                marker=BACKEND_MARKERS.get(backend, "o"),
                color=BACKEND_COLORS.get(backend),
                label=backend,
            )
        ax.set_title(f"Collision density {density:.0%}", loc="left", fontsize=10)
        ax.set_ylabel("queries/s")
        ax.set_xscale("log")
        ax.set_yscale("log")
        _style_axis(ax)
    if y_values:
        lower = min(y_values) * 0.8
        upper = max(y_values) * 1.25
        axes.ravel()[0].set_ylim(lower, upper)
    axes.ravel()[-1].set_xlabel("static objects")
    fig.suptitle("Scene Scaling by Object Count and Collision Density")
    _legend_outside(fig, axes.ravel()[0], backends)
    fig.tight_layout(rect=(0, 0, 0.86, 0.96))
    _save_plot(fig, path_base)


def _plot_scenario_parallel_speedup_dotplot(path_base: Path, rows):
    scenario_rows = [row for row in rows if row["feature"] == "scenario"]
    scenarios = sorted({row["scenario"] for row in scenario_rows})
    backends = _present_backends(scenario_rows)
    if not scenarios or not backends:
        _plot_status(path_base, "Scenario Parallel Speedup", "No scenario rows")
        return

    x_base = np.arange(len(scenarios))
    bar_width = 0.22
    offsets = np.linspace(-bar_width * (len(backends) - 1) / 2, bar_width * (len(backends) - 1) / 2, len(backends)) if len(backends) > 1 else [0.0]
    
    fig, ax = plt.subplots(figsize=(10.2, 5.8))
    for offset, backend in zip(offsets, backends, strict=True):
        ratios = []
        positions = []
        for index, scenario in enumerate(scenarios):
            sequential = _scenario_value(scenario_rows, scenario, backend, "static_sequential")
            parallel = _scenario_value(scenario_rows, scenario, backend, "static_parallel")
            if sequential <= 0 or parallel <= 0:
                val = 0.0
            else:
                val = parallel / sequential
            ratios.append(val)
            positions.append(x_base[index] + offset)
        ax.bar(
            positions,
            ratios,
            width=bar_width,
            color=BACKEND_COLORS.get(backend),
            label=backend,
            edgecolor="none",
            zorder=3,
        )

    ax.axhline(1.0, color="black", linestyle="--", alpha=0.35)
    ax.set_title("Scenario Parallel Speedup")
    ax.set_ylabel("parallel / sequential throughput")
    ax.set_xticks(x_base)
    ax.set_xticklabels([_short_scenario_label(scenario) for scenario in scenarios], rotation=45, ha="right")
    _style_axis(ax, axis="y")
    _legend_outside(fig, ax, backends)
    fig.tight_layout(rect=(0, 0, 0.86, 1))
    _save_plot(fig, path_base)


def _plot_parallel_summary(path_base: Path, rows, metric: str, ylabel: str):
    if not rows:
        _plot_status(path_base, ylabel.title(), "No parallel scaling rows")
        return

    backends = _present_backends(rows)
    fig, ax = plt.subplots(figsize=(8.8, 5.4))
    for backend in backends:
        grouped = _group_metric_by_thread([row for row in rows if row["backend"] == backend], metric)
        if not grouped:
            continue
        threads = sorted(grouped)
        medians = []
        lows = []
        highs = []
        for thread in threads:
            low, median, high = _median_iqr(grouped[thread])
            lows.append(low)
            medians.append(median)
            highs.append(high)
        color = BACKEND_COLORS.get(backend)
        ax.plot(
            threads,
            medians,
            marker=BACKEND_MARKERS.get(backend, "o"),
            color=color,
            linewidth=2.0,
            label=backend,
        )
        ax.fill_between(threads, lows, highs, color=color, alpha=0.16, linewidth=0)

    thread_values = sorted({_int(row["threads"]) for row in rows})
    if metric == "speedup" and thread_values:
        ax.plot(thread_values, thread_values, color="black", linestyle="--", alpha=0.35, label="ideal")
    if metric == "efficiency":
        ax.axhline(1.0, color="black", linestyle="--", alpha=0.35, label="ideal")
        ax.set_ylim(bottom=0)
    ax.set_title("Parallel Scaling Summary" if metric == "speedup" else "Parallel Efficiency Summary")
    ax.set_xlabel("threads")
    ax.set_ylabel(ylabel)
    _style_axis(ax)
    ax.legend(fontsize=8)
    fig.tight_layout()
    _save_plot(fig, path_base)


def _plot_correctness_summary(path_base: Path, rows):
    if not rows:
        _plot_status(path_base, "Correctness Summary", "No correctness rows")
        return

    nonzero = [row for row in rows if _int(row["mismatches"]) > 0]
    total_queries = sum(_int(row["queries"]) for row in rows)
    total_mismatches = sum(_int(row["mismatches"]) for row in rows)
    if not nonzero:
        _plot_status(
            path_base,
            "Correctness Summary",
            f"0 mismatches across {total_queries:,} checked queries",
            detail=f"{len(rows)} backend/workload correctness groups matched",
        )
        return

    labels = [f"{row['feature']}:{row['workload']} / {row['backend']}" for row in nonzero]
    values = [_int(row["mismatches"]) for row in nonzero]
    order = np.argsort(values)
    fig, ax = plt.subplots(figsize=(9.8, max(4.8, len(labels) * 0.4)))
    ax.barh(np.arange(len(labels)), [values[index] for index in order], color="#d62728")
    ax.set_title(f"Correctness Mismatches ({total_mismatches:,} total)")
    ax.set_xlabel("mismatches")
    ax.set_yticks(np.arange(len(labels)))
    ax.set_yticklabels([labels[index] for index in order])
    _style_axis(ax, axis="x")
    fig.tight_layout()
    _save_plot(fig, path_base)


def _plot_throughput_variability_ratio(path_base: Path, rows):
    synthetic_rows = _synthetic_run_rows(rows)
    labels = _ordered_workload_labels(synthetic_rows)
    backends = _present_backends(synthetic_run_rows := synthetic_rows)
    repetition_count = len({row["repetition"] for row in synthetic_rows})
    if not labels:
        _plot_status(path_base, "Throughput Variability", "No per-run synthetic rows")
        return
    if repetition_count < 3:
        _plot_status(
            path_base,
            "Throughput Variability",
            "Requires at least 3 repetitions",
            detail=f"Current artifact has {repetition_count} repetition",
        )
        return

    x_base = np.arange(len(labels))
    bar_width = 0.22
    offsets = np.linspace(-bar_width * (len(backends) - 1) / 2, bar_width * (len(backends) - 1) / 2, len(backends)) if len(backends) > 1 else [0.0]
    
    fig, ax = plt.subplots(figsize=(10.2, 5.8))
    for offset, backend in zip(offsets, backends, strict=True):
        values = []
        positions = []
        for index, label in enumerate(labels):
            throughputs = [
                _float(row["queries_per_s"])
                for row in synthetic_rows
                if _row_workload_label(row) == label and row["backend"] == backend
            ]
            if len(throughputs) < 3:
                val = 0.0
            else:
                low, median, high = _median_iqr(throughputs)
                if median <= 0:
                    val = 0.0
                else:
                    val = (high - low) / median
            values.append(val)
            positions.append(x_base[index] + offset)
        ax.bar(
            positions,
            values,
            width=bar_width,
            color=BACKEND_COLORS.get(backend),
            label=backend,
            edgecolor="none",
            zorder=3,
        )

    ax.set_title("Throughput Variability Across Repetitions")
    ax.set_ylabel("relative IQR: (q75 - q25) / median")
    ax.set_xticks(x_base)
    ax.set_xticklabels([_display_workload(label) for label in labels], rotation=45, ha="right")
    _style_axis(ax, axis="y")
    _legend_outside(fig, ax, backends)
    fig.tight_layout(rect=(0, 0, 0.86, 1))
    _save_plot(fig, path_base)


def _clean_plot_dir(plot_dir: Path):
    for path in plot_dir.iterdir():
        if path.is_file() and path.suffix in {".png", ".pdf"}:
            path.unlink()


def _read_optional(path: Path):
    return read_dicts(path) if path.exists() else []


def _synthetic_summary_rows(rows):
    return [row for row in rows if row["feature"] not in {"scenario", "scene_scaling"}]


def _synthetic_run_rows(rows):
    return [row for row in rows if row["feature"] not in {"scenario", "scene_scaling"}]


def _group_metric_by_thread(rows, metric):
    grouped: dict[int, list[float]] = {}
    for row in rows:
        value = _float(row[metric])
        if value <= 0:
            continue
        grouped.setdefault(_int(row["threads"]), []).append(value)
    return grouped


def _median_iqr(values):
    q25, median, q75 = np.percentile(values, [25, 50, 75])
    return float(q25), float(median), float(q75)


def _ordered_workload_labels(rows):
    order = {"pair": 0, "ccd": 1, "distance": 2}
    return sorted({_row_workload_label(row) for row in rows}, key=lambda label: (order.get(label.split(":")[0], 99), label))


def _row_workload_label(row):
    return f"{row['feature']}:{row['workload']}"


def _summary_row(rows, label, backend):
    return next((row for row in rows if _row_workload_label(row) == label and row["backend"] == backend), None)


def _summary_value(rows, label, backend, metric):
    row = _summary_row(rows, label, backend)
    return _float(row[metric]) if row else 0.0


def _scenario_value(rows, scenario, backend, workload):
    row = next(
        (
            item
            for item in rows
            if item["scenario"] == scenario and item["backend"] == backend and item["workload"] == workload
        ),
        None,
    )
    return _float(row["throughput_median"]) if row else 0.0


def _present_backends(rows):
    present = {row["backend"] for row in rows}
    return [backend for backend, _ in ENGINE_ITEMS if backend in present]


def _offsets(items, spread: float):
    if len(items) <= 1:
        return [0.0]
    return np.linspace(-spread, spread, len(items))


def _figure_height(labels):
    return max(4.8, len(labels) * 0.52)


def _legend_outside(fig, ax, backends):
    ax.legend(
        handles=_backend_handles(backends),
        title="Backend",
        loc="center left",
        bbox_to_anchor=(1.01, 0.5),
        fontsize=8,
    )


def _backend_handles(backends):
    return [
        Line2D(
            [0],
            [0],
            marker=BACKEND_MARKERS.get(backend, "o"),
            color=BACKEND_COLORS.get(backend),
            label=backend,
            linestyle="None",
            markersize=7,
        )
        for backend in backends
    ]


def _style_axis(ax, *, axis="both"):
    ax.grid(axis=axis, alpha=0.25)
    ax.set_axisbelow(True)


def _plot_status(path_base: Path, title: str, message: str, detail: str | None = None):
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    ax.set_title(title)
    ax.text(0.5, 0.56, message, ha="center", va="center", transform=ax.transAxes, fontsize=14)
    if detail:
        ax.text(0.5, 0.44, detail, ha="center", va="center", transform=ax.transAxes, fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)
    fig.tight_layout()
    _save_plot(fig, path_base)


def _display_workload(label: str):
    feature, _, workload = label.partition(":")
    return f"{feature}: {workload.replace('_', ' ')}"


def _short_scenario_label(scenario: str):
    if len(scenario) <= 28:
        return scenario
    parts = scenario.split("_")
    if len(parts) >= 2 and not _looks_like_uuid(parts[0]):
        return "_".join(parts[:2])
    return f"{scenario[:8]}...{scenario[-6:]}"


def _looks_like_uuid(value: str):
    return len(value) >= 8 and any(char.isdigit() for char in value) and "-" in value


def _float(value):
    if value in (None, ""):
        return 0.0
    return float(value)


def _int(value):
    if value in (None, ""):
        return 0
    return int(float(value))


def _is_true(value):
    return str(value).lower() == "true"


def _save_plot(fig, path_base: Path):
    fig.savefig(path_base.with_suffix(".png"), dpi=160)
    fig.savefig(path_base.with_suffix(".pdf"))
    plt.close(fig)


def _plot_parallel_scene_scaling(path_base: Path, parallel_rows):
    import re
    import matplotlib.cm as cm
    scene_rows = [row for row in parallel_rows if "scene_scaling_objects_" in row["scenario"]]
    if not scene_rows:
        _plot_status(path_base, "Parallel Scene Scaling", "No scene scaling parallel rows")
        return

    parsed_rows = []
    for r in scene_rows:
        m = re.match(r"scene_scaling_objects_(\d+)_density_(.+)", r["scenario"])
        if m:
            objects = int(m.group(1))
            density = float(m.group(2))
            parsed_rows.append({
                "backend": r["backend"],
                "threads": int(r["threads"]),
                "objects": objects,
                "density": density,
                "queries_per_s": float(r["queries_per_s"])
            })

    if not parsed_rows:
        _plot_status(path_base, "Parallel Scene Scaling", "No parsed scene scaling parallel rows")
        return

    backends = sorted({r["backend"] for r in parsed_rows})
    object_sizes = sorted({r["objects"] for r in parsed_rows})

    fig, axes = plt.subplots(
        len(backends),
        1,
        figsize=(8.8, max(4.8, 3.0 * len(backends))),
        sharex=True,
        squeeze=False
    )

    colors = cm.plasma(np.linspace(0, 0.85, len(object_sizes)))

    for ax, backend in zip(axes.ravel(), backends):
        backend_rows = [r for r in parsed_rows if r["backend"] == backend]
        for color, obj_size in zip(colors, object_sizes):
            obj_rows = [r for r in backend_rows if r["objects"] == obj_size]
            threads_list = sorted({r["threads"] for r in obj_rows})
            qps_by_threads = []
            for t in threads_list:
                vals = [r["queries_per_s"] for r in obj_rows if r["threads"] == t]
                qps_by_threads.append(np.median(vals) if vals else 0.0)

            if qps_by_threads and any(q > 0 for q in qps_by_threads):
                ax.plot(
                    threads_list,
                    qps_by_threads,
                    marker="o",
                    color=color,
                    label=f"{obj_size:,} objects",
                )

        ax.set_title(f"Engine: {backend}", loc="left", fontsize=10)
        ax.set_ylabel("queries/s")
        ax.set_yscale("log")
        ax.set_xscale("log", base=2)
        all_threads = sorted({r["threads"] for r in parsed_rows})
        ax.set_xticks(all_threads)
        ax.set_xticklabels([str(t) for t in all_threads])
        _style_axis(ax)

    axes.ravel()[-1].set_xlabel("thread count")
    fig.suptitle("Parallel Throughput by Thread Count and Object Size")

    handles, labels_list = axes.ravel()[0].get_legend_handles_labels()
    fig.legend(handles, labels_list, loc="center left", bbox_to_anchor=(0.88, 0.5), title="Scene Size")
    fig.tight_layout(rect=(0, 0, 0.86, 0.96))
    _save_plot(fig, path_base)
