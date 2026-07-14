import math
import os
import shutil
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from .config import ENGINE_ITEMS
from .io import ArtifactBundle, load_artifacts

# Apply a clean, modern design styling globally
plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Inter", "SF Pro Text", "Helvetica Neue", "Arial", "DejaVu Sans", "sans-serif"],
        "font.size": 9.5,
        "axes.titlesize": 11,
        "axes.titleweight": "bold",
        "axes.labelsize": 9.5,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "legend.fontsize": 8.5,
        "legend.title_fontsize": 9.5,
        "figure.facecolor": "#ffffff",
        "axes.facecolor": "#ffffff",
    }
)

BACKEND_COLORS = {"parry": "#3b82f6", "rhusics": "#f43f5e", "collide": "#10b981"}
BACKEND_MARKERS = {"parry": "o", "rhusics": "s", "collide": "^"}
RAYON_QUERY_THRESHOLD = 32
PLOT_NAMES = {
    "backend_throughput_iqr",
    "backend_speedup_forest",
    "latency_percentiles",
    "scene_scaling_curves",
    "rayon_scaling_summary",
    "rayon_efficiency_summary",
    "commonroad_scenario_sequential",
    "commonroad_scenario_rayon",
    "correctness_mismatch_matrix",
    "update_time_scaling",
    "density_scaling_curves",
    "shape_complexity_throughput",
    "memory_growth",
    "api_batch_amortization_sequential",
    "api_batch_amortization_rayon",
    "dynamic_batch_amortization_sequential",
    "dynamic_batch_amortization_rayon",
    "dynamic_time_window_scaling_sequential",
    "dynamic_time_window_scaling_rayon",
    "time_variant_query_scaling_sequential",
    "time_variant_query_scaling_rayon",
    "execution_layer_cost",
}


def write_plots(output_dir: Path, artifacts: ArtifactBundle | None = None):
    output_dir = Path(output_dir)
    artifacts = artifacts or load_artifacts(output_dir)
    summary = artifacts.get("summary.csv")
    comparisons = artifacts.get("comparisons.csv")
    correctness = artifacts.get("correctness.csv")
    parallel = artifacts.get("parallel_scaling.csv")
    memory = artifacts.get("memory.csv")

    with tempfile.TemporaryDirectory(prefix="crcc-plots-", dir=output_dir) as temporary:
        plot_dir = Path(temporary)
        _write_plots(plot_dir, summary, comparisons, correctness, parallel, memory)
        destination = output_dir / "plots"
        backup = output_dir / ".plots-backup"
        if backup.exists():
            shutil.rmtree(backup)
        if destination.exists():
            destination.rename(backup)
        try:
            shutil.copytree(plot_dir, destination)
        except Exception:
            if destination.exists():
                shutil.rmtree(destination)
            if backup.exists():
                backup.rename(destination)
            raise
        if backup.exists():
            shutil.rmtree(backup)


def _write_plots(plot_dir: Path, summary, comparisons, correctness, parallel, memory):
    plottable_summary = [row for row in summary if not _is_true(row.get("unsupported"))]
    _plot_backend_throughput_dotplot(plot_dir / "backend_throughput_iqr", plottable_summary)
    _plot_latency_tail_ratio(plot_dir / "latency_percentiles", plottable_summary)
    _plot_scene_scaling_curves(plot_dir / "scene_scaling_curves", plottable_summary)
    _plot_parallel_summary(plot_dir / "rayon_scaling_summary", parallel, "speedup", "speedup vs 1 Rayon worker")
    _plot_parallel_summary(plot_dir / "rayon_efficiency_summary", parallel, "efficiency", "Rayon efficiency")
    _plot_scenario_throughput(plot_dir / "commonroad_scenario_sequential", plottable_summary, "sequential")
    _plot_scenario_throughput(plot_dir / "commonroad_scenario_rayon", plottable_summary, "rayon")
    _plot_correctness_summary(plot_dir / "correctness_mismatch_matrix", correctness)
    _plot_backend_speedup_forest(plot_dir / "backend_speedup_forest", comparisons)
    _plot_feature_scaling(
        plot_dir / "update_time_scaling",
        plottable_summary,
        "update_proxy",
        "objects",
        "ns_per_query_median",
        "Pose-query Proxy Time (not scene mutation)",
        "ns/query",
    )
    _plot_feature_scaling(
        plot_dir / "density_scaling_curves",
        plottable_summary,
        "density_scaling",
        "objects",
        "throughput_median",
        "Density Scaling Throughput",
        "queries/s",
    )
    _plot_shape_complexity(plot_dir / "shape_complexity_throughput", plottable_summary)
    _plot_memory_growth(plot_dir / "memory_growth", memory)
    for execution_mode in ("sequential", "rayon"):
        _plot_api_batch_amortization(
            plot_dir / f"api_batch_amortization_{execution_mode}", plottable_summary, execution_mode
        )
        _plot_dynamic_batch_amortization(
            plot_dir / f"dynamic_batch_amortization_{execution_mode}", plottable_summary, execution_mode
        )
        _plot_dynamic_time_window_scaling(
            plot_dir / f"dynamic_time_window_scaling_{execution_mode}", plottable_summary, execution_mode
        )
        _plot_time_variant_scaling(
            plot_dir / f"time_variant_query_scaling_{execution_mode}", plottable_summary, execution_mode
        )
    _plot_execution_layer_cost(plot_dir / "execution_layer_cost", plottable_summary)


def _plot_backend_throughput_dotplot(path_base: Path, rows):
    synthetic_rows = _synthetic_summary_rows(rows)
    labels = _ordered_workload_labels(synthetic_rows)
    if not labels:
        _plot_status(path_base, "Backend Throughput", "No synthetic benchmark rows")
        return

    backends = _present_backends(synthetic_rows)
    x_base = np.arange(len(labels))
    bar_width = 0.22
    offsets = (
        np.linspace(-bar_width * (len(backends) - 1) / 2, bar_width * (len(backends) - 1) / 2, len(backends))
        if len(backends) > 1
        else [0.0]
    )

    fig, ax = plt.subplots(figsize=(10.2, 5.8), layout="constrained")
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

    ax.set_title("Backend Throughput by Synthetic Workload", loc="left", pad=12)
    ax.set_ylabel("median throughput (queries/s, log)")
    ax.set_xticks(x_base)
    ax.set_xticklabels([_display_workload(label) for label in labels], rotation=45, ha="right", rotation_mode="anchor")
    ax.set_yscale("log")
    _style_axis(ax, axis="y")
    _legend_outside(fig, ax, backends)
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
    offsets = (
        np.linspace(-bar_width * (len(backends) - 1) / 2, bar_width * (len(backends) - 1) / 2, len(backends))
        if len(backends) > 1
        else [0.0]
    )

    fig, ax = plt.subplots(figsize=(10.2, 5.8), layout="constrained")
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

    ax.axhline(1.0, color="#6b7280", linestyle="--", linewidth=1.0, alpha=0.6)
    ax.set_title("Tail Latency Ratio by Workload", loc="left", pad=12)
    ax.set_ylabel("p99 / p50 latency ratio (log)")
    ax.set_xticks(x_base)
    ax.set_xticklabels([_display_workload(label) for label in labels], rotation=45, ha="right", rotation_mode="anchor")
    ax.set_yscale("log")
    _style_axis(ax, axis="y")
    _legend_outside(fig, ax, backends)
    _save_plot(fig, path_base)


def _plot_scene_scaling_curves(path_base: Path, rows):
    scene_rows = [row for row in rows if row["feature"] in {"scene_scaling", "dynamic_scene"}]
    families = sorted({row.get("shape_family") or row.get("shape") or "unspecified" for row in scene_rows})
    modes = [
        mode
        for mode in ("static_static", "dynamic_static", "pure_dynamic")
        if any(row.get("scene_mode") == mode for row in scene_rows)
    ]
    if not families or not modes:
        _plot_status(path_base, "Scene Scaling", "No scene scaling rows")
        return

    backends = _present_backends(scene_rows)
    fig, axes = plt.subplots(
        len(modes),
        len(families),
        figsize=(3.4 * len(families), 3.0 * len(modes)),
        sharex=True,
        sharey=True,
        squeeze=False,
        layout="constrained",
    )
    for row_index, mode in enumerate(modes):
        for column_index, family in enumerate(families):
            ax = axes[row_index, column_index]
            selected = [
                row
                for row in scene_rows
                if row.get("scene_mode") == mode and (row.get("shape_family") or row.get("shape")) == family
            ]
            densities = sorted({row.get("density", "") for row in selected}) or [""]
            for backend in backends:
                for density_index, density in enumerate(densities):
                    backend_rows = sorted(
                        [row for row in selected if row["backend"] == backend and row.get("density", "") == density],
                        key=lambda row: _int(row["objects"]),
                    )
                    if backend_rows:
                        ax.plot(
                            [_int(row["objects"]) for row in backend_rows],
                            [_float(row["throughput_median"]) for row in backend_rows],
                            marker=BACKEND_MARKERS.get(backend, "o"),
                            color=BACKEND_COLORS.get(backend),
                            linestyle="-" if density_index == 0 else "--",
                            linewidth=1.5,
                        )
            ax.set_title(f"{mode.replace('_', ' ')}\n{family.replace('_', ' ')}", loc="left", fontsize=8.8)
            ax.set_xscale("log")
            ax.set_yscale("log")
            _style_axis(ax)
            if column_index == 0:
                ax.set_ylabel("queries/s")
            if row_index == len(modes) - 1:
                ax.set_xlabel("environment objects")
    fig.suptitle(
        "Scene Scaling by Mode and Shape Family",
        x=0.02,
        ha="left",
        fontsize=11.5,
        fontweight="bold",
        color="#111827",
    )
    _legend_outside(fig, axes.ravel()[0], backends, style="line")
    _save_plot(fig, path_base)


def _plot_scenario_throughput(path_base: Path, rows, execution_mode: str):
    workload = "static_sequential" if execution_mode == "sequential" else "static_parallel"
    scenario_rows = [row for row in rows if row["feature"] == "scenario" and row["workload"] == workload]
    scenarios = sorted({row["scenario"] for row in scenario_rows})
    backends = _present_backends(scenario_rows)
    if not scenarios or not backends:
        _plot_status(path_base, f"Scenario {execution_mode.title()} Throughput", "No scenario rows")
        return

    x_base = np.arange(len(scenarios))
    bar_width = 0.22
    offsets = (
        np.linspace(-bar_width * (len(backends) - 1) / 2, bar_width * (len(backends) - 1) / 2, len(backends))
        if len(backends) > 1
        else [0.0]
    )

    fig, ax = plt.subplots(figsize=(10.2, 5.8), layout="constrained")
    for offset, backend in zip(offsets, backends, strict=True):
        values = []
        positions = []
        for index, scenario in enumerate(scenarios):
            values.append(_scenario_value(scenario_rows, scenario, backend, workload))
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

    ax.set_title(f"CommonRoad Scenario {execution_mode.title()} Throughput", loc="left", pad=12)
    ax.set_ylabel("queries/s")
    ax.set_xticks(x_base)
    ax.set_xticklabels(
        [_short_scenario_label(scenario) for scenario in scenarios], rotation=45, ha="right", rotation_mode="anchor"
    )
    _style_axis(ax, axis="y")
    _legend_outside(fig, ax, backends)
    _save_plot(fig, path_base)


def _plot_parallel_summary(path_base: Path, rows, metric: str, ylabel: str):
    if not rows:
        _plot_status(path_base, ylabel.title(), "No parallel scaling rows")
        return

    backends = _present_backends(rows)
    fig, ax = plt.subplots(figsize=(8.8, 5.4), layout="constrained")
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
            markersize=6,
        )
        ax.fill_between(threads, lows, highs, color=color, alpha=0.12, linewidth=0)

    thread_values = sorted({_int(row["threads"]) for row in rows})
    if thread_values:
        ax.set_xticks(thread_values)
        ax.set_xticklabels([str(t) for t in thread_values])
    if metric == "speedup" and thread_values:
        ax.plot(thread_values, thread_values, color="#6b7280", linestyle="--", linewidth=1.2, alpha=0.6, label="ideal")
    if metric == "efficiency":
        ax.axhline(1.0, color="#6b7280", linestyle="--", linewidth=1.2, alpha=0.6, label="ideal")
        ax.set_ylim(bottom=0)
    ax.set_title("Rayon Scaling Summary" if metric == "speedup" else "Rayon Efficiency Summary", loc="left", pad=12)
    ax.set_xlabel("threads")
    ax.set_ylabel(ylabel)
    _style_axis(ax)
    legend = ax.legend(
        loc="upper left" if metric == "speedup" else "lower left",
        fontsize=8.5,
        title_fontsize=9,
        frameon=True,
        facecolor="#f9fafb",
        edgecolor="#e5e7eb",
        fancybox=True,
    )
    legend.get_frame().set_linewidth(0.8)
    _save_plot(fig, path_base)


def _plot_feature_scaling(path_base: Path, rows, feature: str, x_field: str, y_field: str, title: str, ylabel: str):
    feature_rows = [row for row in rows if row["feature"] == feature]
    if not feature_rows:
        _plot_status(path_base, title, f"No {feature} rows")
        return

    backends = _present_backends(feature_rows)
    group_field = "transform_kind" if feature == "update_proxy" else "density_label"
    present_groups = {row.get(group_field, "") for row in feature_rows}
    preferred_order = (
        ("translation", "rotation", "translation_rotation", "randomized")
        if feature == "update_proxy"
        else ("sparse", "medium", "dense", "worst_case")
    )
    groups = [group for group in preferred_order if group in present_groups]
    groups.extend(sorted(present_groups - set(groups)))
    columns = min(2, len(groups))
    rows_count = math.ceil(len(groups) / columns)
    fig, axes = plt.subplots(
        rows_count,
        columns,
        figsize=(10.2, max(4.8, 4.1 * rows_count)),
        sharex=True,
        sharey=True,
        squeeze=False,
        layout="constrained",
    )
    for ax, group in zip(axes.ravel(), groups, strict=False):
        for backend in backends:
            backend_rows = sorted(
                [row for row in feature_rows if row["backend"] == backend and row.get(group_field, "") == group],
                key=lambda row: _int(row[x_field]),
            )
            if not backend_rows:
                continue
            ax.plot(
                [_int(row[x_field]) for row in backend_rows],
                [_float(row[y_field]) for row in backend_rows],
                marker=BACKEND_MARKERS.get(backend, "o"),
                color=BACKEND_COLORS.get(backend),
                linewidth=1.5,
                alpha=0.85,
            )
        ax.set_title(group.replace("_", " ").title(), loc="left", fontsize=9.5, color="#374151")
        ax.set_xscale("log")
        ax.set_yscale("log")
        _style_axis(ax)

    for ax in axes.ravel()[len(groups) :]:
        ax.set_visible(False)
    for ax in axes[-1, :]:
        if ax.get_visible():
            ax.set_xlabel(x_field.replace("_", " "))
    for ax in axes[:, 0]:
        if ax.get_visible():
            ax.set_ylabel(ylabel)
    fig.suptitle(title, x=0.02, ha="left", fontsize=11.5, fontweight="bold", color="#111827")
    fig.legend(
        handles=_backend_handles(backends, "line"),
        title="Backend",
        loc="upper right",
        bbox_to_anchor=(0.99, 0.99),
        frameon=True,
        facecolor="#f9fafb",
        edgecolor="#e5e7eb",
    )
    _save_plot(fig, path_base)


def _plot_shape_complexity(path_base: Path, rows):
    shape_rows = [row for row in rows if row["feature"] == "shape_complexity"]
    if not shape_rows:
        _plot_status(path_base, "Shape Complexity Throughput", "No shape complexity rows")
        return
    labels = _ordered_workload_labels(shape_rows)
    backends = _present_backends(shape_rows)
    x_base = np.arange(len(labels))
    offsets = _offsets(backends, 0.24)
    fig, ax = plt.subplots(figsize=(10.2, 5.8), layout="constrained")
    for offset, backend in zip(offsets, backends, strict=True):
        values = [_summary_value(shape_rows, label, backend, "throughput_median") for label in labels]
        ax.bar(
            x_base + offset,
            [value if value > 0 else 0.1 for value in values],
            width=0.22,
            color=BACKEND_COLORS.get(backend),
            label=backend,
            edgecolor="none",
            zorder=3,
        )
    ax.set_title("Shape Complexity Throughput", loc="left", pad=12)
    ax.set_ylabel("median throughput (queries/s, log)")
    ax.set_xticks(x_base)
    ax.set_xticklabels([_display_workload(label) for label in labels], rotation=45, ha="right")
    ax.set_yscale("log")
    _style_axis(ax, axis="y")
    _legend_outside(fig, ax, backends)
    _save_plot(fig, path_base)


def _plot_memory_growth(path_base: Path, rows):
    if not rows:
        _plot_status(path_base, "Memory Growth", "No memory rows")
        return
    rows = [row for row in rows if row.get("measurement") == "isolated_rss_delta"]
    if not rows:
        _plot_status(path_base, "Incremental Memory Growth", "No isolated RSS delta rows")
        return
    backends = _present_backends(rows)
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 5.2), sharex=True, layout="constrained")
    for backend in backends:
        grouped = {}
        for row in rows:
            if row["backend"] == backend:
                grouped.setdefault(_int(row["objects"]), []).append(_float(row["peak_bytes"]))
        objects = sorted(grouped)
        medians = []
        lows = []
        highs = []
        for count in objects:
            low, median, high = _median_iqr(grouped[count])
            lows.append(low)
            medians.append(median)
            highs.append(high)
        color = BACKEND_COLORS.get(backend)
        for ax, divisor in zip(axes, (1024 * 1024, np.asarray(objects)), strict=True):
            values = np.asarray(medians) / divisor
            lower = np.asarray(lows) / divisor
            upper = np.asarray(highs) / divisor
            ax.plot(
                objects,
                values,
                marker=BACKEND_MARKERS.get(backend, "o"),
                color=color,
                linewidth=1.8,
            )
            ax.fill_between(objects, lower, upper, color=color, alpha=0.12, linewidth=0)
    axes[0].set_title("Incremental RSS", loc="left", pad=10)
    axes[0].set_ylabel("median RSS delta (MiB)")
    axes[1].set_title("Memory Cost per Object", loc="left", pad=10)
    axes[1].set_ylabel("median incremental bytes/object")
    for ax in axes:
        ax.set_xlabel("static objects")
        ax.set_xscale("log")
        ax.set_ylim(bottom=0)
        _style_axis(ax)
    fig.suptitle("Isolated Static Checker Memory Growth", x=0.02, ha="left", fontsize=11.5, fontweight="bold")
    fig.legend(
        handles=_backend_handles(backends, "line"),
        title="Backend",
        loc="upper right",
        bbox_to_anchor=(0.99, 0.99),
        frameon=True,
        facecolor="#f9fafb",
        edgecolor="#e5e7eb",
    )
    _save_plot(fig, path_base)


def _plot_api_batch_amortization(path_base: Path, rows, execution_mode: str):
    api_rows = [row for row in rows if row["feature"] == "api_overhead" and _execution_mode(row) == execution_mode]
    if not api_rows:
        _plot_status(path_base, f"Python API {execution_mode.title()} Cost", "No API overhead rows")
        return
    backends = _present_backends(api_rows)
    fig, axes = plt.subplots(
        1, len(backends), figsize=(12.4, 4.8), sharex=True, sharey=True, squeeze=False, layout="constrained"
    )
    mode_styles = {
        "python_scalar": ("--", "scalar calls"),
        "python_batch": ("-", "global-pool batch"),
        "python_batch_fresh_pool_1t": (":", "fresh 1-thread pool"),
    }
    if execution_mode == "rayon":
        mode_styles.pop("python_scalar")
    for ax, backend in zip(axes.ravel(), backends, strict=True):
        for workload, (linestyle, _) in mode_styles.items():
            selected = sorted(
                [
                    row
                    for row in api_rows
                    if row["backend"] == backend and row["workload"] == workload and _int(row["queries"]) > 0
                ],
                key=lambda row: _int(row["queries"]),
            )
            ax.plot(
                [_int(row["queries"]) for row in selected],
                [_float(row["ns_per_query_median"]) for row in selected],
                color=BACKEND_COLORS.get(backend),
                marker=BACKEND_MARKERS.get(backend, "o"),
                linestyle=linestyle,
            )
        ax.set_title(backend.title(), loc="left", color=BACKEND_COLORS.get(backend))
        ax.set_xlabel("queries per call")
        ax.set_xscale("log", base=2)
        ax.set_yscale("log")
        _style_axis(ax)
    axes[0, 0].set_ylabel("median ns/query")
    fig.suptitle(
        f"Python API {execution_mode.title()} Call Cost",
        x=0.02,
        ha="left",
        fontsize=11.5,
        fontweight="bold",
        zorder=10,
    )
    fig.legend(
        handles=[
            *[
                Line2D([0], [0], color="#4b5563", linestyle=linestyle, label=label)
                for linestyle, label in mode_styles.values()
            ],
        ],
        title="API mode",
        loc="upper right",
        bbox_to_anchor=(0.99, 0.99),
        frameon=True,
        facecolor="#f9fafb",
        edgecolor="#e5e7eb",
    )
    _save_plot(fig, path_base)


def _plot_dynamic_batch_amortization(path_base: Path, rows, execution_mode: str):
    dynamic_rows = [row for row in rows if row["feature"] == "dynamic_batch" and _execution_mode(row) == execution_mode]
    steps = sorted({_int(row["trajectory_steps"]) for row in dynamic_rows})
    if not steps:
        _plot_status(path_base, f"Dynamic {execution_mode.title()} Cost", "No dynamic batch rows")
        return
    backends = _present_backends(dynamic_rows)
    fig, axes = plt.subplots(
        1, len(steps), figsize=(12.8, 4.8), sharex=True, sharey=True, squeeze=False, layout="constrained"
    )
    for ax, trajectory_steps in zip(axes.ravel(), steps, strict=True):
        for backend in backends:
            modes = (("dynamic_scalar", "--"), ("dynamic_batch", "-"))
            if execution_mode == "rayon":
                modes = (("dynamic_batch", "-"),)
            for workload, linestyle in modes:
                selected = sorted(
                    [
                        row
                        for row in dynamic_rows
                        if row["backend"] == backend
                        and row["workload"] == workload
                        and _int(row["trajectory_steps"]) == trajectory_steps
                    ],
                    key=lambda row: _int(row["batch_size"]),
                )
                ax.plot(
                    [_int(row["batch_size"]) for row in selected],
                    [_float(row["ns_per_query_median"]) for row in selected],
                    color=BACKEND_COLORS.get(backend),
                    marker=BACKEND_MARKERS.get(backend, "o"),
                    linestyle=linestyle,
                    linewidth=1.6,
                )
        ax.set_title(f"{trajectory_steps} trajectory steps", loc="left")
        ax.set_xscale("log", base=2)
        ax.set_yscale("log")
        ax.set_xlabel("dynamic queries per call")
        _style_axis(ax)
    axes[0, 0].set_ylabel("median ns/query")
    fig.suptitle(
        f"Dynamic {execution_mode.title()} API Cost",
        x=0.02,
        ha="left",
        fontsize=11.5,
        fontweight="bold",
        zorder=10,
    )
    fig.legend(
        handles=[
            *_backend_handles(backends, "line"),
            Line2D(
                [0],
                [0],
                color="#4b5563",
                linestyle="--" if execution_mode == "sequential" else "-",
                label=execution_mode,
            ),
        ],
        loc="upper right",
        bbox_to_anchor=(0.99, 0.99),
        frameon=True,
    )
    _save_plot(fig, path_base)


def _plot_time_variant_scaling(path_base: Path, rows, execution_mode: str):
    variant_rows = [row for row in rows if row["feature"] == "time_variant" and _execution_mode(row) == execution_mode]
    variations = sorted({row["shape_variation"] for row in variant_rows})
    if not variations:
        _plot_status(path_base, f"Time-Variant {execution_mode.title()} Scaling", "No time-variant rows")
        return
    backends = _present_backends(variant_rows)
    fig, axes = plt.subplots(
        1, len(variations), figsize=(12.8, 4.8), sharex=True, sharey=True, squeeze=False, layout="constrained"
    )
    for ax, variation in zip(axes.ravel(), variations, strict=True):
        for backend in backends:
            modes = (
                (("time_variant_scalar", "--"),) if execution_mode == "sequential" else (("time_variant_batch", "-"),)
            )
            for workload, linestyle in modes:
                selected = sorted(
                    [
                        row
                        for row in variant_rows
                        if row["backend"] == backend
                        and row["workload"] == workload
                        and row["shape_variation"] == variation
                    ],
                    key=lambda row: _int(row["trajectory_steps"]),
                )
                ax.plot(
                    [_int(row["trajectory_steps"]) for row in selected],
                    [_float(row["ns_per_query_median"]) for row in selected],
                    color=BACKEND_COLORS.get(backend),
                    marker=BACKEND_MARKERS.get(backend, "o"),
                    linestyle=linestyle,
                    linewidth=1.6,
                )
        ax.set_title(variation.replace("_", " ").title(), loc="left")
        ax.set_xscale("log", base=2)
        ax.set_yscale("log")
        ax.set_xlabel("trajectory steps")
        _style_axis(ax)
    axes[0, 0].set_ylabel("median ns/query")
    fig.suptitle(
        f"Time-Variant Shape {execution_mode.title()} Scaling",
        x=0.02,
        ha="left",
        fontsize=11.5,
        fontweight="bold",
        zorder=10,
    )
    fig.legend(
        handles=[
            *_backend_handles(backends, "line"),
            Line2D(
                [0],
                [0],
                color="#4b5563",
                linestyle="--" if execution_mode == "sequential" else "-",
                label=execution_mode,
            ),
        ],
        loc="upper right",
        frameon=True,
    )
    _save_plot(fig, path_base)


def _plot_dynamic_time_window_scaling(path_base: Path, rows, execution_mode: str):
    window_rows = [
        row for row in rows if row["scene_kind"] == "dynamic_time_window" and _execution_mode(row) == execution_mode
    ]
    if not window_rows:
        _plot_status(path_base, f"Dynamic Time-Window {execution_mode.title()} Scaling", "No time-window rows")
        return
    backends = _present_backends(window_rows)
    fig, ax = plt.subplots(figsize=(8.8, 5.2), layout="constrained")
    for backend in backends:
        modes = (("scalar", "--"),) if execution_mode == "sequential" else (("batch_global", "-"),)
        for mode, linestyle in modes:
            selected = sorted(
                [row for row in window_rows if row["backend"] == backend and row["api_mode"] == mode],
                key=lambda row: _int(row["time_window_steps"]),
            )
            ax.plot(
                [_int(row["time_window_steps"]) for row in selected],
                [_float(row["ns_per_query_median"]) for row in selected],
                color=BACKEND_COLORS.get(backend),
                marker=BACKEND_MARKERS.get(backend, "o"),
                linestyle=linestyle,
                linewidth=1.7,
            )
    ax.set_title(f"Dynamic {execution_mode.title()} Cost by Requested Time Window", loc="left")
    ax.set_xlabel("time steps searched")
    ax.set_ylabel("median ns/query")
    ax.set_xscale("log", base=2)
    ax.set_yscale("log")
    _style_axis(ax)
    fig.legend(
        handles=[
            *_backend_handles(backends, "line"),
            Line2D(
                [0],
                [0],
                color="#4b5563",
                linestyle="--" if execution_mode == "sequential" else "-",
                label=execution_mode,
            ),
        ],
        loc="upper right",
        frameon=True,
    )
    _save_plot(fig, path_base)


def _plot_execution_layer_cost(path_base: Path, rows):
    layer_rows = [row for row in rows if row["feature"] == "native_layers"]
    workloads = sorted({row["workload"] for row in layer_rows})
    if not workloads:
        _plot_status(path_base, "Execution Layer Cost", "No native-layer rows")
        return
    backends = _present_backends(layer_rows)
    x = np.arange(len(workloads))
    fig, axes = plt.subplots(
        2,
        len(backends),
        figsize=(15.2, 8.2),
        sharex="col",
        sharey="row",
        gridspec_kw={"height_ratios": (1.35, 1)},
        layout="constrained",
        squeeze=False,
    )
    layer_specs = (
        ("engine_native", "Native Rust", "#475569"),
        ("rust_public_convert_and_query", "Public Rust", "#94a3b8"),
        ("python_end_to_end", "Python", "#e2e8f0"),
    )
    width = 0.24
    for column, backend in enumerate(backends):
        cost_ax, ratio_ax = axes[:, column]
        costs = {layer: [] for layer, _, _ in layer_specs}
        ratios = []
        for workload in workloads:
            matched = {
                row["execution_layer"]: row
                for row in layer_rows
                if row["backend"] == backend and row["workload"] == workload
            }
            for layer, _, _ in layer_specs:
                row = matched.get(layer)
                costs[layer].append(_float(row["ns_per_query_median"]) if row else np.nan)
            native = costs["engine_native"][-1]
            python = costs["python_end_to_end"][-1]
            ratios.append(python / native if native > 0 and python > 0 else np.nan)

        for index, (layer, label, color) in enumerate(layer_specs):
            cost_ax.bar(x + (index - 1) * width, costs[layer], width=width, color=color, label=label, zorder=3)
        ratio_ax.bar(x, ratios, width=0.66, color=BACKEND_COLORS.get(backend), zorder=3)
        ratio_ax.axhline(1.0, color="#6b7280", linestyle="--", linewidth=1.0)

        cost_ax.set_title(backend.title(), color=BACKEND_COLORS.get(backend), loc="left", pad=8)
        cost_ax.set_yscale("log")
        ratio_ax.set_yscale("log")
        ratio_ax.set_xticks(x)
        ratio_ax.set_xticklabels(
            [workload.replace("_", " ") for workload in workloads], rotation=42, ha="right", rotation_mode="anchor"
        )
        _style_axis(cost_ax, axis="y")
        _style_axis(ratio_ax, axis="y")

    axes[0, 0].set_ylabel("median cost (ns/query, log)")
    axes[1, 0].set_ylabel("Python / native Rust cost ratio (log)")
    fig.suptitle("Python Binding Cost vs Native Rust", x=0.01, ha="left", fontsize=16)
    fig.legend(
        handles=[Patch(facecolor=color, label=label) for _, label, color in layer_specs],
        loc="upper right",
        frameon=True,
    )
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
    fig, ax = plt.subplots(figsize=(9.8, max(4.8, len(labels) * 0.4)), layout="constrained")
    ax.barh(np.arange(len(labels)), [values[index] for index in order], color="#ef4444", edgecolor="none", height=0.6)
    ax.set_title(f"Correctness Mismatches ({total_mismatches:,} total)", loc="left", pad=12)
    ax.set_xlabel("mismatches")
    ax.set_yticks(np.arange(len(labels)))
    ax.set_yticklabels([labels[index] for index in order])
    _style_axis(ax, axis="x")
    _save_plot(fig, path_base)


def _plot_throughput_variability_ratio(path_base: Path, rows):
    synthetic_rows = _synthetic_run_rows(rows)
    labels = _ordered_workload_labels(synthetic_rows)
    backends = _present_backends(synthetic_rows)
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
    offsets = (
        np.linspace(-bar_width * (len(backends) - 1) / 2, bar_width * (len(backends) - 1) / 2, len(backends))
        if len(backends) > 1
        else [0.0]
    )

    fig, ax = plt.subplots(figsize=(10.2, 5.8), layout="constrained")
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

    ax.set_title("Throughput Variability Across Repetitions", loc="left", pad=12)
    ax.set_ylabel("relative IQR: (q75 - q25) / median")
    ax.set_xticks(x_base)
    ax.set_xticklabels([_display_workload(label) for label in labels], rotation=45, ha="right", rotation_mode="anchor")
    _style_axis(ax, axis="y")
    _legend_outside(fig, ax, backends)
    _save_plot(fig, path_base)


def _plot_backend_speedup_forest(path_base: Path, rows):
    if not rows:
        _plot_status(path_base, "Backend Speedup vs Baseline", "No paired comparison rows")
        return

    ordered = sorted(rows, key=lambda row: (_float(row["speedup_median"]), row["feature"], row["workload"]))
    labels = [_comparison_label(row) for row in ordered]
    y = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(10.4, _figure_height(labels)), layout="constrained")
    for index, row in enumerate(ordered):
        median = _float(row["speedup_median"])
        low = _float(row["speedup_ci_low"])
        high = _float(row["speedup_ci_high"])
        color = BACKEND_COLORS.get(row["backend"], "#4b5563")
        ax.errorbar(
            median,
            y[index],
            xerr=[[max(0.0, median - low)], [max(0.0, high - median)]],
            fmt=BACKEND_MARKERS.get(row["backend"], "o"),
            color=color,
            ecolor=color,
            elinewidth=1.4,
            capsize=3,
            markersize=5.5,
            zorder=3,
        )

    ax.axvline(1.0, color="#6b7280", linestyle="--", linewidth=1.1, alpha=0.75)
    ax.set_title("Backend Speedup vs Baseline", loc="left", pad=12)
    ax.set_xlabel("speedup ratio with 95% bootstrap CI (log scale)")
    ax.set_yticks(y)
    ax.set_yticklabels(labels)
    ax.set_xscale("log")
    _style_axis(ax, axis="x")
    _save_plot(fig, path_base)


def _plot_throughput_repetition_strip(path_base: Path, rows):
    synthetic_rows = _synthetic_run_rows(rows)
    labels = _ordered_workload_labels(synthetic_rows)
    backends = _present_backends(synthetic_rows)
    if not labels or not backends:
        _plot_status(path_base, "Throughput by Repetition", "No per-run synthetic rows")
        return

    x_base = np.arange(len(labels))
    offsets = _offsets(backends, 0.24)
    fig, ax = plt.subplots(figsize=(10.8, 5.8), layout="constrained")
    for offset, backend in zip(offsets, backends, strict=True):
        color = BACKEND_COLORS.get(backend)
        for index, label in enumerate(labels):
            values = [
                _float(row["queries_per_s"])
                for row in synthetic_rows
                if _row_workload_label(row) == label and row["backend"] == backend and _float(row["queries_per_s"]) > 0
            ]
            if not values:
                continue
            x = x_base[index] + offset
            jitter = np.linspace(-0.035, 0.035, len(values)) if len(values) > 1 else [0.0]
            ax.scatter(
                [x + item for item in jitter],
                values,
                color=color,
                alpha=0.48,
                s=18,
                linewidth=0,
                zorder=3,
            )
            ax.plot(
                [x - 0.055, x + 0.055], [np.median(values), np.median(values)], color=color, linewidth=2.0, zorder=4
            )

    ax.set_title("Throughput Distribution Across Repetitions", loc="left", pad=12)
    ax.set_ylabel("queries/s (log)")
    ax.set_xticks(x_base)
    ax.set_xticklabels([_display_workload(label) for label in labels], rotation=45, ha="right", rotation_mode="anchor")
    ax.set_yscale("log")
    _style_axis(ax, axis="y")
    _legend_outside(fig, ax, backends, style="marker")
    _save_plot(fig, path_base)


def _synthetic_summary_rows(rows):
    return [row for row in rows if row["feature"] not in {"api_overhead", "scenario", "scene_scaling"}]


def _synthetic_run_rows(rows):
    return [row for row in rows if row["feature"] not in {"api_overhead", "scenario", "scene_scaling"}]


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
    order = {"pair": 0, "continuous": 1, "distance": 2}
    return sorted(
        {_row_workload_label(row) for row in rows}, key=lambda label: (order.get(label.split(":")[0], 99), label)
    )


def _row_workload_label(row):
    return f"{row['feature']}:{row['workload']}"


def _comparison_label(row):
    base = f"{row['backend']} / {row['feature']}:{row['workload']}"
    if row.get("scenario"):
        base = f"{row['backend']} / {row['scenario']}:{row['workload']}"
    if row.get("objects"):
        base = f"{base} / n={row['objects']}, hit={_float(row['density']):.0%}"
    if row["feature"] == "api_overhead":
        base = f"{base} / batch={row['queries']}"
    return base


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


def _legend_outside(fig, ax, backends, *, style="bar"):
    legend = ax.legend(
        handles=_backend_handles(backends, style),
        title="Backend",
        loc="center left",
        bbox_to_anchor=(1.02, 0.5),
        fontsize=8.5,
        title_fontsize=9.5,
        frameon=True,
        facecolor="#f9fafb",
        edgecolor="#e5e7eb",
        fancybox=True,
    )
    legend.get_frame().set_linewidth(0.8)


def _backend_handles(backends, style):
    if style == "bar":
        return [Patch(facecolor=BACKEND_COLORS.get(backend), edgecolor="none", label=backend) for backend in backends]
    return [
        Line2D(
            [0],
            [0],
            marker=BACKEND_MARKERS.get(backend, "o"),
            color=BACKEND_COLORS.get(backend),
            label=backend,
            linestyle="-" if style == "line" else "None",
            markersize=6,
        )
        for backend in backends
    ]


def _style_axis(ax, *, axis="both"):
    ax.grid(axis=axis, which="major", color="#e5e7eb", linestyle="-", linewidth=0.5)
    if ax.get_yscale() == "log" or ax.get_xscale() == "log":
        ax.grid(axis=axis, which="minor", color="#f3f4f6", linestyle=":", linewidth=0.4)
    ax.set_axisbelow(True)
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    for spine in ["left", "bottom"]:
        ax.spines[spine].set_color("#9ca3af")
        ax.spines[spine].set_linewidth(0.8)
    ax.tick_params(which="major", colors="#4b5563", width=0.8, length=4)
    ax.tick_params(which="minor", colors="#9ca3af", width=0.6, length=2)


def _plot_status(path_base: Path, title: str, message: str, detail: str | None = None):
    fig, ax = plt.subplots(figsize=(7.2, 4.2), layout="constrained")
    ax.set_title(title, loc="left", pad=12)
    ax.text(0.5, 0.56, message, ha="center", va="center", transform=ax.transAxes, fontsize=12, color="#374151")
    if detail:
        ax.text(0.5, 0.44, detail, ha="center", va="center", transform=ax.transAxes, fontsize=9.5, color="#6b7280")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)
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


def _execution_mode(row):
    workload = row.get("workload", "")
    if workload == "static_sequential" or row.get("api_mode") == "scalar" or workload.endswith("_scalar"):
        return "sequential"
    if workload == "static_parallel" or row.get("api_mode") == "batch_reusable":
        return "rayon"
    batch_size = _int(row.get("batch_size") or row.get("queries"))
    return "rayon" if batch_size >= RAYON_QUERY_THRESHOLD else "sequential"


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
            parsed_rows.append(
                {
                    "backend": r["backend"],
                    "threads": int(r["threads"]),
                    "objects": objects,
                    "density": density,
                    "queries_per_s": float(r["queries_per_s"]),
                }
            )

    if not parsed_rows:
        _plot_status(path_base, "Parallel Scene Scaling", "No parsed scene scaling parallel rows")
        return

    backends = sorted({r["backend"] for r in parsed_rows})
    object_sizes = sorted({r["objects"] for r in parsed_rows})

    fig, axes = plt.subplots(
        len(backends), 1, figsize=(8.8, max(4.8, 3.0 * len(backends))), sharex=True, squeeze=False, layout="constrained"
    )

    colors = cm.viridis(np.linspace(0.1, 0.9, len(object_sizes)))

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
                    linewidth=1.8,
                    markersize=5,
                )

        ax.set_title(f"Engine: {backend}", loc="left", fontsize=9.5, fontweight="semibold", color="#374151")
        ax.set_ylabel("queries/s")
        ax.set_yscale("log")
        ax.set_xscale("log", base=2)
        all_threads = sorted({r["threads"] for r in parsed_rows})
        ax.set_xticks(all_threads)
        ax.set_xticklabels([str(t) for t in all_threads])
        _style_axis(ax)

    axes.ravel()[-1].set_xlabel("thread count")
    fig.suptitle(
        "Parallel Throughput by Thread Count and Object Size",
        x=0.02,
        ha="left",
        fontsize=11.5,
        fontweight="bold",
        color="#111827",
    )

    handles, labels_list = axes.ravel()[0].get_legend_handles_labels()
    legend = axes.ravel()[0].legend(
        handles,
        labels_list,
        loc="center left",
        bbox_to_anchor=(1.02, 0.5),
        title="Scene Size",
        fontsize=8.5,
        title_fontsize=9.5,
        frameon=True,
        facecolor="#f9fafb",
        edgecolor="#e5e7eb",
        fancybox=True,
    )
    legend.get_frame().set_linewidth(0.8)
    _save_plot(fig, path_base)
