import csv
import json
import math
from itertools import pairwise

import pytest
from crcc import CollisionCheckerBuilder
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

import main
from tools.benchmark.config import ENGINE_ITEMS, SCHEMA_VERSION
from tools.benchmark.contract import (
    scene_workload as canonical_scene_workload,
    synthetic_workloads as canonical_synthetic_workloads,
)
from tools.benchmark.io import (
    ARTIFACT_FIELDS,
    ArtifactError,
    _best_parallel_rows,
    _plot_interpretation,
    load_artifacts,
    write_report_from_artifacts,
)
from tools.benchmark.plots import (
    PLOT_NAMES,
    _backend_handles,
    _execution_mode,
    _parallel_scaling_rows,
    _plot_api_batch_amortization,
    _plot_execution_layer_cost,
    _plot_memory_growth,
    _plot_scene_scaling_curves,
)
from tools.benchmark.results import RunResult, compare_layers, compare_modes, compare_runs, run_row, summarize_runs
from tools.benchmark.runner import (
    _correctness_mismatches,
    _execute_pair_query,
    _measure_prepared_scene_with_checker,
    _measure_scene_with_checker,
    _parallel_speedup,
    _python_layer_workload,
    _reusable_thread_counts,
    _synthetic_correctness,
)
from tools.benchmark.workloads import (
    coverage_matrix_workloads,
    dynamic_query_batch,
    planning_frame_workload,
    primitive_queries,
    robustness_queries,
    scene_workload,
    spec_shape_workloads,
    synthetic_workloads,
    time_variant_query_batch,
)


def _row(fields, **values):
    row = dict.fromkeys(fields, "")
    row.update({"schema_version": SCHEMA_VERSION, **values})
    return row


def test_primitive_queries_follow_the_canonical_synthetic_contract():
    records = canonical_synthetic_workloads(4, 2026)["convex_polygon"]
    queries = primitive_queries(4, "convex_polygon", 2026)

    assert [query.expected for query in queries] == [record["expected"] for record in records]
    assert [query.right_pose.translation for query in queries] == [
        tuple(record["right"]["pose"][:2]) for record in records
    ]
    assert [query.right_pose.rotation for query in queries] == pytest.approx(
        [record["right"]["pose"][2] for record in records]
    )


def _write_csv(path, fields, rows=()):
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _write_artifacts(output_dir):
    output_dir.mkdir()
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "command": {
            "sample_count": 100,
            "repetitions": 3,
            "engines": ["parry", "rhusics", "collide"],
            "scenarios": ["scenario|one"],
            "thread_counts": [1, 2],
            "profile": "spec",
            "suites": ["pair"],
            "include_stress": False,
        },
        "environment": {"python": "3.12", "platform": "test", "rustc": "rustc test"},
        "git": {"revision": "abc123"},
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata))
    for filename, fields in ARTIFACT_FIELDS.items():
        rows = []
        if filename == "summary.csv":
            rows = [
                _row(
                    fields,
                    feature="pair",
                    workload="circle|overlap",
                    backend="rhusics",
                    queries="100",
                    repetitions="3",
                    errors_total="0",
                    unsupported="False",
                    throughput_median="1000",
                    ns_per_query_median="1000000",
                )
            ]
        elif filename == "correctness.csv":
            rows = [
                _row(
                    fields,
                    feature="pair",
                    workload="circle|overlap",
                    backend="rhusics",
                    queries="100",
                    false_positive="1",
                    false_negative="0",
                    mismatches="0",
                    oracle="analytic",
                )
            ]
        _write_csv(output_dir / filename, fields, rows)
    return output_dir


def test_missing_artifacts_fail_with_actionable_message(tmp_path):
    with pytest.raises(ArtifactError, match="main.py study"):
        load_artifacts(tmp_path / "missing")


def test_partial_aggregate_is_rejected(tmp_path):
    output_dir = tmp_path / "results"
    output_dir.mkdir()
    _write_csv(output_dir / "summary.csv", ARTIFACT_FIELDS["summary.csv"])
    with pytest.raises(ArtifactError, match="Incomplete aggregate"):
        load_artifacts(output_dir)


def test_mixed_schema_is_rejected(tmp_path):
    output_dir = _write_artifacts(tmp_path / "results")
    summary = output_dir / "summary.csv"
    lines = summary.read_text().splitlines()
    _, separator, remainder = lines[1].partition(",")
    lines[1] = f"old{separator}{remainder}"
    summary.write_text("\n".join(lines) + "\n")
    with pytest.raises(ArtifactError, match="Unsupported benchmark schema"):
        load_artifacts(output_dir)


def test_research_report_contains_provenance_correctness_and_plot_links(tmp_path):
    output_dir = _write_artifacts(tmp_path / "results")
    write_report_from_artifacts(output_dir)
    report = (output_dir / "benchmark_report.md").read_text()

    assert "## Environment And Provenance" in report
    assert "abc123" in report
    assert "## Correctness" in report
    assert "circle\\|overlap" in report
    assert "## Parallel Scaling" in report
    assert "plots/rayon_scaling_summary.png" in report
    assert "## Threats To Validity" in report

    for name in PLOT_NAMES:
        title = name.replace("_", " ").title()
        section = report.split(f"### {title}", 1)[1].split("### ", 1)[0]
        assert f"plots/{name}.png" in section
        assert "**Purpose**" in section
        assert "**How to Read**" in section
        assert "**Observed Results**" in section
        assert "**Interpretation**" in section
        assert "**Limitations**" in section


def test_report_interpretation_uses_artifact_observation():
    observation = "Parry records 1.25x speedup."
    assert _plot_interpretation("rayon_scaling_summary", [observation]) == f"Artifact-derived result: {observation}"


def test_plot_names_exclude_mixed_sequential_rayon_views():
    assert {
        "parallel_scaling_summary",
        "parallel_efficiency_summary",
        "commonroad_scenario_summary",
        "api_batch_amortization",
        "api_batch_speedup",
        "dynamic_batch_amortization",
        "dynamic_time_window_scaling",
        "time_variant_query_scaling",
    }.isdisjoint(PLOT_NAMES)


def test_execution_mode_uses_explicit_batch_mode():
    assert _execution_mode({"api_mode": "scalar", "batch_size": "32"}) == "sequential"
    assert _execution_mode({"api_mode": "batch_sequential", "batch_size": "128"}) == "sequential"
    assert _execution_mode({"api_mode": "batch_parallel", "batch_size": "1"}) == "rayon"
    assert _execution_mode({"api_mode": "batch_parallel_fresh_pool", "batch_size": "1"}) == "rayon"
    assert _execution_mode({"api_mode": "batch_reusable", "batch_size": "512", "threads": "2"}) == "rayon"
    assert _execution_mode({"api_mode": "batch_reusable", "batch_size": "128", "threads": "2"}) == "rayon"
    assert _execution_mode({"workload": "static_sequential"}) == "sequential"
    assert _execution_mode({"workload": "static_parallel"}) == "rayon"


def test_rayon_speedup_uses_one_thread_batch_baseline():
    assert _parallel_speedup(100, 100, 1) == (1.0, 1.0)
    assert _parallel_speedup(50, 100, 2) == (2.0, 1.0)
    assert _parallel_speedup(25, 100, 4) == (4.0, 1.0)


def test_reusable_parallel_suite_preserves_requested_thread_counts():
    assert _reusable_thread_counts((2, 4, 16)) == (1, 2, 4, 16)


def test_rayon_scaling_rows_keep_operation_and_largest_batch_dimensions():
    rows = [
        {
            "api_mode": "batch_reusable",
            "backend": backend,
            "operation": operation,
            "batch_size": str(batch_size),
            "threads": str(threads),
            "speedup": "1.0",
            "efficiency": "1.0",
        }
        for backend in ("parry", "collide")
        for operation in ("static", "dynamic")
        for batch_size in (128, 10_000)
        for threads in (1, 2, 4)
    ]
    rows.append(
        {
            "api_mode": "batch_reusable",
            "backend": "parry",
            "operation": "static",
            "batch_size": "10_000",
            "threads": "8",
            "speedup": "1.0",
            "efficiency": "1.0",
        }
    )

    selected = _parallel_scaling_rows(rows)

    assert {row["operation"] for row in selected} == {"static", "dynamic"}
    assert {int(row["batch_size"]) for row in selected} == {10_000}
    assert {int(row["threads"]) for row in selected if row["backend"] == "parry"} == {1, 2, 4, 8}


def test_parallel_report_uses_largest_batch_median_not_best_repetition():
    rows = [
        {
            "api_mode": "batch_reusable",
            "scenario": "reusable_static_128",
            "backend": "parry",
            "batch_size": "128",
            "threads": "2",
            "speedup": "9.0",
            "efficiency": "4.5",
        },
        {
            "api_mode": "batch_reusable",
            "scenario": "reusable_static_10000",
            "backend": "parry",
            "batch_size": "10000",
            "threads": "2",
            "speedup": "1.5",
            "efficiency": "0.75",
        },
        {
            "api_mode": "batch_reusable",
            "scenario": "reusable_static_10000",
            "backend": "parry",
            "batch_size": "10000",
            "threads": "2",
            "speedup": "2.5",
            "efficiency": "1.25",
        },
    ]

    assert _best_parallel_rows(rows) == [
        {
            **rows[1],
            "speedup": "2.000",
            "efficiency": "1.000",
        }
    ]


def test_report_default_output_is_repository_relative():
    args = main.parse_args(["report"])
    assert args.benchmark_output == str(main.ROOT / "target/crcc-python-bench")


def test_report_forwards_only_supported_benchmark_options(monkeypatch, tmp_path):
    received = {}

    def capture(**options):
        received.update(options)

    monkeypatch.setattr(main.benchmark, "run_all", capture)
    main.main(["report", "--benchmark-output", str(tmp_path), "--benchmark-step", "run"])

    assert received["step"] == "plot"
    assert received["output_dir"] == str(tmp_path)
    assert "benchmark_step" not in received


def _api_result(backend, workload, repetition, queries, total_ns):
    return RunResult(
        "api_overhead",
        None,
        backend,
        workload,
        repetition,
        queries,
        1,
        0.5,
        queries // 2,
        0,
        False,
        total_ns,
        [total_ns // queries],
        scene_kind="python_end_to_end",
    )


def test_api_batch_sizes_remain_separate_in_aggregates():
    results = [
        _api_result(backend, "python_batch", repetition, queries, queries * 100)
        for backend in ("parry", "rhusics")
        for queries in (1, 8, 32)
        for repetition in range(2)
    ]

    summary = summarize_runs(results)
    comparisons = compare_runs(results)

    assert {row["queries"] for row in summary} == {1, 8, 32}
    assert all(row["repetitions"] == 2 for row in summary)
    assert {row["queries"] for row in comparisons} == {1, 8, 32}


def test_backend_legend_handles_match_plot_kind():
    bar_handle = _backend_handles(["parry"], "bar")[0]
    line_handle = _backend_handles(["parry"], "line")[0]

    assert isinstance(bar_handle, Patch)
    assert isinstance(line_handle, Line2D)
    assert line_handle.get_linestyle() == "-"


def test_rhusics_tangency_policy_is_explicit():
    tangent = robustness_queries()[0]

    assert tangent.expected is True
    assert tangent.expected_by_backend == {"rhusics": False}


@pytest.mark.parametrize(("backend", "engine"), ENGINE_ITEMS)
def test_compound_clear_oracle_and_boundary_backend_semantics(backend, engine):
    for workload in spec_shape_workloads(2, (), (64, 256)):
        assert not _execute_pair_query(engine, workload.operation, workload.queries[1])

    boundary = next(
        workload
        for workload in synthetic_workloads(2, 2026)
        if workload.feature == "pair" and workload.workload == "boundary_robustness"
    )
    assert _synthetic_correctness(backend, engine, boundary).mismatches == 0


@pytest.mark.parametrize("engine", [engine for _, engine in ENGINE_ITEMS])
def test_native_layer_extra_python_workloads(engine):
    for name in ("circle_hit", "rectangle_clear", "ccd", "distance"):
        execute, operation, _, _ = _python_layer_workload(engine, name)
        value = execute()
        if operation == "distance":
            assert isinstance(value, float)
            assert math.isfinite(value) and value >= 0
        else:
            collides = value if isinstance(value, bool) else value.collides
            assert isinstance(collides, bool)
            if name == "circle_hit":
                assert collides is True
            elif name == "rectangle_clear":
                assert collides is False


@pytest.mark.parametrize("engine", [engine for _, engine in ENGINE_ITEMS])
@pytest.mark.parametrize(("workload", "expected"), (("compound_clear", False), ("compound_hit", True)))
def test_python_compound_workloads_have_expected_discrete_outcomes(engine, workload, expected):
    execute, operation, _, _ = _python_layer_workload(engine, workload)
    value = execute()
    collides = value if isinstance(value, bool) else value.collides

    assert operation == "discrete"
    assert collides is expected


def test_correctness_rejects_false_positives_unless_ccd_is_explicitly_conservative():
    counts = {"fp": 2, "fn": 1}

    assert _correctness_mismatches(counts, "moving_static") == 3
    assert _correctness_mismatches(counts, "conservative_ccd") == 1


def test_run_rows_leave_percentiles_blank_without_genuine_latency_samples():
    result = RunResult(
        "api_overhead",
        None,
        "parry",
        "batch",
        0,
        8,
        None,
        None,
        4,
        0,
        False,
        800,
        [],
        sample_semantics="batch_average",
    )

    row = run_row(result)
    summary = summarize_runs([result])[0]

    assert all(row[field] == "" for field in ("min_ns", "p50_ns", "p90_ns", "p95_ns", "p99_ns", "max_ns"))
    assert all(summary[field] == "" for field in ("p50_ns_median", "p90_ns_median", "p95_ns_median", "p99_ns_median"))


def test_faceted_api_and_incremental_memory_plots_are_written(tmp_path):
    api_rows = [
        {
            "feature": "api_overhead",
            "backend": backend,
            "workload": workload,
            "queries": str(queries),
            "batch_size": str(queries),
            "api_mode": "scalar" if workload == "python_scalar" else "batch_parallel",
            "ns_per_query_median": str(value),
        }
        for backend in ("parry", "rhusics", "collide")
        for workload, value in (("python_scalar", 400), ("python_batch", 800))
        for queries in (1, 8, 32)
    ]
    memory_rows = [
        {
            "backend": backend,
            "objects": str(objects),
            "peak_bytes": str(objects * multiplier),
            "measurement": "isolated_rss_delta",
        }
        for backend, multiplier in (("parry", 1000), ("rhusics", 1200), ("collide", 1400))
        for objects in (100, 500, 1000)
        for _ in range(3)
    ]

    _plot_api_batch_amortization(tmp_path / "api_sequential", api_rows, "sequential")
    _plot_api_batch_amortization(tmp_path / "api_rayon", api_rows, "rayon")
    _plot_memory_growth(tmp_path / "memory", memory_rows)

    for name in ("api_sequential", "api_rayon", "memory"):
        assert (tmp_path / f"{name}.png").is_file()
        assert (tmp_path / f"{name}.pdf").is_file()


def test_execution_layer_plot_compares_python_with_native_rust(tmp_path):
    rows = [
        {
            "feature": "native_layers",
            "backend": backend,
            "workload": workload,
            "execution_layer": layer,
            "ns_per_query_median": str(cost),
        }
        for backend in ("parry", "rhusics", "collide")
        for workload in ("circle_clear", "compound_clear")
        for layer, cost in (
            ("engine_native", 40),
            ("rust_public_convert_and_query", 400),
            ("python_end_to_end", 800),
        )
    ]

    _plot_execution_layer_cost(tmp_path / "execution_layer", rows)

    assert (tmp_path / "execution_layer.png").is_file()
    assert (tmp_path / "execution_layer.pdf").is_file()


def test_static_scene_plot_separates_prepared_queries_and_marks_capacity_breaks(monkeypatch, tmp_path):
    rows = [
        {
            "feature": "scene_scaling",
            "scene_mode": "static_static",
            "shape_family": "circle",
            "backend": "parry",
            "execution_layer": layer,
            "workload": "capacity_static_scene" if objects == 100_000 else "static_scene",
            "objects": str(objects),
            "queries": str(queries),
            "density": "0.0",
            "throughput_median": str(1_000_000 / latency),
            "errors_total": "0",
            "unsupported": "False",
        }
        for layer, latency in (("python_end_to_end", 120), ("rust_prepared_query", 30))
        for objects, queries in (
            (100, 1_000),
            (1_000, 1_000),
            (10_000, 1_000),
            (25_000, 200),
            (50_000, 200),
            (100_000, 100),
        )
    ]
    captured = []

    def capture_figure(fig, _path_base):
        captured.append(fig)

    monkeypatch.setattr("tools.benchmark.plots._save_plot", capture_figure)
    _plot_scene_scaling_curves(tmp_path / "scene_scaling", rows)

    assert len(captured) == 1
    fig = captured[0]
    try:
        ax = fig.axes[0]
        data_lines = [line for line in ax.lines if len(line.get_xdata()) > 1 and len(set(line.get_xdata())) > 1]
        assert len(data_lines) == 4
        assert {tuple(line.get_xdata()) for line in data_lines} == {(100, 1_000, 10_000), (25_000, 50_000)}
        assert {line.get_linestyle() for line in data_lines} == {"-", "--"}
        assert any(list(line.get_xdata()) == [100_000] for line in ax.lines)
        assert {text.get_text() for text in ax.texts} >= {"q=1,000 to 200", "q=200 to 100"}
        legend_labels = {
            text.get_text()
            for legend in (*fig.legends, *(axis.get_legend() for axis in fig.axes if axis.get_legend()))
            for text in legend.get_texts()
        }
        assert {"Python end-to-end", "Prepared Rust query"} <= legend_labels
    finally:
        plt.close(fig)


def test_mode_comparisons_pair_repetitions_and_preserve_dimensions():
    runs = []
    for repetition, scalar_ns, batch_ns in ((0, 1_000, 500), (1, 1_200, 600)):
        for mode, total_ns in (("scalar", scalar_ns), ("batch_parallel", batch_ns)):
            runs.append(
                RunResult(
                    "dynamic_batch",
                    None,
                    "parry",
                    mode,
                    repetition,
                    10,
                    1,
                    None,
                    5,
                    0,
                    False,
                    total_ns,
                    [total_ns // 10],
                    api_mode=mode,
                    batch_size=10,
                    trajectory_steps=16,
                    time_window_steps=4,
                )
            )

    assert compare_modes(runs) == [
        {
            "schema_version": SCHEMA_VERSION,
            "feature": "dynamic_batch",
            "backend": "parry",
            "execution_layer": "python_end_to_end",
            "baseline_mode": "scalar",
            "candidate_mode": "batch_parallel",
            "batch_size": 10,
            "threads": 0,
            "trajectory_steps": 16,
            "time_window_steps": 4,
            "shape_variation": "fixed",
            "paired_repetitions": 2,
            "ratio_median": "2.000000",
            "ratio_q25": "2.000000",
            "ratio_q75": "2.000000",
            "ratio_ci_low": "2.000000",
            "ratio_ci_high": "2.000000",
            "verdict": "faster",
        }
    ]


def test_dynamic_workloads_cover_fixed_and_time_variant_shapes():
    fixed = dynamic_query_batch(3, 4)
    varying = time_variant_query_batch(3, 4, "primitive_switch")

    assert len(fixed) == len(varying) == 3
    assert all(type(obstacle).__name__ == "DynamicObstacle" for obstacle in (*fixed, *varying))


def test_planning_workload_contains_prepared_map_predictions_and_candidates():
    workload = planning_frame_workload(9, 3, 5, 4, 2026)

    assert len(workload.static_objects) == 9
    assert len(workload.predicted_obstacles) == 3
    assert len(workload.candidate_trajectories) == 5
    assert workload.trajectory_steps == 4
    assert all(type(obstacle).__name__ == "DynamicObstacle" for obstacle in workload.predicted_obstacles)
    assert all(type(obstacle).__name__ == "DynamicObstacle" for obstacle in workload.candidate_trajectories)


def test_planning_summary_preserves_cache_and_deadline_dimensions():
    results = [
        RunResult(
            "planning",
            None,
            backend,
            "static_100_dynamic_4_candidates_16_steps_8",
            repetition,
            16,
            104,
            None,
            8,
            0,
            False,
            total_ns,
            [total_ns],
            operation="planning_frame",
            api_mode="batch_parallel",
            batch_size=16,
            sample_semantics="per_frame",
            static_scene_objects=100,
            dynamic_scene_objects=4,
            trajectory_steps=8,
            deadline_ns=20_000_000,
            deadline_misses=int(total_ns > 20_000_000),
            cache_state=cache_state,
            candidate_count=16,
        )
        for backend in ("parry", "collide")
        for cache_state in ("warm", "cold")
        for repetition, total_ns in enumerate((10_000_000, 30_000_000))
    ]

    summary = summarize_runs(results)

    assert {(row["backend"], row["cache_state"]) for row in summary} == {
        ("parry", "warm"),
        ("parry", "cold"),
        ("collide", "warm"),
        ("collide", "cold"),
    }
    assert all(row["candidate_count"] == 16 for row in summary)
    assert all(row["deadline_misses"] == 1 for row in summary)
    assert all(row["deadline_miss_rate"] == "0.500000" for row in summary)


def test_planning_totals_are_normalized_by_complete_frames():
    result = RunResult(
        "planning",
        None,
        "parry",
        "static_100_dynamic_4_candidates_16_steps_8",
        0,
        2,
        104,
        None,
        16,
        0,
        False,
        30_000_000,
        [10_000_000, 20_000_000],
        operation="planning_frame",
        api_mode="batch_parallel",
        batch_size=16,
        sample_semantics="per_frame",
        static_scene_objects=100,
        dynamic_scene_objects=4,
        trajectory_steps=8,
        deadline_ns=20_000_000,
        deadline_misses=0,
        candidate_count=16,
    )

    row = run_row(result)

    assert row["queries"] == 2
    assert row["ns_per_query"] == "15000000.000"
    assert row["p50_ns"] == 10000000


def test_coverage_matrix_contains_every_shape_and_detection_mode():
    workloads = list(coverage_matrix_workloads(2))

    assert {(workload.shape_family, workload.ccd_mode) for workload in workloads} == {
        (shape, mode)
        for shape in ("circle", "rectangle", "polygon32", "compound16_polygon32")
        for mode in ("discrete", "stationary", "moving_static", "moving_moving")
    }
    assert all(len(workload.queries) == 2 for workload in workloads)


def test_scene_workloads_preserve_shape_dimensions():
    for family in ("circle", "rectangle", "polygon32", "compound16_polygon32"):
        workload = scene_workload(4, 3, 0.5, family)
        assert workload.shape_family == family
        assert workload.scene_mode == "static_static"
        assert workload.ccd_mode == "discrete"
        assert len(workload.static_objects) == 4
        assert len(workload.positioned_queries) == 3


def test_static_scene_queries_cover_the_scene_and_match_canonical_contract():
    objects, queries, density = 1_000, 128, 0.5
    workload = scene_workload(objects, queries, density, "circle")
    canonical = canonical_scene_workload(objects, queries, density, "circle")
    width = math.ceil(math.sqrt(objects))
    targets = [
        math.floor(pose.translation[1] / 6 + 0.5) * width + math.floor(pose.translation[0] / 6 + 0.5)
        for _, pose in workload.positioned_queries
    ]

    for (_, pose), query in zip(workload.positioned_queries, canonical["queries"], strict=True):
        assert (*pose.translation, pose.rotation) == pytest.approx(query["pose"])
    assert targets == [((2 * index + 1) * objects) // (2 * queries) for index in range(queries)]
    assert targets == sorted(set(targets))
    assert targets[0] < objects * 0.05
    assert targets[-1] >= objects * 0.95


def test_static_scene_density_matches_observed_hits_and_handles_small_samples():
    objects, queries, density = 64, 101, 0.5
    for family in ("circle", "rectangle", "polygon32", "compound16_polygon32"):
        workload = scene_workload(objects, queries, density, family)
        canonical = canonical_scene_workload(objects, queries, density, family)
        builder = CollisionCheckerBuilder()
        for obstacle in workload.static_objects:
            builder.add_static_obstacle(obstacle)
        checker = builder.build()
        observed = [checker.collides_static(shape, pose).collides for shape, pose in workload.positioned_queries]

        expected = [query["expected"] for query in canonical["queries"]]
        assert observed == expected
        assert all(left != right for left, right in pairwise(expected))
        assert sum(observed) == 51

        for boundary_density, expected_hits in ((0.0, 0), (1.0, queries)):
            boundary = scene_workload(objects, queries, boundary_density, family)
            actual = [checker.collides_static(shape, pose).collides for shape, pose in boundary.positioned_queries]
            assert sum(actual) == expected_hits

    empty = scene_workload(1, 0, 0.5, "circle")
    singleton = scene_workload(1, 1, 0.5, "circle")
    assert empty.positioned_queries == ()
    assert len(singleton.positioned_queries) == 1
    assert canonical_scene_workload(1, 0, 0.5, "circle")["queries"] == []
    assert canonical_scene_workload(1, 1, 0.5, "circle")["queries"][0]["expected"] is True


@pytest.mark.parametrize("dimensions", [(0, 0, 0.5), (1, -1, 0.5), (1, 1, -0.1), (1, 1, 1.1)])
def test_scene_workloads_reject_invalid_dimensions(dimensions):
    with pytest.raises(ValueError):
        scene_workload(*dimensions)
    with pytest.raises(ValueError):
        canonical_scene_workload(*dimensions)


@pytest.mark.parametrize(("backend", "engine"), ENGINE_ITEMS)
@pytest.mark.parametrize("shape_family", ("circle", "rectangle", "polygon32", "compound16_polygon32"))
def test_prepared_rust_scene_measurement_is_additive_and_matches_end_to_end(backend, engine, shape_family):
    workload = scene_workload(16, 9, 0.5, shape_family)
    builder = CollisionCheckerBuilder(backend=engine)
    for obstacle in workload.static_objects:
        builder.add_static_obstacle(obstacle)
    checker = builder.build()

    end_to_end = _measure_scene_with_checker(backend, workload, checker, 0)
    prepared = _measure_prepared_scene_with_checker(backend, workload, checker, 0)

    expected = [query["expected"] for query in canonical_scene_workload(16, 9, 0.5, shape_family)["queries"]]
    assert end_to_end.execution_layer == "python_end_to_end"
    assert end_to_end.collisions == sum(expected)
    assert prepared.execution_layer == "rust_prepared_query"
    assert prepared.queries == end_to_end.queries == len(expected)
    assert prepared.collisions == end_to_end.collisions
    assert prepared.errors == 0
    assert len(prepared.samples_ns) == prepared.queries
    assert all(sample >= 0 for sample in prepared.samples_ns)
    assert prepared.total_ns >= 0

    from crcc._core import benchmark as core_benchmark

    query = checker.prepare_static(workload.positioned_queries[0][0])
    native_collisions, native_samples, native_total, native_errors = core_benchmark.collides_static_prepared_timed(
        checker, query, [pose for _, pose in workload.positioned_queries]
    )
    assert native_collisions == expected
    assert native_errors == 0
    assert len(native_samples) == len(expected)
    assert all(sample >= 0 for sample in native_samples)
    assert native_total >= 0
    assert core_benchmark.collides_static_prepared_timed(checker, query, []) == ([], [], 0, 0)


def _layer_result(layer, repetition, total_ns):
    return RunResult(
        "native_layers",
        None,
        "parry",
        "circle_clear",
        repetition,
        10,
        None,
        None,
        0,
        0,
        False,
        total_ns,
        [total_ns // 10],
        execution_layer=layer,
        operation="discrete",
    )


def test_three_layer_comparison_reports_both_ratios():
    runs = [
        _layer_result(layer, repetition, total_ns)
        for repetition in range(2)
        for layer, total_ns in (
            ("engine_native", 100),
            ("rust_public_convert_and_query", 200),
            ("python_end_to_end", 600),
        )
    ]

    row = compare_layers(runs)[0]
    assert row["public_native_ratio_median"] == "2.000000"
    assert row["python_public_ratio_median"] == "3.000000"
    assert row["paired_repetitions"] == 2


def test_three_layer_comparison_rejects_unmatched_repetitions():
    runs = [
        _layer_result("engine_native", 0, 100),
        _layer_result("rust_public_convert_and_query", 0, 200),
    ]

    with pytest.raises(ValueError, match="unmatched execution-layer repetitions"):
        compare_layers(runs)
