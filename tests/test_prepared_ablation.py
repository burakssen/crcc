import csv
import importlib
import importlib.util
import json

import pytest


def _module():
    return importlib.import_module("tools.benchmark.prepared_ablation")


def test_prepared_ablation_module_exists():
    assert importlib.util.find_spec("tools.benchmark.prepared_ablation") is not None


def test_ablation_config_rejects_invalid_sweep_and_repetition_counts(tmp_path):
    AblationConfig = _module().AblationConfig

    with pytest.raises(ValueError, match="reuse counts"):
        AblationConfig(output_dir=tmp_path / "bad-sweep", reuse_counts=(1, 2, 2))

    with pytest.raises(ValueError, match="repetitions"):
        AblationConfig(output_dir=tmp_path / "bad-repetitions", repetitions=0)


def test_prepared_ablation_records_matched_static_and_dynamic_runs(tmp_path):
    AblationConfig = _module().AblationConfig
    run_ablation = _module().run_ablation

    output_dir = tmp_path / "prepared-ablation"
    config = AblationConfig(
        output_dir=output_dir,
        engines=("parry",),
        repetitions=2,
        reuse_counts=(1, 2, 4),
        warmup_queries=1,
        preparation_samples=2,
    )

    result_dir = run_ablation(config)

    with (result_dir / "runs.csv").open(newline="") as file:
        runs = list(csv.DictReader(file))
    with (result_dir / "summary.csv").open(newline="") as file:
        summary = list(csv.DictReader(file))
    metadata = json.loads((result_dir / "metadata.json").read_text())

    assert result_dir == output_dir
    assert len(runs) == 2 * 3 * 2
    assert {row["operation"] for row in runs} == {"static", "dynamic"}
    assert {int(row["reuse_count"]) for row in runs} == {1, 2, 4}
    assert {row["backend"] for row in runs} == {"parry"}
    assert {row["raw_result"] for row in runs} == {row["prepared_result"] for row in runs}
    assert all(int(row["preparation_ns"]) >= 0 for row in runs)
    assert all(int(row["raw_query_total_ns"]) > 0 for row in runs)
    assert all(int(row["prepared_query_total_ns"]) > 0 for row in runs)
    assert all(
        int(row["prepared_amortized_total_ns"]) == int(row["preparation_ns"]) + int(row["prepared_query_total_ns"])
        for row in runs
    )
    assert len(summary) == 2 * 3
    for operation in ("static", "dynamic"):
        for reuse_count in (1, 2, 4):
            paired = [row for row in runs if row["operation"] == operation and int(row["reuse_count"]) == reuse_count]
            assert {row["query_order"] for row in paired} == {"raw_first", "prepared_first"}

    assert metadata["study"] == "prepared_query_ablation"
    assert metadata["command"]["reuse_counts"] == [1, 2, 4]
    assert metadata["environment"]["python"]
    assert metadata["git"]["source_sha256"]

    for plot in ("preparation_cost", "reuse_latency", "reuse_speedup"):
        assert (result_dir / "plots" / f"{plot}.pdf").read_bytes().startswith(b"%PDF-")


def test_prepared_ablation_plots_use_the_thesis_backend_palette(tmp_path):
    module = _module()
    output_dir = tmp_path / "styled-plots"
    output_dir.mkdir()
    rows = [
        {
            "study_schema_version": "1",
            "operation": operation,
            "backend": backend,
            "reuse_count": reuse_count,
            "preparation_ns_median": 100,
            "raw_ns_per_query_median": 200 / reuse_count,
            "prepared_amortized_ns_per_query_median": 150 / reuse_count,
            "speedup_median": 1.5,
        }
        for operation in ("static", "dynamic")
        for backend in ("parry", "rhusics", "collide")
        for reuse_count in (1, 2)
    ]
    with (output_dir / "summary.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    module.write_ablation_plots(output_dir)

    for plot in ("preparation_cost", "reuse_latency", "reuse_speedup"):
        svg = (output_dir / "plots" / f"{plot}.svg").read_text().lower()
        assert all(color in svg for color in ("#0072b2", "#d55e00", "#009e73"))
        assert 'width="421.2pt"' in svg
    latency_svg = (output_dir / "plots" / "reuse_latency.svg").read_text()
    assert "#D9DEE5" in latency_svg or "#d9dee5" in latency_svg
    assert "stroke-dasharray" in latency_svg


def test_ablation_summary_only_reports_observed_sweep_crossing():
    AblationRun = _module().AblationRun
    _summarize_runs = _module()._summarize_runs

    runs = []
    for repetition in range(2):
        for reuse_count, raw_total, prepared_queries in (
            (1, 100, 80),
            (2, 200, 160),
            (4, 400, 320),
        ):
            runs.append(
                AblationRun(
                    operation="static",
                    backend="parry",
                    repetition=repetition,
                    reuse_count=reuse_count,
                    preparation_ns=50,
                    raw_query_total_ns=raw_total,
                    prepared_query_total_ns=prepared_queries,
                    raw_result="(True, None)",
                    prepared_result="(True, None)",
                    query_order="raw_first",
                    trajectory_steps=0,
                )
            )

    summary = _summarize_runs(runs)

    assert [row["reuse_count"] for row in summary] == [1, 2, 4]
    assert {row["first_prepared_faster_reuse_count"] for row in summary} == {4}
    assert {row["last_raw_faster_tested_reuse_count"] for row in summary} == {2}
