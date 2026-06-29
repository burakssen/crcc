import csv
import io
from contextlib import redirect_stdout
from pathlib import Path

from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Circle, Rectangle
from crcc.commonroad import add_road_boundary_to_builder
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose
from matplotlib import pyplot as plt

import main
from examples import benchmark, concepts, dynamics, playground, scenario as scenario_example, shapes, utils as ex_utils
from examples.drawing import demo_shapes


class CollisionResultStub:
    def __init__(self, collides):
        self.collides = collides


def test_parse_args_behavior():
    args = main.parse_args(
        [
            "study",
            "--scenario",
            "scenarios/ZAM_Yield-1_1_T-1.xml",
            "--engine",
            "parry",
            "--benchmark-scenarios",
            "scenarios/ZAM_Yield-1_1_T-1.xml",
            "scenarios/ZAM_Merge-1_1_T-1.xml",
            "--benchmark-thread-counts",
            "1",
            "2",
            "--benchmark-repetitions",
            "3",
            "--benchmark-step",
            "run",
            "--benchmark-engines",
            "parry",
            "rhusics",
        ]
    )
    assert args.action == main.ExampleAction.STUDY
    assert args.scenario == "scenarios/ZAM_Yield-1_1_T-1.xml"
    assert args.engine == CollisionEngine.Parry
    assert args.benchmark_scenarios == [
        "scenarios/ZAM_Yield-1_1_T-1.xml",
        "scenarios/ZAM_Merge-1_1_T-1.xml",
    ]
    assert args.benchmark_thread_counts == [1, 2]
    assert args.benchmark_repetitions == 3
    assert args.benchmark_step == "run"
    assert args.benchmark_engines == ["parry", "rhusics"]

    assert main.parse_args(["scenario", "--engine", "collide"]).action == main.ExampleAction.SCENARIO
    assert main.parse_args(["report"]).action == main.ExampleAction.REPORT
    assert main.parse_args(["shapes"]).action == main.ExampleAction.SHAPES


def test_old_cli_action_names_are_removed():
    for action in ("geometry", "features", "smoke", "benchmark", "plot", "visualize", "interactive"):
        try:
            main.parse_args([action])
        except SystemExit:
            continue
        raise AssertionError(f"old action still accepted: {action}")


def test_prompt_for_action_parsing():
    with redirect_stdout(io.StringIO()):
        action = main.prompt_for_action(prompt=lambda _: "6")
    assert action == main.ExampleAction.STUDY


def test_action_delegation_execution(monkeypatch):
    called = []

    def fail_if_scenario_loads(*_args, **_kwargs):
        raise AssertionError("Concept examples should not load scenario XML files.")

    monkeypatch.setattr(main, "load_collision_checker", fail_if_scenario_loads)
    monkeypatch.setattr(concepts, "run", lambda engine: called.append(engine))

    main.run_action(main.ExampleAction.CONCEPTS, "missing.xml", CollisionEngine.Parry)
    assert called == [CollisionEngine.Parry]


def test_concepts_results(engine):
    results = concepts.concept_results(engine)
    assert results["near_static_collision"] is True
    assert results["far_static_collision"] is False
    assert results["distance_far"] > 0.0
    assert results["polygon_hole_collision"] is False
    assert results["compound_collision"] is True
    assert results["continuous_tunnel_collision"] is True


def test_shapes_collision_matrix_and_drawing(engine):
    labels, matrix = shapes.collision_matrix(engine)
    assert {"Circle", "Rectangle", "Triangle", "Polygon", "Polygon+hole", "Compound", "HalfSpace", "FullSpace", "Empty"} <= set(labels)
    assert len(matrix) == len(labels)
    assert any(any(row) for row in matrix)
    empty_index = labels.index("Empty")
    assert not any(matrix[empty_index])
    left_center, right_center = shapes.pair_display_centers(
        demo_shapes()[0],
        demo_shapes()[1],
        True,
    )
    assert abs(left_center[0] - right_center[0]) < 0.25
    cases = shapes.pair_collision_cases(engine)
    circle_rectangle = next(
        case for case in cases if case["left"].label == "Circle" and case["right"].label == "Rectangle"
    )
    assert circle_rectangle["hit"]["supported"]
    assert circle_rectangle["clear"]["supported"]
    assert circle_rectangle["hit"]["actual"] is True
    assert circle_rectangle["clear"]["actual"] is False

    fig, axes, artists = shapes.draw_collision_matrix(engine)
    try:
        assert artists
        assert len(fig.axes) == len(labels) * len(labels)
        assert axes[0][0].get_title() == labels[0]
    finally:
        plt.close(fig)


def test_checker_builder_engine_parity(engine):
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(1.0)).build()
    assert str(checker.collides_static(Circle(1.0, (1.0, 0.0)))) == "CollidesStatic"


def test_time_window_filtering_queries():
    dynamic_obstacle = DynamicObstacle(
        Circle(1.0),
        [
            Pose.from_translation((10.0, 10.0)),
            Pose.from_translation((9.0, 9.0)),
            Pose.from_translation((10.0, 10.0)),
        ],
        0,
    )
    checker = CollisionCheckerBuilder().with_dynamic_obstacle(dynamic_obstacle).build()
    query = Circle(1.0, (8.0, 8.0))

    assert str(checker.collides_static(query, min_time=0, max_time=0)) == "NoCollision"
    assert str(checker.collides_static(query, min_time=1, max_time=1)) == "CollidesDynamic(1)"
    assert str(checker.collides_static(query, min_time=2, max_time=2)) == "NoCollision"
    assert str(checker.collides_static(query, min_time=0, max_time=2)) == "CollidesDynamic(0)"
    assert str(checker.collides_static(query, min_time=1)) == "CollidesDynamic(1)"
    assert str(checker.collides_static(query, max_time=1)) == "CollidesDynamic(0)"
    assert str(checker.collides_static(query, min_time=2)) == "NoCollision"


def test_parallel_vs_sequential_query_parity():
    checker = CollisionCheckerBuilder().with_static_obstacle(Circle(2.0)).build()
    positioned_queries = [(Circle(1.0, (float(index), 0.0)), Pose.identity()) for index in range(8)]
    parallel_results = checker.par_static(positioned_queries)
    sequential_results = [checker.collides_static(query, pose) for query, pose in positioned_queries]
    assert [str(result) for result in parallel_results] == [str(result) for result in sequential_results]


def test_benchmark_writes_research_artifacts(tmp_path, monkeypatch):
    monkeypatch.setattr(benchmark.runner, "DEFAULT_SCENE_SIZES", (10, 50))
    monkeypatch.setattr(benchmark.runner, "DEFAULT_DENSITIES", (0.0, 0.5, 1.0))
    benchmark.run_all(
        ["scenarios/ZAM_Yield-1_1_T-1.xml", "scenarios/ZAM_Merge-1_1_T-1.xml"],
        sample_count=4,
        output_dir=tmp_path,
        seed=1,
        thread_counts=[1, 2],
        repetitions=1,
    )

    expected_files = {
        "runs.csv",
        "summary.csv",
        "correctness.csv",
        "comparisons.csv",
        "parallel_scaling.csv",
        "metadata.json",
        "benchmark_report.md",
    }
    assert expected_files <= {path.name for path in tmp_path.iterdir()}

    with (tmp_path / "runs.csv").open(newline="") as file:
        run_rows = list(csv.DictReader(file))
    assert {row["schema_version"] for row in run_rows} == {"3"}
    assert {"parry", "rhusics", "collide"} <= {row["backend"] for row in run_rows}
    assert {"pair", "scene_scaling", "continuous", "distance"} <= {row["feature"] for row in run_rows}
    assert "compound_polygon" in {row["workload"] for row in run_rows}
    assert "tunneling" in {row["workload"] for row in run_rows}

    with (tmp_path / "correctness.csv").open(newline="") as file:
        correctness_rows = list(csv.DictReader(file))
    assert any(row["feature"] == "continuous" for row in correctness_rows)
    assert all(row["schema_version"] == "3" for row in correctness_rows)

    with (tmp_path / "comparisons.csv").open(newline="") as file:
        comparison_rows = list(csv.DictReader(file))
    assert comparison_rows
    assert {row["baseline_backend"] for row in comparison_rows} == {"parry"}
    assert {row["verdict"] for row in comparison_rows} <= {"faster", "slower", "inconclusive"}

    with (tmp_path / "parallel_scaling.csv").open(newline="") as file:
        parallel_rows = list(csv.DictReader(file))
    assert {"1", "2"} == {row["threads"] for row in parallel_rows}
    assert all(row["schema_version"] == "3" for row in parallel_rows)

    report = (tmp_path / "benchmark_report.md").read_text()
    assert "# CRCC Benchmark Report" in report
    assert "bootstrap confidence intervals" in report

    expected_plots = {
        "backend_throughput_iqr",
        "backend_speedup_forest",
        "latency_percentiles",
        "scene_scaling_curves",
        "parallel_scaling_summary",
        "parallel_efficiency_summary",
        "commonroad_scenario_summary",
        "correctness_mismatch_matrix",
    }
    for plot_name in expected_plots:
        assert (tmp_path / "plots" / f"{plot_name}.png").exists()
        assert (tmp_path / "plots" / f"{plot_name}.pdf").exists()


def test_scenario_load_and_bounds():
    for scenario_path in sorted(Path("scenarios").glob("*.xml")):
        scenario, checker = ex_utils.load_collision_checker(str(scenario_path), CollisionEngine.Rhusics)
        lower_bounds, upper_bounds = ex_utils.scenario_pose_bounds(scenario)
        assert lower_bounds[0] < upper_bounds[0]
        assert lower_bounds[1] < upper_bounds[1]
        assert lower_bounds[2] < upper_bounds[2]
        pose = Pose(((lower_bounds[0] + upper_bounds[0]) / 2.0, (lower_bounds[1] + upper_bounds[1]) / 2.0), 0.0)
        assert isinstance(checker.collides_static(Rectangle(*ex_utils.CAR_SIZE), pose).collides, bool)


def test_road_boundary_collision_matches_between_engines(engine):
    scenario, _ = ex_utils.CommonRoadFileReader("scenarios/ZAM_Yield-1_1_T-1.xml").open()
    checker = add_road_boundary_to_builder(CollisionCheckerBuilder(engine=engine), scenario.lanelet_network).build()
    assert checker.collides_static(
        Rectangle(*ex_utils.CAR_SIZE),
        Pose((62.013604720981206, -8.905038959453274), 0.8852293987803505),
    ).collides


def test_time_steps_computations():
    scenario, _ = ex_utils.CommonRoadFileReader("scenarios/ZAM_Yield-1_1_T-1.xml").open()
    time_steps = ex_utils.scenario_time_steps(scenario)
    assert time_steps == sorted(set(time_steps))
    assert time_steps[0] == 0
    assert time_steps[-1] == 80


def test_scenario_audit():
    scenario, checker = ex_utils.load_collision_checker(main.DEFAULT_SCENARIO_PATH, CollisionEngine.Rhusics)
    pose_bounds = ex_utils.scenario_pose_bounds(scenario)
    audit = scenario_example.scenario_audit(scenario, checker, main.DEFAULT_SCENARIO_PATH, pose_bounds, sample_count=8)
    assert audit["scenario"] == Path(main.DEFAULT_SCENARIO_PATH).name
    assert audit["lanelets"] > 0
    assert audit["sample_count"] == 8
    assert {"road_boundary", "dynamic_conflict"} <= set(audit["probes"])


def test_dynamics_scenario_example_and_drawing():
    scenario, checker = ex_utils.load_collision_checker(main.DEFAULT_SCENARIO_PATH, CollisionEngine.Rhusics)
    pose_bounds = ex_utils.scenario_pose_bounds(scenario)
    example = dynamics.scenario_dynamic_example(scenario, checker, pose_bounds, max_steps=8)
    statuses = [example.checker.collides_dynamic(example.dynamic_obstacle, min_time=t, max_time=t) for t in example.time_steps]
    assert len(example.poses) == 8
    assert len(example.visual_shapes) == 8
    assert {status.collides for status in statuses} <= {True, False}
    frames = dynamics.animation_frames(example)
    assert frames == [t for t in example.time_steps for _ in range(dynamics.FRAME_HOLD_COUNT)]
    fig, ax = plt.subplots()
    try:
        artists = dynamics.draw_frame(ax, scenario, example, example.time_steps[len(example.time_steps) // 2])
    finally:
        plt.close(fig)
    assert artists
    assert "Scenario time-variant" in ax.get_title()


def test_playground_editor_state():
    state = playground.PlaygroundState(CollisionEngine.Rhusics, tuple(range(4)))
    static = state.add_object((0.0, 0.0))
    assert static.mode == "static"
    state.mode = "dynamic"
    state.add_path_point((0.0, 0.0))
    state.add_path_point((1.0, 0.0))
    dynamic = state.add_object((0.0, 0.0))
    assert dynamic.dynamic_obstacle(0) is not None
    state.mode = "time_variant_dynamic"
    state.add_path_point((0.0, 0.0))
    state.add_path_point((1.0, 0.0))
    variant = state.add_object((0.0, 0.0))
    assert variant.dynamic_obstacle(0) is not None
    state.add_freehand_vertex((0.0, 0.0))
    state.add_freehand_vertex((1.0, 0.0))
    assert state.finalize_freehand() is None
    state.add_freehand_vertex((0.0, 1.0))
    assert state.finalize_freehand() is not None
    assert state.select_next() is not None
    assert " | " in state.status_summary()
    state.add_path_point((2.0, 0.0))
    state.add_freehand_vertex((2.0, 1.0))
    state.clear_draft()
    assert state.draft_path == []
    assert state.draft_polygon == []
    before = state.time_index
    state.step_simulation()
    assert state.time_index == before + 1
    assert state.last_results
    assert state.toggle_simulation() is True
    assert state.toggle_simulation() is False


def test_playground_draws_objects_above_scenario():
    scenario, checker = ex_utils.load_collision_checker(main.DEFAULT_SCENARIO_PATH, CollisionEngine.Rhusics)
    pose_bounds = ex_utils.scenario_pose_bounds(scenario)
    plot_limits = (pose_bounds[0][0], pose_bounds[1][0], pose_bounds[0][1], pose_bounds[1][1])
    state = playground.PlaygroundState(checker.engine, tuple(ex_utils.scenario_time_steps(scenario)[:4]))
    state.add_object(((plot_limits[0] + plot_limits[1]) / 2.0, (plot_limits[2] + plot_limits[3]) / 2.0))
    fig, ax = plt.subplots()
    try:
        artists = playground.draw_playground(ax, scenario, state, plot_limits)
        assert artists
        assert max(getattr(artist, "zorder", 0) for artist in artists) >= 50
    finally:
        plt.close(fig)


def test_playground_initialization_runs_without_crashing(monkeypatch):
    scenario, checker = ex_utils.load_collision_checker(main.DEFAULT_SCENARIO_PATH, CollisionEngine.Rhusics)
    pose_bounds = ex_utils.scenario_pose_bounds(scenario)

    class MockSlider:
        def __init__(self, *args, **kwargs):
            pass

        def on_changed(self, func):
            pass

        def on_clicked(self, func):
            pass

    monkeypatch.setattr(playground, "Slider", MockSlider)
    monkeypatch.setattr(playground, "RadioButtons", MockSlider)
    monkeypatch.setattr(playground, "TextBox", lambda *args, **kwargs: type("Box", (), {"text": "object"})())
    monkeypatch.setattr(playground, "Button", MockSlider)
    monkeypatch.setattr(plt, "subplots", lambda *args, **kwargs: (plt.figure(), plt.subplot(111)))
    monkeypatch.setattr(plt, "show", lambda: None)
    try:
        playground.run(scenario, checker, main.DEFAULT_SCENARIO_PATH, pose_bounds)
    finally:
        plt.close("all")
