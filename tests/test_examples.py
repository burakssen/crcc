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
from examples import benchmark, features, interactive, utils as ex_utils, visualize


class CollisionResultStub:
    def __init__(self, collides):
        self.collides = collides


def test_parse_args_behavior():
    """Verify parsing CLI arguments yields correct actions, scenario paths, and engines."""
    args = main.parse_args(
        [
            "benchmark",
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
    assert args.action == main.ExampleAction.BENCHMARK
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

    args2 = main.parse_args(["smoke", "--engine", "collide"])
    assert args2.action == main.ExampleAction.SMOKE
    assert args2.engine == CollisionEngine.Collide
    assert args2.benchmark_scenarios == ["all"]

    args3 = main.parse_args(["plot"])
    assert args3.action == main.ExampleAction.PLOT


def test_prompt_for_action_parsing():
    """Verify interactive menus translate string inputs to ExampleAction enum instances."""
    with redirect_stdout(io.StringIO()):
        action = main.prompt_for_action(prompt=lambda _: "4")
    assert action == main.ExampleAction.BENCHMARK


def test_action_delegation_execution(monkeypatch):
    """Ensure run_action correctly delegates to modular sub-examples without scenario load overhead when possible."""
    called = []

    def fail_if_scenario_loads(*_args, **_kwargs):
        raise AssertionError("Features examples should not load scenario XML files.")

    monkeypatch.setattr(ex_utils, "load_collision_checker", fail_if_scenario_loads)
    monkeypatch.setattr(features, "run", lambda engine: called.append(engine))

    main.run_action(main.ExampleAction.FEATURES, "missing.xml", CollisionEngine.Parry)
    assert called == [CollisionEngine.Parry]


def test_checker_builder_engine_parity(engine):
    """Verify engine selection during check build performs as expected."""
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(1.0)).build()
    assert str(checker.collides_static(Circle(1.0, (1.0, 0.0)))) == "CollidesStatic"


def test_time_window_filtering_queries():
    """Ensure dynamic object queries can be filtered using time bounds."""
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

    # Time bounds filtering queries
    assert str(checker.collides_static(query, min_time=0, max_time=0)) == "NoCollision"
    assert str(checker.collides_static(query, min_time=1, max_time=1)) == "CollidesDynamic(1)"
    assert str(checker.collides_static(query, min_time=2, max_time=2)) == "NoCollision"
    assert str(checker.collides_static(query, min_time=0, max_time=2)) == "CollidesDynamic(0)"

    # Half-bounded range queries
    assert str(checker.collides_static(query, min_time=1)) == "CollidesDynamic(1)"
    assert str(checker.collides_static(query, max_time=1)) == "CollidesDynamic(0)"
    assert str(checker.collides_static(query, min_time=2)) == "NoCollision"


def test_parallel_vs_sequential_query_parity():
    """Assert parallel batch query results exactly match sequential check results."""
    checker = CollisionCheckerBuilder().with_static_obstacle(Circle(2.0)).build()
    positioned_queries = [(Circle(1.0, (float(index), 0.0)), Pose.identity()) for index in range(8)]

    parallel_results = checker.par_static(positioned_queries)
    sequential_results = [checker.collides_static(query, pose) for query, pose in positioned_queries]

    assert [str(result) for result in parallel_results] == [str(result) for result in sequential_results]


def test_benchmark_writes_csv_outputs(tmp_path, monkeypatch):
    monkeypatch.setattr(benchmark.runner, "DEFAULT_SCENE_SIZES", (10, 50))
    benchmark.run_all(
        ["scenarios/ZAM_Yield-1_1_T-1.xml", "scenarios/ZAM_Merge-1_1_T-1.xml"],
        sample_count=4,
        output_dir=tmp_path,
        seed=1,
        thread_counts=[1, 2],
        repetitions=1,
    )

    runs_path = tmp_path / "runs.csv"
    summary_path = tmp_path / "summary.csv"
    correctness_path = tmp_path / "correctness.csv"
    comparisons_path = tmp_path / "comparisons.csv"
    parallel_path = tmp_path / "parallel_scaling.csv"
    metadata_path = tmp_path / "metadata.json"
    plot_dir = tmp_path / "plots"
    assert runs_path.exists()
    assert summary_path.exists()
    assert correctness_path.exists()
    assert comparisons_path.exists()
    assert parallel_path.exists()
    assert metadata_path.exists()
    assert plot_dir.exists()

    with runs_path.open(newline="") as file:
        run_rows = list(csv.DictReader(file))
    assert {row["schema_version"] for row in run_rows} == {"2"}
    assert {"parry", "rhusics", "collide"} <= {row["backend"] for row in run_rows}
    assert {"pair", "scene_scaling", "ccd", "distance"} <= {row["feature"] for row in run_rows}
    unsupported_distance = {
        row["backend"] for row in run_rows if row["feature"] == "distance" and row["unsupported"] == "True"
    }
    assert unsupported_distance == set()

    with summary_path.open(newline="") as file:
        rows = list(csv.DictReader(file))
    scenario_rows = [row for row in rows if row["feature"] == "scenario"]
    assert {"parry", "rhusics", "collide"} == {row["backend"] for row in rows}
    assert {"ZAM_Yield-1_1_T-1", "ZAM_Merge-1_1_T-1"} == {row["scenario"] for row in scenario_rows}
    assert {row["workload"] for row in scenario_rows} == {"static_sequential", "static_parallel"}

    with correctness_path.open(newline="") as file:
        correctness_rows = list(csv.DictReader(file))
    robustness_rows = [
        row for row in correctness_rows if row["feature"] == "pair" and row["workload"] == "numerical_robustness"
    ]
    assert all(row["mismatches"] == "0" for row in robustness_rows)
    scenario_correctness = [row for row in correctness_rows if row["feature"] == "scenario"]
    assert {"parry", "rhusics", "collide"} == {row["backend"] for row in scenario_correctness}
    assert {"ZAM_Yield-1_1_T-1", "ZAM_Merge-1_1_T-1"} == {row["scenario"] for row in scenario_correctness}
    assert all(row["mismatches"] == "0" for row in scenario_correctness)

    with comparisons_path.open(newline="") as file:
        comparison_rows = list(csv.DictReader(file))
    assert comparison_rows
    assert {row["baseline_backend"] for row in comparison_rows} == {"parry"}
    assert {"rhusics", "collide"} <= {row["backend"] for row in comparison_rows}
    assert all(float(row["speedup_median"]) > 0.0 for row in comparison_rows)
    assert {row["verdict"] for row in comparison_rows} <= {"faster", "slower", "inconclusive"}

    with parallel_path.open(newline="") as file:
        parallel_rows = list(csv.DictReader(file))
    assert {"parry", "rhusics", "collide"} == {row["backend"] for row in parallel_rows}
    assert {"ZAM_Yield-1_1_T-1", "ZAM_Merge-1_1_T-1"} <= {row["scenario"] for row in parallel_rows}
    assert {"1", "2"} == {row["threads"] for row in parallel_rows}
    assert all(float(row["queries_per_s"]) > 0.0 for row in parallel_rows)
    assert all(float(row["speedup"]) > 0.0 for row in parallel_rows)

    expected_plots = {
        "backend_throughput_dotplot",
        "latency_tail_ratio",
        "scene_scaling_curves",
        "scenario_parallel_speedup_dotplot",
        "parallel_scaling_summary",
        "parallel_efficiency_summary",
        "correctness_summary",
        "throughput_variability_ratio",
        "backend_speedup_forest",
        "throughput_repetition_strip",
        "parallel_scene_scaling",
    }
    for plot_name in expected_plots:
        assert (plot_dir / f"{plot_name}.png").exists()
        assert (plot_dir / f"{plot_name}.pdf").exists()


def test_scenario_load_and_bounds():
    """Verify all xml scenarios can be parsed and produce valid pose bounds."""
    for scenario_path in sorted(Path("scenarios").glob("*.xml")):
        scenario, checker = ex_utils.load_collision_checker(str(scenario_path), CollisionEngine.Rhusics)
        lower_bounds, upper_bounds = ex_utils.scenario_pose_bounds(scenario)

        assert lower_bounds[0] < upper_bounds[0]
        assert lower_bounds[1] < upper_bounds[1]
        assert lower_bounds[2] < upper_bounds[2]

        car = Rectangle(*ex_utils.CAR_SIZE)
        pose = Pose(((lower_bounds[0] + upper_bounds[0]) / 2.0, (lower_bounds[1] + upper_bounds[1]) / 2.0), 0.0)
        assert isinstance(checker.collides_static(car, pose).collides, bool)


def test_road_boundary_collision_matches_between_engines(engine):
    """Ensure boundary collisions are consistent across all engines."""
    scenario, _ = ex_utils.CommonRoadFileReader("scenarios/ZAM_Yield-1_1_T-1.xml").open()
    checker = add_road_boundary_to_builder(
        CollisionCheckerBuilder(engine=engine),
        scenario.lanelet_network,
    ).build()

    assert checker.collides_static(
        Rectangle(*ex_utils.CAR_SIZE),
        Pose((62.013604720981206, -8.905038959453274), 0.8852293987803505),
    ).collides


def test_time_steps_computations():
    """Verify time step extraction yields correct values."""
    scenario, _ = ex_utils.CommonRoadFileReader("scenarios/ZAM_Yield-1_1_T-1.xml").open()
    time_steps = ex_utils.scenario_time_steps(scenario)

    assert len(time_steps) > 0
    assert time_steps == sorted(set(time_steps))
    assert time_steps[0] == 0
    assert time_steps[-1] == 80


def test_visualize_poses_generation():
    """Verify pose generation functions yield expected determinism and counts."""
    scenario_path = main.DEFAULT_SCENARIO_PATH
    scenario, _ = ex_utils.CommonRoadFileReader(scenario_path).open()
    pose_bounds = ex_utils.scenario_pose_bounds(scenario)

    poses1 = visualize.visualization_poses(scenario_path, pose_bounds)
    poses2 = visualize.visualization_poses(scenario_path, pose_bounds)

    assert len(poses1) == visualize.VISUALIZATION_SAMPLE_COUNT
    assert [(p.translation, p.rotation) for p in poses1] == [(p.translation, p.rotation) for p in poses2]

    # Verify preset poses are included first
    assert poses1[0].translation == (55.29, -1.99)
    assert poses1[0].rotation == 1.326
    assert poses1[1].translation == (37.33, 4.07)
    assert poses1[1].rotation == -2.207

    # Respect count limit
    limited_poses = visualize.visualization_poses(scenario_path, pose_bounds, count=1)
    assert len(limited_poses) == 1
    assert limited_poses[0].translation == (55.29, -1.99)


def test_visualize_collision_flag_persistency():
    """Assert cumulative collision flags remain True once set."""
    flags = [False, True, False]
    visualize.update_collision_flags(
        flags, [CollisionResultStub(False), CollisionResultStub(False), CollisionResultStub(True)]
    )
    assert flags == [False, True, True]


def test_features_fixed_dynamic_example(engine):
    """Verify fixed dynamic obstacle feature example behavior."""
    example = features.feature_example_fixed_dynamic(engine)
    statuses = [
        example.checker.collides_dynamic(example.dynamic_obstacle, min_time=t, max_time=t) for t in example.time_steps
    ]
    collided_steps = [t for t, status in zip(example.time_steps, statuses, strict=True) if status.collides]

    assert example.title == "Fixed-shape dynamic obstacle"
    assert example.time_steps == features.FEATURE_TIME_STEPS
    assert len(example.dynamic_shapes) == len(features.FEATURE_TIME_STEPS)
    assert len(collided_steps) > 1
    assert min(collided_steps) > min(example.time_steps)
    assert max(collided_steps) < max(example.time_steps)


def test_features_time_variant_example(engine):
    """Verify time variant obstacle feature example behavior."""
    example = features.feature_example_time_variant(engine)
    statuses = [
        example.checker.collides_dynamic(example.dynamic_obstacle, min_time=t, max_time=t) for t in example.time_steps
    ]
    collided_steps = [t for t, status in zip(example.time_steps, statuses, strict=True) if status.collides]

    assert example.title == "Time-variant dynamic obstacle"
    assert example.time_steps == features.FEATURE_TIME_STEPS
    assert len(example.dynamic_shapes) == len(features.FEATURE_TIME_STEPS)
    assert len(collided_steps) > 1


def test_features_animation_frames(engine):
    """Verify animation frames hold steps as expected."""
    examples = [features.feature_example_fixed_dynamic(engine), features.feature_example_time_variant(engine)]
    frames = features.feature_animation_frames(examples)
    assert frames == [t for t in features.FEATURE_TIME_STEPS for _ in range(features.FEATURE_FRAME_HOLD_COUNT)]


def test_features_draw_artists(engine):
    """Assert drawing frames return valid Matplotlib artist lists."""
    example = features.feature_example_fixed_dynamic(engine)
    fig, ax = plt.subplots()
    try:
        artists = features.draw_feature_frame(ax, example, example.time_steps[len(example.time_steps) // 2])
    finally:
        plt.close(fig)

    assert len(artists) >= 5
    assert "COLLISION" in ax.get_title()


def test_interactive_playground_initialization_runs_without_crashing(monkeypatch):
    """Ensure the interactive playground GUI is initialization-safe (mocked to run headlessly)."""
    scenario, checker = ex_utils.load_collision_checker(main.DEFAULT_SCENARIO_PATH, CollisionEngine.Rhusics)
    pose_bounds = ex_utils.scenario_pose_bounds(scenario)

    class MockSlider:
        def __init__(self, *args, **kwargs):
            pass

        def on_changed(self, func):
            pass

    monkeypatch.setattr(interactive, "Slider", MockSlider)
    monkeypatch.setattr(plt, "subplots", lambda *args, **kwargs: (plt.figure(), plt.subplot(111)))
    monkeypatch.setattr(plt, "show", lambda: None)

    try:
        interactive.run(scenario, checker, main.DEFAULT_SCENARIO_PATH, pose_bounds)
    finally:
        plt.close("all")
