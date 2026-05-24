import io
from contextlib import redirect_stdout
from pathlib import Path

from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Circle, Rectangle
from crcc.commonroad import add_road_boundary_to_builder
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose

import main


class CollisionResultStub:
    def __init__(self, collides):
        self.collides = collides


def test_parse_args_selects_action_scenario_and_engine():
    args = main.parse_args(
        [
            "benchmark",
            "--scenario",
            "scenarios/ZAM_Yield-1_1_T-1.xml",
            "--engine",
            "parry",
        ],
    )

    assert args.action == main.ExampleAction.BENCHMARK
    assert args.scenario == "scenarios/ZAM_Yield-1_1_T-1.xml"
    assert args.engine == CollisionEngine.Parry


def test_parse_args_selects_feature_examples():
    args = main.parse_args(["features"])

    assert args.action == main.ExampleAction.FEATURES


def test_feature_action_does_not_load_scenario(monkeypatch):
    called = []

    def fail_if_scenario_loads(*_args, **_kwargs):
        raise AssertionError("feature examples should not load a CommonRoad scenario")

    def record_feature_visualization(engine):
        called.append(engine)

    monkeypatch.setattr(main, "load_collision_checker", fail_if_scenario_loads)
    monkeypatch.setattr(main, "run_feature_visualization", record_feature_visualization)

    main.run_action(main.ExampleAction.FEATURES, "missing.xml", CollisionEngine.Parry)

    assert called == [CollisionEngine.Parry]


def test_prompt_for_action_accepts_numbered_selection():
    with redirect_stdout(io.StringIO()):
        action = main.prompt_for_action(prompt=lambda _: "4")

    assert action == main.ExampleAction.BENCHMARK


def test_python_builder_engine_selection_matches_for_simple_collision(engine):
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(1.0)).build()

    assert str(checker.collides_static(Circle(1.0, (1.0, 0.0)))) == "CollidesStatic"


def test_static_collision_time_window_filtering():
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


def test_half_bounded_time_range_queries():
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

    # With only min_time=1, it checks 1..=MAX (which is 1..=2). It should detect collision at step 1.
    assert str(checker.collides_static(query, min_time=1)) == "CollidesDynamic(1)"

    # With only max_time=1, it checks MIN..=1 (which is 0..=1). It should detect collision at step 0 (during 0 to 1 transition).
    assert str(checker.collides_static(query, max_time=1)) == "CollidesDynamic(0)"

    # With min_time=2, it checks 2..=MAX. There's no collision from step 2 onwards.
    assert str(checker.collides_static(query, min_time=2)) == "NoCollision"


def test_parallel_static_results_match_sequential_results():
    checker = CollisionCheckerBuilder().with_static_obstacle(Circle(2.0)).build()
    positioned_queries = [(Circle(1.0, (float(index), 0.0)), Pose.identity()) for index in range(8)]

    parallel_results = checker.par_collides_static(positioned_queries)
    sequential_results = [checker.collides_static(query, pose) for query, pose in positioned_queries]

    assert [str(result) for result in parallel_results] == [str(result) for result in sequential_results]


def test_bundled_scenarios_build_with_valid_pose_bounds():
    for scenario_path in sorted(Path("scenarios").glob("*.xml")):
        scenario, checker = main.load_collision_checker(str(scenario_path))
        lower_bounds, upper_bounds = main.scenario_pose_bounds(scenario)

        assert lower_bounds[0] < upper_bounds[0]
        assert lower_bounds[1] < upper_bounds[1]
        assert lower_bounds[2] < upper_bounds[2]

        car = main.Rectangle(*main.CAR_SIZE)
        pose = main.Pose(
            (
                (lower_bounds[0] + upper_bounds[0]) / 2.0,
                (lower_bounds[1] + upper_bounds[1]) / 2.0,
            ),
            0.0,
        )
        assert isinstance(checker.collides_static(car, pose).collides, bool)


def test_zam_yield_road_boundary_collision_matches_between_engines(engine):
    scenario, _ = main.CommonRoadFileReader("scenarios/ZAM_Yield-1_1_T-1.xml").open()
    checker = add_road_boundary_to_builder(
        CollisionCheckerBuilder(engine=engine),
        scenario.lanelet_network,
    ).build()

    assert checker.collides_static(
        Rectangle(*main.CAR_SIZE),
        Pose((62.013604720981206, -8.905038959453274), 0.8852293987803505),
    ).collides


def test_scenario_time_steps_are_ordered_and_non_empty():
    scenario, _ = main.CommonRoadFileReader("scenarios/ZAM_Yield-1_1_T-1.xml").open()

    time_steps = main.scenario_time_steps(scenario)

    assert len(time_steps) > 0
    assert time_steps == sorted(set(time_steps))
    assert time_steps[0] == 0
    assert time_steps[-1] == 80


def test_visualization_poses_are_deterministic():
    scenario, _ = main.CommonRoadFileReader(main.SCENARIO_PATH).open()
    pose_bounds = main.scenario_pose_bounds(scenario)

    first_poses = main.visualization_poses(main.SCENARIO_PATH, pose_bounds)
    second_poses = main.visualization_poses(main.SCENARIO_PATH, pose_bounds)

    assert len(first_poses) == main.VISUALIZATION_SAMPLE_COUNT
    assert [(pose.translation, pose.rotation) for pose in first_poses] == [
        (pose.translation, pose.rotation) for pose in second_poses
    ]


def test_visualization_poses_include_known_merge_samples_first():
    scenario, _ = main.CommonRoadFileReader(main.SCENARIO_PATH).open()
    pose_bounds = main.scenario_pose_bounds(scenario)

    poses = main.visualization_poses(main.SCENARIO_PATH, pose_bounds)

    assert poses[0].translation == (55.29, -1.99)
    assert poses[0].rotation == 1.326
    assert poses[1].translation == (37.33, 4.07)
    assert poses[1].rotation == -2.207


def test_visualization_poses_respect_count_limit():
    scenario, _ = main.CommonRoadFileReader(main.SCENARIO_PATH).open()
    pose_bounds = main.scenario_pose_bounds(scenario)

    poses = main.visualization_poses(main.SCENARIO_PATH, pose_bounds, count=1)

    assert len(poses) == 1
    assert poses[0].translation == (55.29, -1.99)


def test_collision_flags_persist_after_first_collision():
    collided_flags = [False, True, False]

    main.update_collision_flags(
        collided_flags,
        [CollisionResultStub(False), CollisionResultStub(False), CollisionResultStub(True)],
    )

    assert collided_flags == [False, True, True]


def test_fixed_dynamic_feature_example_is_deterministic(engine):
    example = main.feature_example_fixed_dynamic(engine)

    statuses = [
        example.checker.collides_dynamic(example.dynamic_obstacle, min_time=time_step, max_time=time_step)
        for time_step in example.time_steps
    ]
    collided_steps = [time_step for time_step, status in zip(example.time_steps, statuses, strict=True) if status.collides]

    assert example.title == "Fixed-shape dynamic obstacle"
    assert example.time_steps == main.FEATURE_TIME_STEPS
    assert len(example.dynamic_shapes) == len(main.FEATURE_TIME_STEPS)
    assert len(collided_steps) > 1
    assert min(collided_steps) > min(example.time_steps)
    assert max(collided_steps) < max(example.time_steps)
    assert example.time_steps[len(example.time_steps) // 2] in collided_steps


def test_time_variant_feature_example_is_deterministic(engine):
    example = main.feature_example_time_variant(engine)

    statuses = [
        example.checker.collides_dynamic(example.dynamic_obstacle, min_time=time_step, max_time=time_step)
        for time_step in example.time_steps
    ]
    collided_steps = [time_step for time_step, status in zip(example.time_steps, statuses, strict=True) if status.collides]

    assert example.title == "Time-variant dynamic obstacle"
    assert example.time_steps == main.FEATURE_TIME_STEPS
    assert len(example.dynamic_shapes) == len(main.FEATURE_TIME_STEPS)
    assert len(collided_steps) > 1
    assert min(collided_steps) > min(example.time_steps)
    assert max(collided_steps) < max(example.time_steps)
    assert example.time_steps[len(example.time_steps) // 2] in collided_steps


def test_feature_animation_frames_hold_each_time_step(engine):
    examples = [
        main.feature_example_fixed_dynamic(engine),
        main.feature_example_time_variant(engine),
    ]

    frames = main.feature_animation_frames(examples)

    assert frames == [
        time_step
        for time_step in main.FEATURE_TIME_STEPS
        for _ in range(main.FEATURE_FRAME_HOLD_COUNT)
    ]


def test_draw_feature_frame_returns_visible_artists(engine):
    example = main.feature_example_fixed_dynamic(engine)
    fig, ax = main.plt.subplots()

    try:
        artists = main.draw_feature_frame(ax, example, example.time_steps[len(example.time_steps) // 2])
    finally:
        main.plt.close(fig)

    assert len(artists) >= 5
    assert "COLLISION" in ax.get_title()


def test_interactive_playground_initialization_runs_without_crashing(monkeypatch):
    scenario, checker = main.load_collision_checker(main.SCENARIO_PATH)
    pose_bounds = main.scenario_pose_bounds(scenario)

    # Mock plt and Slider to avoid rendering / blocking
    class MockSlider:
        def __init__(self, *args, **kwargs):
            pass
        def on_changed(self, func):
            pass

    monkeypatch.setattr(main, "Slider", MockSlider)
    monkeypatch.setattr(main.plt, "subplots", lambda *args, **kwargs: (main.plt.figure(), main.plt.subplot(111)))
    monkeypatch.setattr(main.plt, "show", lambda: None)

    try:
        main.run_interactive_playground(scenario, checker, main.SCENARIO_PATH, pose_bounds)
    finally:
        main.plt.close("all")
