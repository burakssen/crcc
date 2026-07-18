import io
from contextlib import redirect_stdout
from pathlib import Path

from crcc import CollisionEngine
from matplotlib import pyplot as plt

import main
from examples import basic, commonroad, continuous, utils
from tools import playground


def test_cli_exposes_clean_and_legacy_example_names():
    assert main.parse_args(["basic", "--engine", "parry"]).action == main.ExampleAction.BASIC
    assert main.parse_args(["continuous"]).action == main.ExampleAction.CONTINUOUS
    assert main.parse_args(["commonroad"]).action == main.ExampleAction.COMMONROAD
    assert main.parse_args(["playground"]).action == main.ExampleAction.PLAYGROUND
    assert main.parse_args(["study"]).action == main.ExampleAction.STUDY
    assert main.parse_args(["report"]).action == main.ExampleAction.REPORT
    assert main.parse_args(["concepts"]).action == main.ExampleAction.CONCEPTS
    assert main.parse_args(["shapes"]).action == main.ExampleAction.SHAPES
    assert main.parse_args(["dynamics"]).action == main.ExampleAction.DYNAMICS
    assert main.parse_args(["scenario"]).action == main.ExampleAction.SCENARIO
    assert main.parse_args(["all"]).action == main.ExampleAction.ALL


def test_interactive_menu_uses_new_order():
    with redirect_stdout(io.StringIO()):
        assert main.prompt_for_action(prompt=lambda _: "2") == main.ExampleAction.CONTINUOUS
        assert main.prompt_for_action(prompt=lambda _: "commonroad") == main.ExampleAction.COMMONROAD


def test_basic_tutorial_is_deterministic_and_preserves_unsupported(engine):
    results = basic.basic_results(engine)
    outcomes = {name: outcome for name, outcome, _detail in results}
    assert outcomes["overlapping primitives"] == "hit"
    assert outcomes["boundary contact"] in {"hit", "clear"}
    assert outcomes["separated primitives"] == "clear"
    assert outcomes["polygon hole"] == "clear"
    assert outcomes["compound child"] == "hit"
    assert set(outcomes) == {
        "overlapping primitives",
        "boundary contact",
        "separated primitives",
        "polygon hole",
        "compound child",
        "triangle vs circle",
        "half space",
        "full space",
        "empty space",
        "separation distance",
        "pose composition",
    }
    assert outcomes["empty space"] == "clear"
    assert set(outcomes.values()) <= {"hit", "clear", "unsupported"}


def test_continuous_tutorial_distinguishes_endpoints_from_sweep(engine):
    results = continuous.continuous_results(engine)
    outcomes = {name: outcome for name, outcome, _detail in results}
    assert outcomes["translation start"] == "clear"
    assert outcomes["translation end"] == "clear"
    assert outcomes["translation interval"] in {"potential collision", "unsupported"}
    assert outcomes["rotation interval"] in {
        "potential collision",
        "certified clear",
        "unsupported",
    }


def test_playground_uses_the_exact_sweep_shown(engine):
    state = playground.InspectorState()
    results = playground.inspect(state, engine)
    assert results[0][1] == "clear"
    assert results[1][1] == "clear"
    assert results[2][1] in {"potential collision", "unsupported"}

    fig, ax = plt.subplots()
    try:
        assert playground.draw_inspector(ax, state, engine) == results
        assert ax.get_title()
    finally:
        plt.close(fig)


def test_playground_scene_supports_all_occupancy_modes(engine):
    state = playground.PlaygroundState(engine, (0, 1, 2, 3))
    state.shape_kind = "rectangle"
    state.mode = "static"
    environment = state.add_object((0.0, 0.0), role="environment")
    assert environment.role == "environment"

    for mode in ("dynamic", "time_variant", "time_variant_dynamic"):
        state.mode = mode
        state.shape_kind = "circle"
        state.draft_path = [playground.Pose.from_translation((-3.0, 0.0)), playground.Pose.from_translation((3.0, 0.0))]
        query = state.add_object((-3.0, 0.0))
        assert query.mode == mode
        if mode != "dynamic":
            assert query.variants
    assert state.evaluate()
    state.set_engine(engine)
    assert state.step() == 1


def test_playground_presets_color_every_collision_participant(engine):
    bounds = (-10.0, 10.0, -10.0, 10.0)
    for preset in ("Tunneling", "Intersection", "Overtaking"):
        state = playground.PlaygroundState(engine, (0, 1, 2))
        state.load_preset(preset, bounds)
        assert set(state._results) == {obj.object_id for obj in state.objects}
        assert {result.verdict for result in state._results.values()} == {playground.Verdict.POTENTIAL_COLLISION}

    state = playground.PlaygroundState(engine, (0, 1))
    state.shape_kind, state.mode = "rectangle", "static"
    colliding = state.add_object((0.0, 0.0), role="environment")
    separated = state.add_object((20.0, 0.0), role="environment")
    state.shape_kind, state.mode = "circle", "dynamic"
    state.draft_path = [playground.Pose.from_translation((-8.0, 0.0)), playground.Pose.from_translation((8.0, 0.0))]
    query = state.add_object((-8.0, 0.0))
    results = state.evaluate()

    assert results[query.object_id].verdict == playground.Verdict.POTENTIAL_COLLISION
    assert results[colliding.object_id].verdict == playground.Verdict.POTENTIAL_COLLISION
    assert results[separated.object_id].verdict == playground.Verdict.CERTIFIED_CLEAR


def test_playground_freehand_polygon_uses_local_coordinates():
    shape, center = playground.normalized_polygon([(10.0, 10.0), (12.0, 10.0), (11.0, 12.0)])
    assert center == (11.0, 10.666666666666666)
    assert shape.kind == "polygon"
    xs = [point[0] for point in shape.values[0][:-1]]
    ys = [point[1] for point in shape.values[0][:-1]]
    assert abs(sum(xs)) < 1e-12
    assert abs(sum(ys)) < 1e-12


def test_playground_paths_do_not_obscure_scene_and_zoom_is_preserved(engine):
    state = playground.PlaygroundState(engine, (0, 1, 2, 3))
    state.mode = "dynamic"
    state.draft_path = [playground.Pose.from_translation((-3.0, 0.0)), playground.Pose.from_translation((3.0, 0.0))]
    state.add_object((-3.0, 0.0))
    state.draft_path = [playground.Pose.from_translation((-1.0, 1.0)), playground.Pose.from_translation((1.0, 1.0))]

    fig, ax = plt.subplots()
    bounds = (-10.0, 10.0, -5.0, 5.0)
    try:
        artists = playground.draw_scene(ax, None, state, bounds, reset_view=True)
        path = next(
            artist for artist in artists if getattr(artist, "get_color", lambda: None)() == playground.COLOR_PATH
        )
        assert path.get_marker() == "None"

        draft = [artist for artist in artists if getattr(artist, "get_color", lambda: None)() == playground.COLOR_PATH][
            -1
        ]
        assert draft.get_marker() == "o"
        assert draft.get_markerfacecolor() == "none"

        ax.set_xlim(-2.0, 2.0)
        ax.set_ylim(-1.5, 1.5)
        playground.draw_scene(ax, None, state, bounds)
        assert ax.get_xlim() == (-2.0, 2.0)
        assert ax.get_ylim() == (-1.5, 1.5)

        playground.draw_scene(ax, None, state, bounds, reset_view=True)
        assert ax.get_xlim() == (-10.0, 10.0)
        assert ax.get_ylim() == (-5.0, 5.0)
    finally:
        plt.close(fig)


def test_commonroad_probes_are_geometry_derived_and_repeatable():
    scenario, checker = utils.load_collision_checker(main.DEFAULT_SCENARIO_PATH, CollisionEngine.Rhusics)
    bounds = utils.scenario_pose_bounds(scenario)
    first = commonroad.deterministic_probes(scenario, bounds)
    second = commonroad.deterministic_probes(scenario, bounds)
    assert [(name, pose.translation, pose.rotation) for name, pose in first] == [
        (name, pose.translation, pose.rotation) for name, pose in second
    ]
    assert [name for name, _pose in first] == ["first lanelet centroid", "outside road bounds"]
    assert commonroad.commonroad_results(scenario, checker, bounds) == commonroad.commonroad_results(
        scenario, checker, bounds
    )


def test_all_bundled_scenarios_load():
    for scenario_path in Path("scenarios").glob("*.xml"):
        scenario, checker = utils.load_collision_checker(str(scenario_path), CollisionEngine.Rhusics)
        lower, upper = utils.scenario_pose_bounds(scenario)
        assert lower[0] < upper[0]
        assert checker.engine == CollisionEngine.Rhusics


def test_playground_unselect_click_outside(monkeypatch, engine):
    monkeypatch.setattr(plt, "show", lambda: None)
    scenario, checker = utils.load_collision_checker(main.DEFAULT_SCENARIO_PATH, engine)
    bounds = utils.scenario_pose_bounds(scenario)
    pose_bounds = bounds

    state = playground.run(scenario, checker, main.DEFAULT_SCENARIO_PATH, pose_bounds)
    try:
        assert state.selected_id is not None
        selected_obj = state.selected
        assert selected_obj is not None

        fig = plt.gcf()
        ax = fig.axes[0]

        from matplotlib.backend_bases import MouseEvent

        def trigger_click(xdata, ydata):
            disp = ax.transData.transform((xdata, ydata))
            event = MouseEvent("button_press_event", fig.canvas, disp[0], disp[1], button=1)
            event.inaxes = ax
            event.xdata = xdata
            event.ydata = ydata
            fig.canvas.callbacks.process("button_press_event", event)

        # Click far away to unselect
        trigger_click(1000.0, 1000.0)
        assert state.selected_id is None

        # Click near the object to select it again
        obj_translation = selected_obj.pose_at(state.current_time).translation
        trigger_click(obj_translation[0], obj_translation[1])
        assert state.selected_id == selected_obj.object_id
    finally:
        plt.close("all")
