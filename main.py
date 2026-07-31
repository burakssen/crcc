import argparse
from collections.abc import Callable, Sequence
from enum import Enum
from pathlib import Path
from typing import Any, cast

from crcc import CollisionEngine

from tools import benchmark

ROOT = Path(__file__).resolve().parent
DEFAULT_SCENARIO_PATH = str(ROOT / "scenarios/DEU_MerzenichRather-2_870_T-149.xml")
DEFAULT_ENGINE = CollisionEngine.Rhusics


class ExampleAction(Enum):
    BASIC = "basic"
    CONTINUOUS = "continuous"
    COMMONROAD = "commonroad"
    CONCEPTS = "concepts"
    SHAPES = "shapes"
    DYNAMICS = "dynamics"
    SCENARIO = "scenario"
    ALL = "all"
    PLAYGROUND = "playground"
    STUDY = "study"
    REPORT = "report"


ENGINE_CHOICES = {
    "collide": CollisionEngine.Collide,
    "parry": CollisionEngine.Parry,
    "rhusics": CollisionEngine.Rhusics,
}
ACTION_CHOICES = {action.value: action for action in ExampleAction}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run deterministic CRCC tutorials and research benchmarks.")
    parser.add_argument("action", nargs="?", choices=sorted(ACTION_CHOICES), help="tutorial or research action")
    parser.add_argument("--engine", choices=sorted(ENGINE_CHOICES), default="rhusics", type=str.lower)
    parser.add_argument("--scenario", default=DEFAULT_SCENARIO_PATH)
    parser.add_argument("--benchmark-samples", type=int, default=benchmark.BENCHMARK_SAMPLE_COUNT)
    parser.add_argument("--benchmark-output", default=str(ROOT / benchmark.DEFAULT_OUTPUT_DIR))
    parser.add_argument("--benchmark-scenarios", nargs="+", default=["all"])
    parser.add_argument("--benchmark-thread-counts", nargs="+", type=int, default=None)
    parser.add_argument("--benchmark-repetitions", type=int, default=5)
    parser.add_argument("--benchmark-seed", type=int, default=2026)
    parser.add_argument(
        "--benchmark-engines", nargs="+", choices=sorted(name for name, _ in benchmark.ENGINE_ITEMS), default=None
    )
    parser.add_argument("--benchmark-step", choices=["run", "plot", "all"], default="all")
    parser.add_argument("--benchmark-profile", choices=["smoke", "spec"], default="smoke")
    parser.add_argument("--benchmark-suite", nargs="+", choices=["all", *benchmark.BENCHMARK_SUITES], default=["all"])
    parser.add_argument("--benchmark-include-stress", action="store_true")
    args = parser.parse_args(argv)
    args.action = ACTION_CHOICES[args.action] if args.action else None
    args.engine = ENGINE_CHOICES[args.engine]
    return args


def prompt_for_action(prompt: Callable[[str], str] = input) -> ExampleAction:
    actions = list(ExampleAction)
    print("Select an action:")
    for index, action in enumerate(actions, start=1):
        print(f"{index}. {action.value}")
    while True:
        selection = prompt("Choice: ").strip().lower()
        if selection.isdigit() and 1 <= int(selection) <= len(actions):
            return actions[int(selection) - 1]
        if selection in ACTION_CHOICES:
            return ACTION_CHOICES[selection]
        print(f"Choose 1-{len(actions)} or: {', '.join(ACTION_CHOICES)}")


def benchmark_scenario_paths(selection: Sequence[Any] | None) -> Any:
    if not selection or "all" in selection:
        return benchmark.discover_scenario_paths()
    return selection


def run_action(action: ExampleAction, scenario_path: str, engine: CollisionEngine, **benchmark_options: Any) -> Any:
    if action in {ExampleAction.BASIC, ExampleAction.CONCEPTS, ExampleAction.SHAPES}:
        from examples.basic import run

        return run(engine)
    if action in {ExampleAction.CONTINUOUS, ExampleAction.DYNAMICS}:
        from examples.continuous import run

        return run(engine)
    if action == ExampleAction.PLAYGROUND:
        from examples.utils import load_collision_checker, scenario_pose_bounds
        from tools.playground import run as run_playground

        scenario, checker = load_collision_checker(scenario_path, engine)
        bounds = cast(Any, scenario_pose_bounds(scenario))
        return cast(Any, run_playground)(scenario, checker, scenario_path, bounds)
    if action in {ExampleAction.COMMONROAD, ExampleAction.SCENARIO}:
        from examples.commonroad import run as run_cr
        from examples.utils import load_collision_checker, scenario_pose_bounds

        scenario, checker = load_collision_checker(scenario_path, engine)
        bounds = cast(Any, scenario_pose_bounds(scenario))
        return cast(Any, run_cr)(scenario, checker, scenario_path, bounds)
    if action == ExampleAction.ALL:
        from examples.basic import run as run_basic
        from examples.commonroad import run as run_commonroad
        from examples.continuous import run as run_continuous
        from examples.utils import load_collision_checker, scenario_pose_bounds

        run_basic(engine)
        run_continuous(engine)
        scenario, checker = load_collision_checker(scenario_path, engine)
        bounds = cast(Any, scenario_pose_bounds(scenario))
        return cast(Any, run_commonroad)(scenario, checker, scenario_path, bounds)
    if action in {ExampleAction.STUDY, ExampleAction.REPORT}:
        options: dict[str, Any] = dict(benchmark_options)
        options["scenario_paths"] = benchmark_scenario_paths(options.pop("benchmark_scenarios", None))
        options["sample_count"] = options.pop("benchmark_samples", benchmark.BENCHMARK_SAMPLE_COUNT)
        options["output_dir"] = options.pop("benchmark_output", str(benchmark.DEFAULT_OUTPUT_DIR))
        options["thread_counts"] = options.pop("benchmark_thread_counts", None)
        options["repetitions"] = options.pop("benchmark_repetitions", 5)
        options["seed"] = options.pop("benchmark_seed", 2026)
        options["engines"] = options.pop("benchmark_engines", None)
        requested_step = options.pop("benchmark_step", "all")
        options["step"] = "plot" if action == ExampleAction.REPORT else requested_step
        options["profile"] = options.pop("benchmark_profile", "smoke")
        options["suites"] = options.pop("benchmark_suite", ["all"])
        options["include_stress"] = options.pop("benchmark_include_stress", False)
        return cast(Any, benchmark.run_all)(**options)
    raise ValueError(f"Unsupported action: {action}")


def main(argv: Sequence[str] | None = None) -> Any:
    args = parse_args(argv)
    action = args.action or prompt_for_action()
    values = vars(args)
    values.pop("action")
    scenario_path = values.pop("scenario")
    engine = values.pop("engine")
    return run_action(action, scenario_path, engine, **values)


if __name__ == "__main__":
    try:
        main()
    except benchmark.ArtifactError as error:
        raise SystemExit(str(error)) from error
