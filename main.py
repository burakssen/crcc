import argparse
from enum import Enum

from crcc.collision_checker import CollisionEngine

from examples import benchmark, concepts, dynamics, playground, scenario as scenario_example, shapes
from examples.utils import load_collision_checker, scenario_pose_bounds

DEFAULT_SCENARIO_PATH = "scenarios/DEU_MerzenichRather-2_870_T-149.xml"
DEFAULT_ENGINE = CollisionEngine.Rhusics


class ExampleAction(Enum):
    CONCEPTS = "concepts"
    SHAPES = "shapes"
    DYNAMICS = "dynamics"
    SCENARIO = "scenario"
    PLAYGROUND = "playground"
    STUDY = "study"
    REPORT = "report"
    ALL = "all"


ENGINE_CHOICES = {
    "collide": CollisionEngine.Collide,
    "parry": CollisionEngine.Parry,
    "rhusics": CollisionEngine.Rhusics,
}
ACTION_CHOICES = {action.value: action for action in ExampleAction}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Run research-oriented CommonRoad collision checker examples.")
    parser.add_argument(
        "action",
        nargs="?",
        choices=sorted(ACTION_CHOICES),
        help="example to run; omit to choose from an interactive menu",
    )
    parser.add_argument(
        "--scenario",
        default=DEFAULT_SCENARIO_PATH,
        help=f"CommonRoad scenario XML path (default: {DEFAULT_SCENARIO_PATH})",
    )
    parser.add_argument(
        "--engine",
        choices=sorted(ENGINE_CHOICES),
        default="rhusics",
        type=str.lower,
        help="collision engine to use for non-benchmark scenario examples",
    )
    parser.add_argument(
        "--benchmark-samples",
        type=int,
        default=benchmark.BENCHMARK_SAMPLE_COUNT,
        help=f"number of benchmark queries to generate (default: {benchmark.BENCHMARK_SAMPLE_COUNT})",
    )
    parser.add_argument(
        "--benchmark-output",
        default=str(benchmark.DEFAULT_OUTPUT_DIR),
        help=f"benchmark CSV output directory (default: {benchmark.DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--benchmark-scenarios",
        nargs="+",
        default=["all"],
        help="benchmark scenario XML paths, or 'all' for every bundled scenario (default: all)",
    )
    parser.add_argument(
        "--benchmark-thread-counts",
        nargs="+",
        type=int,
        default=None,
        help="thread counts for parallel scaling benchmarks (default: 1 2 4 8 16 capped by CPU count)",
    )
    parser.add_argument(
        "--benchmark-repetitions",
        type=int,
        default=benchmark.BenchmarkConfig.__dataclass_fields__["repetitions"].default,
        help="number of repeated measurements per workload/backend (default: 5)",
    )
    parser.add_argument(
        "--benchmark-seed",
        type=int,
        default=2026,
        help="deterministic workload seed (default: 2026)",
    )
    parser.add_argument(
        "--benchmark-engines",
        nargs="+",
        choices=sorted(name for name, _ in benchmark.ENGINE_ITEMS),
        default=None,
        help="collision engines to include in benchmark runs (default: all)",
    )
    parser.add_argument(
        "--benchmark-step",
        choices=["run", "plot", "all"],
        default="all",
        help="benchmark phase to execute (default: all)",
    )
    args = parser.parse_args(argv)
    if args.action is not None:
        args.action = ACTION_CHOICES[args.action]
    args.engine = ENGINE_CHOICES[args.engine]
    return args


def prompt_for_action(prompt=input):
    actions = list(ExampleAction)
    print("Select an example:")
    for index, action in enumerate(actions, start=1):
        print(f"{index}. {action.value}")

    while True:
        selection = prompt("Choice: ").strip().lower()
        if selection.isdigit():
            index = int(selection)
            if 1 <= index <= len(actions):
                return actions[index - 1]
        for action in actions:
            if selection == action.value:
                return action
        print(f"Enter a number from 1 to {len(actions)} or one of: {', '.join(action.value for action in actions)}")


def run_action(
    action,
    scenario_path: str,
    engine: CollisionEngine,
    benchmark_samples: int = benchmark.BENCHMARK_SAMPLE_COUNT,
    benchmark_output: str = str(benchmark.DEFAULT_OUTPUT_DIR),
    benchmark_scenarios: list[str] | None = None,
    benchmark_thread_counts: list[int] | None = None,
    benchmark_repetitions: int = benchmark.BenchmarkConfig.__dataclass_fields__["repetitions"].default,
    benchmark_seed: int = 2026,
    benchmark_engines: list[str] | None = None,
    benchmark_step: str = "all",
):
    if action == ExampleAction.CONCEPTS:
        concepts.run(engine)
        return
    if action == ExampleAction.SHAPES:
        shapes.run(engine)
        return
    if action == ExampleAction.STUDY:
        benchmark.run_all(
            benchmark_scenario_paths(benchmark_scenarios),
            sample_count=benchmark_samples,
            output_dir=benchmark_output,
            thread_counts=benchmark_thread_counts,
            repetitions=benchmark_repetitions,
            seed=benchmark_seed,
            engines=benchmark_engines,
            step=benchmark_step,
        )
        return
    if action == ExampleAction.REPORT:
        benchmark.run_all(
            benchmark_scenario_paths(benchmark_scenarios),
            sample_count=benchmark_samples,
            output_dir=benchmark_output,
            thread_counts=benchmark_thread_counts,
            repetitions=benchmark_repetitions,
            seed=benchmark_seed,
            engines=benchmark_engines,
            step="plot",
        )
        return

    scenario, checker = load_collision_checker(scenario_path, engine)
    pose_bounds = scenario_pose_bounds(scenario)

    if action == ExampleAction.SCENARIO:
        scenario_example.run(scenario, checker, scenario_path, pose_bounds)
    elif action == ExampleAction.DYNAMICS:
        dynamics.run(scenario, checker, scenario_path, pose_bounds)
    elif action == ExampleAction.PLAYGROUND:
        playground.run(scenario, checker, scenario_path, pose_bounds)
    elif action == ExampleAction.ALL:
        scenario_example.run(scenario, checker, scenario_path, pose_bounds)
        concepts.run(engine)
        shapes.run(engine)
        dynamics.run(scenario, checker, scenario_path, pose_bounds)
        benchmark.run_all(
            benchmark_scenario_paths(benchmark_scenarios or [scenario_path]),
            sample_count=benchmark_samples,
            output_dir=benchmark_output,
            thread_counts=benchmark_thread_counts,
            repetitions=benchmark_repetitions,
            seed=benchmark_seed,
            engines=benchmark_engines,
            step=benchmark_step,
        )
        playground.run(scenario, checker, scenario_path, pose_bounds)
    else:
        raise ValueError(f"Unsupported example action: {action}")


def main(argv=None):
    args = parse_args(argv)
    action = args.action or prompt_for_action()
    run_action(
        action,
        args.scenario,
        args.engine,
        args.benchmark_samples,
        args.benchmark_output,
        args.benchmark_scenarios,
        args.benchmark_thread_counts,
        args.benchmark_repetitions,
        args.benchmark_seed,
        args.benchmark_engines,
        args.benchmark_step,
    )


def benchmark_scenario_paths(selection: list[str] | None):
    if not selection or selection == ["all"]:
        return benchmark.discover_scenario_paths()
    if "all" in selection:
        return benchmark.discover_scenario_paths()
    return selection


if __name__ == "__main__":
    main()
