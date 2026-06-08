import argparse
from enum import Enum

from crcc.collision_checker import CollisionEngine

from examples import benchmark, features, geometry, interactive, smoke, visualize
from examples.utils import load_collision_checker, scenario_pose_bounds

DEFAULT_SCENARIO_PATH = "scenarios/DEU_MerzenichRather-2_870_T-149.xml"
DEFAULT_ENGINE = CollisionEngine.Rhusics


class ExampleAction(Enum):
    GEOMETRY = "geometry"
    FEATURES = "features"
    SMOKE = "smoke"
    BENCHMARK = "benchmark"
    VISUALIZE = "visualize"
    INTERACTIVE = "interactive"
    ALL = "all"


ENGINE_CHOICES = {
    "collide": CollisionEngine.Collide,
    "parry": CollisionEngine.Parry,
    "rhusics": CollisionEngine.Rhusics,
}
ACTION_CHOICES = {action.value: action for action in ExampleAction}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Run structured CommonRoad collision checker examples.")
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
        help="collision engine to use for scenario-based examples",
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


def run_action(action, scenario_path: str, engine: CollisionEngine):
    if action == ExampleAction.GEOMETRY:
        geometry.run()
        return
    if action == ExampleAction.FEATURES:
        features.run(engine)
        return

    scenario, checker = load_collision_checker(scenario_path, engine)
    pose_bounds = scenario_pose_bounds(scenario)

    if action == ExampleAction.SMOKE:
        smoke.run(scenario, checker, scenario_path, pose_bounds)
    elif action == ExampleAction.BENCHMARK:
        benchmark.run(checker, pose_bounds)
    elif action == ExampleAction.VISUALIZE:
        visualize.run(scenario, checker, scenario_path, pose_bounds)
    elif action == ExampleAction.INTERACTIVE:
        interactive.run(scenario, checker, scenario_path, pose_bounds)
    elif action == ExampleAction.ALL:
        smoke.run(scenario, checker, scenario_path, pose_bounds)
        geometry.run()
        features.run(engine)
        benchmark.run(checker, pose_bounds)
        visualize.run(scenario, checker, scenario_path, pose_bounds)
        interactive.run(scenario, checker, scenario_path, pose_bounds)
    else:
        raise ValueError(f"Unsupported example action: {action}")


def main(argv=None):
    args = parse_args(argv)
    action = args.action or prompt_for_action()
    run_action(action, args.scenario, args.engine)


if __name__ == "__main__":
    main()
