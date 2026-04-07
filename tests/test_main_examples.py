import io
import unittest
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


class MainExampleTests(unittest.TestCase):
    def test_parse_args_selects_action_scenario_and_engine(self):
        args = main.parse_args(
            [
                "benchmark",
                "--scenario",
                "scenarios/ZAM_Yield-1_1_T-1.xml",
                "--engine",
                "parry",
            ],
        )

        self.assertEqual(args.action, main.ExampleAction.BENCHMARK)
        self.assertEqual(args.scenario, "scenarios/ZAM_Yield-1_1_T-1.xml")
        self.assertEqual(args.engine, CollisionEngine.Parry)

    def test_prompt_for_action_accepts_numbered_selection(self):
        with redirect_stdout(io.StringIO()):
            action = main.prompt_for_action(prompt=lambda _: "3")

        self.assertEqual(action, main.ExampleAction.BENCHMARK)

    def test_python_builder_engine_selection_matches_for_simple_collision(self):
        query = Circle(1.0, (1.0, 0.0))

        for engine in [CollisionEngine.Parry, CollisionEngine.Rhusics]:
            with self.subTest(engine=engine):
                checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(1.0)).build()
                self.assertEqual(str(checker.collides_static(query)), "CollidesStatic")

    def test_static_collision_time_window_filtering(self):
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

        self.assertEqual(str(checker.collides_static(query, min_time=0, max_time=0)), "NoCollision")
        self.assertEqual(str(checker.collides_static(query, min_time=1, max_time=1)), "CollidesDynamic(1)")
        self.assertEqual(str(checker.collides_static(query, min_time=2, max_time=2)), "NoCollision")
        self.assertEqual(str(checker.collides_static(query, min_time=0, max_time=2)), "CollidesDynamic(0)")

    def test_parallel_static_results_match_sequential_results(self):
        checker = CollisionCheckerBuilder().with_static_obstacle(Circle(2.0)).build()
        positioned_queries = [(Circle(1.0, (float(index), 0.0)), Pose.identity()) for index in range(8)]

        parallel_results = checker.par_collides_static(positioned_queries)
        sequential_results = [checker.collides_static(query, pose) for query, pose in positioned_queries]

        self.assertEqual([str(result) for result in parallel_results], [str(result) for result in sequential_results])

    def test_bundled_scenarios_build_with_valid_pose_bounds(self):
        for scenario_path in sorted(Path("scenarios").glob("*.xml")):
            with self.subTest(scenario=scenario_path.name):
                scenario, checker = main.load_collision_checker(str(scenario_path))
                lower_bounds, upper_bounds = main.scenario_pose_bounds(scenario)

                self.assertLess(lower_bounds[0], upper_bounds[0])
                self.assertLess(lower_bounds[1], upper_bounds[1])
                self.assertLess(lower_bounds[2], upper_bounds[2])

                car = main.Rectangle(*main.CAR_SIZE)
                pose = main.Pose(
                    (
                        (lower_bounds[0] + upper_bounds[0]) / 2.0,
                        (lower_bounds[1] + upper_bounds[1]) / 2.0,
                    ),
                    0.0,
                )
                self.assertIsInstance(checker.collides_static(car, pose).collides, bool)

    def test_zam_yield_road_boundary_collision_matches_between_engines(self):
        scenario, _ = main.CommonRoadFileReader("scenarios/ZAM_Yield-1_1_T-1.xml").open()
        car = Rectangle(*main.CAR_SIZE)
        pose = Pose((62.013604720981206, -8.905038959453274), 0.8852293987803505)

        for engine in [CollisionEngine.Parry, CollisionEngine.Rhusics]:
            with self.subTest(engine=engine):
                checker = add_road_boundary_to_builder(
                    CollisionCheckerBuilder(engine=engine),
                    scenario.lanelet_network,
                ).build()
                self.assertTrue(checker.collides_static(car, pose).collides)

    def test_scenario_time_steps_are_ordered_and_non_empty(self):
        scenario, _ = main.CommonRoadFileReader("scenarios/ZAM_Yield-1_1_T-1.xml").open()

        time_steps = main.scenario_time_steps(scenario)

        self.assertGreater(len(time_steps), 0)
        self.assertEqual(time_steps, sorted(set(time_steps)))
        self.assertEqual(time_steps[0], 0)
        self.assertEqual(time_steps[-1], 80)

    def test_visualization_poses_are_deterministic(self):
        scenario, _ = main.CommonRoadFileReader(main.SCENARIO_PATH).open()
        pose_bounds = main.scenario_pose_bounds(scenario)

        first_poses = main.visualization_poses(main.SCENARIO_PATH, pose_bounds)
        second_poses = main.visualization_poses(main.SCENARIO_PATH, pose_bounds)

        self.assertEqual(len(first_poses), main.VISUALIZATION_SAMPLE_COUNT)
        self.assertEqual(
            [(pose.translation, pose.rotation) for pose in first_poses],
            [(pose.translation, pose.rotation) for pose in second_poses],
        )

    def test_visualization_poses_include_known_merge_samples_first(self):
        scenario, _ = main.CommonRoadFileReader(main.SCENARIO_PATH).open()
        pose_bounds = main.scenario_pose_bounds(scenario)

        poses = main.visualization_poses(main.SCENARIO_PATH, pose_bounds)

        self.assertEqual(poses[0].translation, (55.29, -1.99))
        self.assertEqual(poses[0].rotation, 1.326)
        self.assertEqual(poses[1].translation, (37.33, 4.07))
        self.assertEqual(poses[1].rotation, -2.207)

    def test_visualization_poses_respect_count_limit(self):
        scenario, _ = main.CommonRoadFileReader(main.SCENARIO_PATH).open()
        pose_bounds = main.scenario_pose_bounds(scenario)

        poses = main.visualization_poses(main.SCENARIO_PATH, pose_bounds, count=1)

        self.assertEqual(len(poses), 1)
        self.assertEqual(poses[0].translation, (55.29, -1.99))

    def test_collision_flags_persist_after_first_collision(self):
        collided_flags = [False, True, False]

        main.update_collision_flags(
            collided_flags,
            [CollisionResultStub(False), CollisionResultStub(False), CollisionResultStub(True)],
        )

        self.assertEqual(collided_flags, [False, True, True])


if __name__ == "__main__":
    unittest.main()
