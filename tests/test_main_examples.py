import unittest
from pathlib import Path

from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Rectangle
from crcc.commonroad import add_road_boundary_to_builder
from crcc.pose import Pose

import main


class CollisionResultStub:
    def __init__(self, collides):
        self.collides = collides


class MainExampleTests(unittest.TestCase):
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

    def test_collision_flags_persist_after_first_collision(self):
        collided_flags = [False, True, False]

        main.update_collision_flags(
            collided_flags,
            [CollisionResultStub(False), CollisionResultStub(False), CollisionResultStub(True)],
        )

        self.assertEqual(collided_flags, [False, True, True])


if __name__ == "__main__":
    unittest.main()
