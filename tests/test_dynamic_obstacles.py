from crcc.collision_checker import CollisionCheckerBuilder
from crcc.collision_object import Circle, Compound, Rectangle
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose


def test_dynamic_query_collides_with_static_only_checker(engine):
    trajectory = DynamicObstacle(
        Circle(1.0),
        [
            Pose.from_translation((5.0, 0.0)),
            Pose.from_translation((0.5, 0.0)),
            Pose.from_translation((5.0, 0.0)),
        ],
        3,
    )
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(1.0)).build()

    result = checker.collides_dynamic(trajectory)

    assert result.collides
    assert result.time_step == 3


def test_time_variant_obstacle_uses_per_step_shapes(engine):
    trajectory = DynamicObstacle.from_time_variant(
        [
            Circle(0.25, (10.0, 0.0)),
            Rectangle(2.0, 2.0, 0.0, (0.0, 0.0)),
            Circle(0.25, (10.0, 0.0)),
        ],
        time_offset=7,
    )
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(0.5)).build()

    result = checker.collides_dynamic(trajectory)

    assert result.collides
    assert result.time_step == 8


def test_time_variant_obstacle_respects_optional_poses(engine):
    trajectory = DynamicObstacle.from_time_variant(
        [Rectangle(1.0, 1.0), Rectangle(1.0, 1.0)],
        2,
        [Pose.from_translation((5.0, 0.0)), Pose.from_translation((0.0, 0.0))],
    )
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(0.75)).build()

    result = checker.collides_dynamic(trajectory)

    assert result.collides
    assert result.time_step == 3


def test_empty_compound_trajectory_does_not_collide():
    trajectory = DynamicObstacle.from_time_variant([Compound([])], time_offset=0)
    checker = CollisionCheckerBuilder().with_static_obstacle(Rectangle(1.0, 1.0)).build()

    assert not checker.collides_dynamic(trajectory).collides


def test_dynamic_dynamic_time_window_and_parallel_results(engine):
    obstacle = DynamicObstacle(Circle(1.0), [Pose.from_translation((0.0, 0.0))], 5)
    miss = DynamicObstacle(Circle(0.5), [Pose.from_translation((10.0, 0.0))], 5)
    hit = DynamicObstacle(Circle(0.5), [Pose.from_translation((0.25, 0.0))], 5)
    checker = CollisionCheckerBuilder(engine=engine).with_dynamic_obstacle(obstacle).build()

    assert not checker.collides_dynamic(hit, min_time=4, max_time=4).collides
    assert checker.collides_dynamic(hit, min_time=5, max_time=5).time_step == 5

    parallel = checker.par_collides_dynamic([miss, hit], min_time=5, max_time=5)
    sequential = [
        checker.collides_dynamic(miss, min_time=5, max_time=5),
        checker.collides_dynamic(hit, min_time=5, max_time=5),
    ]
    assert [str(result) for result in parallel] == [str(result) for result in sequential]
