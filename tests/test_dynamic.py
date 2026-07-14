import pytest
from crcc import Circle, CollisionCheckerBuilder, Compound, DynamicObstacle, Pose, Rectangle


def test_dynamic_query_against_static_environment(engine):
    """Test queries where a dynamic ego trajectory collides with a static checker environment."""
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


def test_time_variant_obstacle_behavior(engine):
    """Ensure dynamic obstacles composed of varying shapes and offsets collide correctly."""
    # Test shape variant step selection
    trajectory1 = DynamicObstacle.from_time_variant(
        [
            Circle(0.25, (10.0, 0.0)),
            Rectangle(2.0, 2.0, 0.0, (0.0, 0.0)),
            Circle(0.25, (10.0, 0.0)),
        ],
        time_offset=7,
    )
    checker1 = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(0.5)).build()
    result1 = checker1.collides_dynamic(trajectory1)
    assert result1.collides
    assert result1.time_step == 7

    # Test time-variant dynamic obstacle respecting explicitly specified positions
    trajectory2 = DynamicObstacle.from_time_variant(
        [Rectangle(1.0, 1.0), Rectangle(1.0, 1.0)],
        2,
        [Pose.from_translation((5.0, 0.0)), Pose.from_translation((0.0, 0.0))],
    )
    checker2 = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(0.75)).build()
    result2 = checker2.collides_dynamic(trajectory2)
    assert result2.collides
    assert result2.time_step == 2


def test_empty_compound_trajectories():
    """Verify empty trajectory definitions safely return no collisions."""
    trajectory = DynamicObstacle.from_time_variant([Compound([])], time_offset=0)
    checker = CollisionCheckerBuilder().with_static_obstacle(Rectangle(1.0, 1.0)).build()
    assert not checker.collides_dynamic(trajectory).collides


def test_dynamic_obstacle_time_windows_and_parallelization(engine):
    """Verify time window filtering and parallelized dynamic collision batch checks."""
    obstacle = DynamicObstacle(Circle(1.0), [Pose.from_translation((0.0, 0.0))], 5)
    miss = DynamicObstacle(Circle(0.5), [Pose.from_translation((10.0, 0.0))], 5)
    hit = DynamicObstacle(Circle(0.5), [Pose.from_translation((0.25, 0.0))], 5)
    checker = CollisionCheckerBuilder(engine=engine).with_dynamic_obstacle(obstacle).build()

    assert not checker.collides_dynamic(hit, min_time=4, max_time=4).collides
    assert checker.collides_dynamic(hit, min_time=5, max_time=5).time_step == 5

    def assert_parallel_matches_sequential(dynamic_obstacles):
        parallel = checker.collides_dynamic_batch(dynamic_obstacles, min_time=5, max_time=5)
        sequential = [
            checker.collides_dynamic(dynamic_obstacle, min_time=5, max_time=5) for dynamic_obstacle in dynamic_obstacles
        ]
        assert [str(result) for result in parallel] == [str(result) for result in sequential]

    assert_parallel_matches_sequential([miss, hit])
    assert_parallel_matches_sequential(
        [
            DynamicObstacle(
                Circle(0.5),
                [Pose.from_translation((10.0 if index % 2 == 0 else 0.25, 0.0))],
                5,
            )
            for index in range(40)
        ]
    )


def test_static_batch_matches_scalar(engine):
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(1.0)).build()
    positioned = [(Circle(0.25), Pose.from_translation((0.5 if index % 2 == 0 else 10.0, 0.0))) for index in range(40)]
    sequential = [checker.collides_static(shape, pose) for shape, pose in positioned]
    expected = [str(result) for result in sequential]

    assert [str(result) for result in checker.collides_static_batch(positioned)] == expected


def test_inverted_time_window_is_rejected():
    checker = CollisionCheckerBuilder().with_static_obstacle(Circle(1.0)).build()
    with pytest.raises(ValueError, match="min_time"):
        checker.collides_static(Circle(1.0), min_time=2, max_time=1)
