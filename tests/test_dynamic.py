import pytest
from collision_helpers import collision_status
from crcc import Circle, CollisionCheckerBuilder, Compound, DynamicObstacle, Pose, Rectangle


def test_dynamic_query_against_static_environment(backend):
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
    checker = CollisionCheckerBuilder(backend=backend).add_static_obstacle(Circle(1.0)).build()

    assert collision_status(checker.collides_dynamic(trajectory)) == (True, 3)


def test_time_variant_obstacle_behavior(backend):
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
    checker1 = CollisionCheckerBuilder(backend=backend).add_static_obstacle(Circle(0.5)).build()
    assert collision_status(checker1.collides_dynamic(trajectory1)) == (True, 7)

    # Test time-variant dynamic obstacle respecting explicitly specified positions
    trajectory2 = DynamicObstacle.from_time_variant(
        [Rectangle(1.0, 1.0), Rectangle(1.0, 1.0)],
        2,
        [Pose.from_translation((5.0, 0.0)), Pose.from_translation((0.0, 0.0))],
    )
    checker2 = CollisionCheckerBuilder(backend=backend).add_static_obstacle(Circle(0.75)).build()
    assert collision_status(checker2.collides_dynamic(trajectory2)) == (True, 2)


def test_empty_compound_trajectories():
    """Verify empty trajectory definitions safely return no collisions."""
    trajectory = DynamicObstacle.from_time_variant([Compound([])], time_offset=0)
    checker = CollisionCheckerBuilder().add_static_obstacle(Rectangle(1.0, 1.0)).build()
    assert collision_status(checker.collides_dynamic(trajectory)) == (False, None)


def test_dynamic_obstacle_time_windows_and_batch_checks(backend):
    """Verify time window filtering and batch dynamic collision checks."""
    obstacle = DynamicObstacle(Circle(1.0), [Pose.from_translation((0.0, 0.0))], 5)
    miss = DynamicObstacle(Circle(0.5), [Pose.from_translation((10.0, 0.0))], 5)
    hit = DynamicObstacle(Circle(0.5), [Pose.from_translation((0.25, 0.0))], 5)
    checker = CollisionCheckerBuilder(backend=backend).add_dynamic_obstacle(obstacle).build()

    assert collision_status(checker.collides_dynamic(hit, min_time=4, max_time=4)) == (False, None)
    assert collision_status(checker.collides_dynamic(hit, min_time=5, max_time=5)) == (True, 5)
    assert checker.collides_dynamic_batch([], min_time=5, max_time=5) == []
    assert [
        collision_status(result) for result in checker.collides_dynamic_batch([miss, hit, miss], min_time=5, max_time=5)
    ] == [
        (False, None),
        (True, 5),
        (False, None),
    ]


def test_prepared_and_mixed_dynamic_batches_match_scalar_results(backend):
    obstacle_hit = DynamicObstacle(Circle(0.5), [Pose.from_translation((0.25, 0.0))], 5)
    obstacle_miss = DynamicObstacle(Circle(0.5), [Pose.from_translation((10.0, 0.0))], 5)
    checker = CollisionCheckerBuilder(backend=backend).add_dynamic_obstacle(obstacle_hit).build()
    prepared_hit = checker.prepare_dynamic(obstacle_hit)
    prepared_miss = checker.prepare_dynamic(obstacle_miss)

    mixed = [obstacle_hit, prepared_hit, obstacle_miss, prepared_miss]
    expected = [
        collision_status(checker.collides_dynamic(query))
        for query in (obstacle_hit, obstacle_hit, obstacle_miss, obstacle_miss)
    ]

    assert [collision_status(result) for result in checker.collides_dynamic_batch(mixed)] == expected
    assert [collision_status(result) for result in checker.collides_dynamic_batch(mixed, min_time=6)] == [
        (False, None)
    ] * 4


def test_static_batch_is_empty_safe_and_order_preserving(backend):
    checker = CollisionCheckerBuilder(backend=backend).add_static_obstacle(Circle(1.0)).build()
    positioned = [
        (Circle(0.25), Pose.from_translation((10.0, 0.0))),
        (Circle(0.25), Pose.from_translation((0.5, 0.0))),
        (Circle(0.25), Pose.from_translation((-10.0, 0.0))),
    ]

    assert checker.collides_static_batch([]) == []
    assert [collision_status(result) for result in checker.collides_static_batch(positioned)] == [
        (False, None),
        (True, None),
        (False, None),
    ]


def test_inverted_time_window_is_rejected():
    checker = CollisionCheckerBuilder().add_static_obstacle(Circle(1.0)).build()
    with pytest.raises(ValueError, match="min_time"):
        checker.collides_static(Circle(1.0), min_time=2, max_time=1)
