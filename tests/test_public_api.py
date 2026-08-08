import math

import crcc
import pytest
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.collision_object import Circle
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose

PUBLIC_API = {
    "Circle",
    "CollisionChecker",
    "CollisionCheckerBuilder",
    "CollisionEngine",
    "CollisionObject",
    "CollisionStatus",
    "Compound",
    "DynamicObstacle",
    "Empty",
    "FullSpace",
    "HalfSpace",
    "Polygon",
    "Pose",
    "PreparedDynamicQuery",
    "PreparedStaticQuery",
    "Rectangle",
    "Triangle",
}


def test_root_api_is_complete_and_executable():
    assert set(crcc.__all__) == PUBLIC_API
    obstacle = crcc.Circle(1.0)
    query = crcc.Circle(0.5)
    assert obstacle.collides(query)

    checker = crcc.CollisionCheckerBuilder(crcc.CollisionEngine.Parry).with_static_obstacle(obstacle).build()
    assert checker.collides_static(query).collides
    prepared_query = checker.prepare_static(query)
    assert prepared_query.engine == checker.engine
    assert checker.collides_static_prepared(prepared_query).collides
    assert checker.collides_static_batch([(query, crcc.Pose.identity())])[0].collides

    dynamic = crcc.DynamicObstacle(
        query,
        [crcc.Pose.from_translation((3.0, 0.0)), crcc.Pose.identity()],
        0,
    )
    assert checker.collides_dynamic(dynamic).collides
    prepared_dynamic = checker.prepare_dynamic(dynamic)
    assert prepared_dynamic.engine == checker.engine
    assert checker.collides_dynamic_prepared(prepared_dynamic).collides


def test_legacy_module_api_and_batch_names_are_executable():
    obstacle = Circle(1.0)
    query = Circle(0.5)
    checker = CollisionCheckerBuilder(CollisionEngine.Parry).with_static_obstacle(obstacle).build()
    positioned = [(query, Pose.identity())]
    assert checker.par_static(positioned)[0].collides
    assert checker.par_static_threads(positioned, 1)[0].collides

    dynamic = DynamicObstacle(query, [Pose.identity()], 0)
    assert checker.par_dynamic([dynamic])[0].collides


def test_pose_rejects_non_finite_values():
    with pytest.raises(ValueError, match="finite"):
        Pose((math.nan, 0.0), 0.0)
    with pytest.raises(ValueError, match="finite"):
        Pose.from_translation((math.inf, 0.0))
    with pytest.raises(ValueError, match="finite"):
        Pose.from_rotation(math.nan)


def test_stubbed_keywords_match_runtime_api():
    query = crcc.Circle(0.5)
    checker = crcc.CollisionCheckerBuilder().build()

    assert not checker.collides_static(query_shape=query).collides
    assert crcc.HalfSpace.from_points(point_1=(0.0, 0.0), point_2=(0.0, 1.0))
    assert crcc.Triangle(point_a=(0.0, 0.0), point_b=(1.0, 0.0), point_c=(0.0, 1.0))
