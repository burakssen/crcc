import math

import crcc
import pytest
from collision_helpers import collision_status
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine, road_boundary
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
    assert collision_status(checker.collides_static(query)) == (True, None)
    prepared_query = checker.prepare_static(query)
    assert prepared_query.engine == checker.engine
    assert collision_status(checker.collides_static_prepared(prepared_query)) == (True, None)
    assert [collision_status(result) for result in checker.collides_static_batch([(query, crcc.Pose.identity())])] == [
        (True, None)
    ]

    dynamic = crcc.DynamicObstacle(
        query,
        [crcc.Pose.from_translation((3.0, 0.0)), crcc.Pose.identity()],
        0,
    )
    assert collision_status(checker.collides_dynamic(dynamic)) == (True, 0)
    prepared_dynamic = checker.prepare_dynamic(dynamic)
    assert prepared_dynamic.engine == checker.engine
    assert collision_status(checker.collides_dynamic_prepared(prepared_dynamic)) == (True, 0)


def test_legacy_module_api_and_batch_names_are_executable():
    obstacle = Circle(1.0)
    query = Circle(0.5)
    checker = CollisionCheckerBuilder(CollisionEngine.Parry).with_static_obstacle(obstacle).build()
    positioned = [(query, Pose.identity())]
    assert [collision_status(result) for result in checker.par_static(positioned)] == [(True, None)]
    assert [collision_status(result) for result in checker.par_static_threads(positioned, 1)] == [(True, None)]

    dynamic = DynamicObstacle(query, [Pose.identity()], 0)
    assert [collision_status(result) for result in checker.par_dynamic([dynamic])] == [(True, 0)]


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

    assert collision_status(checker.collides_static(query_shape=query)) == (False, None)
    assert crcc.HalfSpace.from_points(point_1=(0.0, 0.0), point_2=(0.0, 1.0))
    assert crcc.Triangle(point_a=(0.0, 0.0), point_b=(1.0, 0.0), point_c=(0.0, 1.0))


def test_distance_merge_and_half_space_public_api(engine):
    left = crcc.Circle(1.0)
    right = crcc.Circle(1.0, (5.0, 0.0))
    assert left.distance(right, engine=engine) == pytest.approx(3.0)
    assert left.distance(crcc.Circle(1.0, (1.0, 0.0)), engine=engine) == 0.0
    with pytest.raises(ValueError, match="not supported"):
        crcc.Empty().distance(left, engine=engine)
    assert crcc.FullSpace().distance(left, engine=engine) == 0.0

    merged = left.merge(right)
    assert merged.collides(crcc.Circle(0.1), engine=engine)
    assert merged.collides(crcc.Circle(0.1, (5.0, 0.0)), engine=engine)
    assert not merged.collides(crcc.Circle(0.1, (2.5, 3.0)), engine=engine)
    assert not crcc.CollisionObject.merge_all([]).collides(left, engine=engine)

    half_space = crcc.HalfSpace((1.0, 0.0), 0.0)
    assert half_space.collides(crcc.Circle(0.1, (-1.0, 0.0)), engine=engine)
    assert not half_space.collides(crcc.Circle(0.1, (1.0, 0.0)), engine=engine)


def test_road_boundary_marks_only_space_outside_lanelets(engine):
    lanelet = [[(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0), (-2.0, -2.0)]]
    boundary = road_boundary(lanelet)

    assert not boundary.collides(crcc.Circle(0.1), engine=engine)
    assert boundary.collides(crcc.Circle(0.1, (3.0, 0.0)), engine=engine)
    assert road_boundary([]).collides(crcc.Circle(0.1), engine=engine)


def test_needle_polygon_does_not_poison_scene(engine):
    # Regression: parry pruned needle-thin convex polygons to <3 points and
    # marked the whole scene invalid (C-DEU_B471-1_1_T-1 road-boundary sliver).
    needle = crcc.Polygon([(-12.681534178140668, -2.7162407628206067), (37.1241, 16.1575), (36.6255, 15.9685)])
    checker = CollisionCheckerBuilder(engine).with_static_obstacle(needle).build()
    query = crcc.Rectangle(4.4, 2.0)

    assert not collision_status(checker.collides_static(query))[0]
    assert collision_status(checker.collides_static(query, Pose.from_translation((30.0, 13.0))))[0]
    assert len(checker.collides_static_batch([(query, Pose.identity())])) == 1


def test_prepared_queries_reject_a_different_engine():
    parry = CollisionCheckerBuilder(CollisionEngine.Parry).build()
    rhusics = CollisionCheckerBuilder(CollisionEngine.Rhusics).build()
    static = parry.prepare_static(Circle(1.0))
    dynamic = parry.prepare_dynamic(DynamicObstacle(Circle(1.0), [Pose.identity()], 0))

    with pytest.raises(ValueError, match="not supported"):
        rhusics.collides_static_prepared(static)
    with pytest.raises(ValueError, match="not supported"):
        rhusics.collides_dynamic_prepared(dynamic)


@pytest.mark.parametrize(
    "factory",
    [
        lambda: crcc.Circle(0.0),
        lambda: crcc.Rectangle(0.0, 1.0),
        lambda: crcc.HalfSpace((0.0, 0.0)),
        lambda: crcc.Polygon([(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)]),
        lambda: crcc.Triangle((math.nan, 0.0), (1.0, 0.0), (0.0, 1.0)),
    ],
    ids=["circle-radius", "rectangle-length", "half-space-normal", "polygon-ring", "triangle-coordinate"],
)
def test_invalid_shapes_are_rejected(factory):
    with pytest.raises(ValueError):
        factory()
