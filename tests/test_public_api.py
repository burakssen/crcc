import math

import crcc
import pytest
from collision_helpers import collision_status
from crcc.collision_checker import CollisionCheckerBuilder, road_boundary
from crcc.collision_object import Circle
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose

PUBLIC_API = {
    "Circle",
    "CollisionBackend",
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
    "road_boundary",
}


def test_root_api_is_complete_and_executable():
    assert set(crcc.__all__) == PUBLIC_API
    assert "road_boundary" in PUBLIC_API
    obstacle = crcc.Circle(1.0)
    query = crcc.Circle(0.5)
    assert obstacle.collides(query, backend=crcc.CollisionBackend.Parry)

    checker = crcc.CollisionCheckerBuilder(crcc.CollisionBackend.Parry).add_static_obstacle(obstacle).build()
    assert checker.backend == crcc.CollisionBackend.Parry
    assert collision_status(checker.collides_static(query)) == (True, None)

    prepared_query = checker.prepare_static(query)
    assert prepared_query.backend == checker.backend
    assert collision_status(checker.collides_static(prepared_query)) == (True, None)

    positioned = [(query, crcc.Pose.identity()), (prepared_query, crcc.Pose.from_translation((4.0, 0.0)))]
    assert [collision_status(result) for result in checker.collides_static_batch(positioned)] == [
        (True, None),
        (False, None),
    ]

    dynamic = crcc.DynamicObstacle(
        query,
        [crcc.Pose.from_translation((3.0, 0.0)), crcc.Pose.identity()],
        0,
    )
    assert collision_status(checker.collides_dynamic(dynamic)) == (True, 0)
    prepared_dynamic = checker.prepare_dynamic(dynamic)
    assert prepared_dynamic.backend == checker.backend
    assert collision_status(checker.collides_dynamic(prepared_dynamic)) == (True, 0)
    assert [collision_status(result) for result in checker.collides_dynamic_batch([dynamic, prepared_dynamic])] == [
        (True, 0),
        (True, 0),
    ]


def test_canonical_operations_cover_raw_and_prepared_queries(backend):
    obstacle = Circle(1.0)
    query = Circle(0.5)
    clear_pose = Pose.from_translation((4.0, 0.0))
    checker = CollisionCheckerBuilder(backend=backend).add_static_obstacle(obstacle).build()
    dynamic = DynamicObstacle(
        query,
        [Pose.from_translation((3.0, 0.0)), Pose.identity()],
        0,
    )

    raw_statuses = (
        collision_status(checker.collides_static(query)),
        collision_status(checker.collides_static(query, clear_pose)),
        collision_status(checker.collides_dynamic(dynamic)),
        collision_status(checker.collides_dynamic(dynamic, min_time=1)),
    )
    assert raw_statuses == ((True, None), (False, None), (True, 0), (True, 1))

    static_prepared = checker.prepare_static(query)
    dynamic_prepared = checker.prepare_dynamic(dynamic)
    prepared_statuses = (
        collision_status(checker.collides_static(static_prepared)),
        collision_status(checker.collides_static(static_prepared, clear_pose)),
        collision_status(checker.collides_dynamic(dynamic_prepared)),
        collision_status(checker.collides_dynamic(dynamic_prepared, min_time=1)),
    )
    assert prepared_statuses == raw_statuses

    mixed_static_batch = [
        (query, Pose.identity()),
        (static_prepared, Pose.identity()),
        (query, clear_pose),
        (static_prepared, clear_pose),
    ]
    assert [collision_status(r) for r in checker.collides_static_batch(mixed_static_batch)] == [
        (True, None),
        (True, None),
        (False, None),
        (False, None),
    ]

    repeated_prepared = [(static_prepared, pose) for pose in (Pose.identity(), clear_pose)]
    assert [collision_status(r) for r in checker.collides_static_batch(repeated_prepared)] == [
        (True, None),
        (False, None),
    ]

    mixed_dynamic_batch = [dynamic, dynamic_prepared]
    assert [collision_status(r) for r in checker.collides_dynamic_batch(mixed_dynamic_batch)] == [
        (True, 0),
        (True, 0),
    ]

    windowed = [collision_status(r) for r in checker.collides_dynamic_batch(mixed_dynamic_batch, min_time=1)]
    assert windowed == [(True, 1), (True, 1)]

    with pytest.raises(TypeError, match="PreparedStaticQuery"):
        checker.collides_static(42)  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="PreparedDynamicQuery"):
        checker.collides_dynamic(42)  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="PreparedStaticQuery"):
        checker.collides_static_batch([(42, Pose.identity())])  # type: ignore[list-item]


def test_empty_batches_return_empty_lists(backend):
    checker = CollisionCheckerBuilder(backend=backend).add_static_obstacle(Circle(1.0)).build()
    prepared = checker.prepare_static(Circle(0.25))

    assert checker.collides_static_batch([]) == []
    assert checker.collides_static_batch([]) == []
    assert checker.collides_dynamic_batch([]) == []
    assert checker.collides_static_batch([(prepared, pose) for pose in ()]) == []


def test_prepared_queries_reject_a_different_backend():
    parry = CollisionCheckerBuilder(crcc.CollisionBackend.Parry).build()
    rhusics = CollisionCheckerBuilder(crcc.CollisionBackend.Rhusics).build()
    static = parry.prepare_static(Circle(1.0))
    dynamic = parry.prepare_dynamic(DynamicObstacle(Circle(1.0), [Pose.identity()], 0))

    with pytest.raises(ValueError, match="not supported"):
        rhusics.collides_static(static)
    with pytest.raises(ValueError, match="not supported"):
        rhusics.collides_dynamic(dynamic)
    with pytest.raises(ValueError, match="not supported"):
        rhusics.collides_static_batch([(static, Pose.identity())])
    with pytest.raises(ValueError, match="not supported"):
        rhusics.collides_dynamic_batch([dynamic])


def test_builder_selects_backend_in_constructor_once(backend):
    builder = CollisionCheckerBuilder(backend=backend)
    checker = (
        builder.add_static_obstacle(Circle(1.0))
        .add_dynamic_obstacle(DynamicObstacle(Circle(0.5), [Pose.identity()], 0))
        .build()
    )

    assert checker.backend == backend
    assert collision_status(checker.collides_static(Circle(0.25))) == (True, None)
    assert collision_status(checker.collides_dynamic(DynamicObstacle(Circle(0.5), [Pose.identity()], 0))) == (
        True,
        0,
    )


def test_deprecated_aliases_still_work_and_warn():
    obstacle = Circle(1.0)
    query = Circle(0.5)

    with pytest.deprecated_call(match="engine="), pytest.deprecated_call(match="with_static_obstacle"):
        checker = CollisionCheckerBuilder(engine=crcc.CollisionEngine.Parry).with_static_obstacle(obstacle).build()

    positioned = [(query, Pose.identity())]
    prepared = checker.prepare_static(query)
    prepared_dynamic = checker.prepare_dynamic(DynamicObstacle(query, [Pose.identity()], 0))

    with pytest.deprecated_call(match="par_static"):
        assert [collision_status(result) for result in checker.par_static(positioned)] == [(True, None)]
    with pytest.deprecated_call(match="par_dynamic"):
        assert [
            collision_status(result) for result in checker.par_dynamic([DynamicObstacle(query, [Pose.identity()], 0)])
        ] == [(True, 0)]
    with pytest.deprecated_call(match="collides_static_prepared\\("):
        assert collision_status(checker.collides_static_prepared(prepared)) == (True, None)
    with pytest.deprecated_call(match="collides_static_prepared_batch"):
        assert [
            collision_status(result) for result in checker.collides_static_prepared_batch(prepared, [Pose.identity()])
        ] == [(True, None)]
    with pytest.deprecated_call(match="collides_dynamic_prepared\\("):
        assert collision_status(checker.collides_dynamic_prepared(prepared_dynamic)) == (True, 0)
    with pytest.deprecated_call(match="collides_dynamic_prepared_batch"):
        assert [collision_status(result) for result in checker.collides_dynamic_prepared_batch([prepared_dynamic])] == [
            (True, 0)
        ]
    with pytest.deprecated_call(match="with_engine"):
        rebuilt = CollisionCheckerBuilder().with_engine(crcc.CollisionBackend.Parry).build()
    assert rebuilt.backend == crcc.CollisionBackend.Parry
    with pytest.deprecated_call(match="with_dynamic_obstacle"):
        CollisionCheckerBuilder().with_dynamic_obstacle(DynamicObstacle(query, [Pose.identity()], 0))
    with pytest.deprecated_call(match="with_road_boundary"):
        lanelet = [[(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0), (-2.0, -2.0)]]
        CollisionCheckerBuilder().with_road_boundary(lanelet)
    with pytest.deprecated_call(match="engine is deprecated"):
        _ = checker.engine
    with pytest.deprecated_call(match="engine is deprecated"):
        _ = prepared.engine
    with pytest.deprecated_call(match="engine is deprecated"):
        _ = prepared_dynamic.engine


def test_engine_keyword_is_accepted_with_warning_on_pair_methods():
    left = Circle(1.0)
    right = Circle(1.0, (5.0, 0.0))

    with pytest.warns(DeprecationWarning):
        assert left.distance(right, engine=crcc.CollisionBackend.Parry) == pytest.approx(3.0)
    assert left.distance(right, backend=crcc.CollisionBackend.Parry) == pytest.approx(3.0)
    # Positional fourth argument keeps working because the value type is unchanged.
    assert right.distance(left, Pose.identity(), Pose.identity(), crcc.CollisionBackend.Parry) == pytest.approx(3.0)
    with pytest.raises(TypeError, match="not both"):
        left.distance(right, backend=crcc.CollisionBackend.Parry, engine=crcc.CollisionBackend.Parry)


def test_thread_controls_live_in_the_internal_benchmark_module():
    from crcc._core import benchmark as core_benchmark

    obstacle = Circle(1.0)
    query = Circle(0.5)
    checker = CollisionCheckerBuilder().add_static_obstacle(obstacle).build()
    result = core_benchmark.collides_static_batch_fresh_pool(checker, [(query, Pose.identity())], 1)

    assert [collision_status(item) for item in result] == [(True, None)]
    assert not hasattr(checker, "par_static_threads")
    assert not hasattr(checker, "_collides_static_batch_threads")


def test_collision_backend_identity_is_shared_between_names():
    assert crcc.CollisionEngine is crcc.CollisionBackend
    assert str(crcc.CollisionBackend.Parry) == "CollisionBackend.Parry"


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

    assert collision_status(checker.collides_static(query=query)) == (False, None)
    assert crcc.HalfSpace.from_points(point_1=(0.0, 0.0), point_2=(0.0, 1.0))
    assert crcc.Triangle(point_a=(0.0, 0.0), point_b=(1.0, 0.0), point_c=(0.0, 1.0))


def test_distance_merge_and_half_space_public_api(backend):
    left = crcc.Circle(1.0)
    right = crcc.Circle(1.0, (5.0, 0.0))
    assert left.distance(right, backend=backend) == pytest.approx(3.0)
    assert left.distance(crcc.Circle(1.0, (1.0, 0.0)), backend=backend) == 0.0
    with pytest.raises(ValueError, match="not supported"):
        crcc.Empty().distance(left, backend=backend)
    assert crcc.FullSpace().distance(left, backend=backend) == 0.0

    merged = left.merge(right)
    assert merged.collides(crcc.Circle(0.1), backend=backend)
    assert merged.collides(crcc.Circle(0.1, (5.0, 0.0)), backend=backend)
    assert not merged.collides(crcc.Circle(0.1, (2.5, 3.0)), backend=backend)
    assert not crcc.CollisionObject.merge_all([]).collides(left, backend=backend)

    half_space = crcc.HalfSpace((1.0, 0.0), 0.0)
    assert half_space.collides(crcc.Circle(0.1, (-1.0, 0.0)), backend=backend)
    assert not half_space.collides(crcc.Circle(0.1, (1.0, 0.0)), backend=backend)


def test_road_boundary_marks_only_space_outside_lanelets(backend):
    lanelet = [[(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0), (-2.0, -2.0)]]
    boundary = road_boundary(lanelet)

    assert not boundary.collides(crcc.Circle(0.1), backend=backend)
    assert boundary.collides(crcc.Circle(0.1, (3.0, 0.0)), backend=backend)
    assert road_boundary([]).collides(crcc.Circle(0.1), backend=backend)


def test_needle_polygon_does_not_poison_scene(backend):
    # Regression: parry pruned needle-thin convex polygons to <3 points and
    # marked the whole scene invalid (C-DEU_B471-1_1_T-1 road-boundary sliver).
    needle = crcc.Polygon([(-12.681534178140668, -2.7162407628206067), (37.1241, 16.1575), (36.6255, 15.9685)])
    checker = CollisionCheckerBuilder(backend=backend).add_static_obstacle(needle).build()
    query = crcc.Rectangle(4.4, 2.0)

    assert not collision_status(checker.collides_static(query))[0]
    assert collision_status(checker.collides_static(query, Pose.from_translation((30.0, 13.0))))[0]
    assert len(checker.collides_static_batch([(query, Pose.identity())])) == 1


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
