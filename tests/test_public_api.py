import crcc
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
    "Rectangle",
    "Triangle",
}


def test_root_api_is_complete_and_executable():
    assert set(crcc.__all__) == PUBLIC_API
    obstacle = crcc.Circle(1.0)
    query = crcc.Circle(0.5)
    assert obstacle.collides(query)

    checker = (
        crcc.CollisionCheckerBuilder(crcc.CollisionEngine.Parry)
        .with_static_obstacle(obstacle)
        .build()
    )
    assert checker.collides_static(query).collides
    assert checker.collides_static_batch([(query, crcc.Pose.identity())])[0].collides

    dynamic = crcc.DynamicObstacle(
        query,
        [crcc.Pose.from_translation((3.0, 0.0)), crcc.Pose.identity()],
        0,
    )
    assert checker.collides_dynamic(dynamic).collides


def test_legacy_module_api_and_batch_names_are_executable():
    obstacle = Circle(1.0)
    query = Circle(0.5)
    checker = CollisionCheckerBuilder(CollisionEngine.Parry).with_static_obstacle(obstacle).build()
    positioned = [(query, Pose.identity())]
    assert checker.par_static(positioned)[0].collides
    assert checker.par_static_threads(positioned, 1)[0].collides

    dynamic = DynamicObstacle(query, [Pose.identity()], 0)
    assert checker.par_dynamic([dynamic])[0].collides
