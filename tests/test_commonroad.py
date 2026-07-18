from types import SimpleNamespace

import commonroad.scenario.obstacle as cr_obstacle
from commonroad.common.util import Interval
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy_group import OccupancyGroup
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.prediction.prediction import SetBasedPrediction
from commonroad.scenario.obstacle import ObstacleType, StaticObstacle
from commonroad.scenario.state import InitialState
from crcc import Circle, CollisionCheckerBuilder, DynamicObstacle, Pose, Rectangle
from crcc.commonroad import (
    add_static_obstacle,
    scenario_builder,
    to_dynamic_obstacle,
    to_occupancy,
)
from shapely.geometry import Point, Polygon as ShapelyPolygon


def test_occupancy_group_collision_mapping():
    """Ensure occupancy group members map correctly to local CollisionObject instances."""
    occupancy_group = OccupancyGroup(
        (
            CircleOccupancy(1.0, Point(0.0, 0.0)),
            RectOccupancy(Point(4.0, 0.0), width=1.0, length=2.0, orientation=0.0),
            PolygonOccupancy(ShapelyPolygon([(7.0, -0.5), (8.0, -0.5), (8.0, 0.5), (7.0, 0.5)])),
        )
    )
    collision_object = to_occupancy(occupancy_group)

    assert collision_object.collides(Circle(0.1, (0.0, 0.0)))
    assert collision_object.collides(Circle(0.1, (4.0, 0.0)))
    assert collision_object.collides(Circle(0.1, (7.5, 0.0)))
    assert not collision_object.collides(Circle(0.1, (2.0, 0.0)))


def test_empty_occupancy_group():
    """Check that an empty occupancy group produces a non-colliding object."""
    collision_object = to_occupancy(OccupancyGroup(()))
    assert not collision_object.collides(Circle(1.0))


def test_occupancy_group_time_variant_dynamic_obstacle(engine):
    """Ensure occupancy groups can be used as time-variant dynamic obstacle shapes."""
    occupancy_group = OccupancyGroup(
        (
            CircleOccupancy(1.0, Point(0.0, 0.0)),
            RectOccupancy(Point(4.0, 0.0), width=1.0, length=2.0, orientation=0.0),
        )
    )
    trajectory = DynamicObstacle.from_time_variant(
        [
            Circle(0.25, (10.0, 0.0)),
            to_occupancy(occupancy_group),
            Circle(0.25, (10.0, 0.0)),
        ],
        time_offset=4,
    )
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(0.5)).build()

    result = checker.collides_dynamic(trajectory, min_time=5, max_time=5)
    assert result.collides
    assert result.time_step == 5


def test_static_obstacle_conversion():
    """Verify conversion of static obstacles preserves coordinates and initial time states."""
    static_obstacle = StaticObstacle(
        obstacle_id=1,
        obstacle_type=ObstacleType.PARKED_VEHICLE,
        obstacle_shape=RectObstacleShape(width=2.0, length=4.0),
        initial_state=InitialState(time_step=3, position=(10.0, 0.0), orientation=0.0),
    )
    builder = CollisionCheckerBuilder()
    add_static_obstacle(builder, static_obstacle)
    checker = builder.build()

    assert checker.collides_static(Rectangle(1.0, 1.0), Pose((10.0, 0.0), 0.0)).collides
    assert not checker.collides_static(Rectangle(1.0, 1.0), Pose((20.0, 0.0), 0.0)).collides


def set_based_commonroad_obstacle():
    prediction = SetBasedPrediction(
        3,
        {
            3: CircleOccupancy(0.5, Point(10.0, 0.0)),
            5: CircleOccupancy(0.5, Point(0.0, 0.0)),
            Interval(7, 8): CircleOccupancy(0.5, Point(0.0, 0.0)),
        },
    )
    return cr_obstacle.DynamicObstacle(
        obstacle_id=2,
        obstacle_type=ObstacleType.CAR,
        obstacle_shape=RectObstacleShape(width=1.0, length=2.0),
        initial_state=InitialState(time_step=0, position=(10.0, 0.0), orientation=0.0),
        prediction=prediction,
    )


def test_set_based_prediction_dynamic_obstacle_conversion(engine):
    """Set-based predictions map occupancies by timestep and keep gaps non-colliding."""
    trajectory = to_dynamic_obstacle(set_based_commonroad_obstacle())
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(0.75)).build()

    assert not checker.collides_dynamic(trajectory, min_time=3, max_time=3).collides
    assert not checker.collides_dynamic(trajectory, min_time=4, max_time=4).collides
    assert checker.collides_dynamic(trajectory, min_time=5, max_time=5).time_step == 5
    assert checker.collides_dynamic(trajectory, min_time=7, max_time=7).time_step == 7
    assert checker.collides_dynamic(trajectory, min_time=8, max_time=8).time_step == 8


def test_scenario_builder_includes_set_based_dynamic_obstacles(engine):
    scenario = SimpleNamespace(
        lanelet_network=SimpleNamespace(lanelets=[]),
        static_obstacles=[],
        dynamic_obstacles=[set_based_commonroad_obstacle()],
    )
    checker = scenario_builder(scenario, CollisionCheckerBuilder(engine=engine)).build()

    assert checker.collides_static(Circle(0.75), min_time=5, max_time=5).time_step == 5
    assert not checker.collides_static(Circle(0.75), min_time=4, max_time=4).collides
