from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy_group import OccupancyGroup
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.scenario.obstacle import ObstacleType, StaticObstacle
from commonroad.scenario.state import InitialState
from crcc.collision_checker import CollisionCheckerBuilder
from crcc.collision_object import Circle, Rectangle
from crcc.commonroad import add_commonroad_static_obstacle_to_builder, commonroad_occupancy_to_collision_object
from crcc.pose import Pose
from shapely.geometry import Point, Polygon as ShapelyPolygon


def test_occupancy_group_collides_with_each_member():
    occupancy_group = OccupancyGroup(
        (
            CircleOccupancy(1.0, Point(0.0, 0.0)),
            RectOccupancy(Point(4.0, 0.0), width=1.0, length=2.0, orientation=0.0),
            PolygonOccupancy(ShapelyPolygon([(7.0, -0.5), (8.0, -0.5), (8.0, 0.5), (7.0, 0.5)])),
        )
    )
    collision_object = commonroad_occupancy_to_collision_object(occupancy_group)

    assert collision_object.collides(Circle(0.1, (0.0, 0.0)))
    assert collision_object.collides(Circle(0.1, (4.0, 0.0)))
    assert collision_object.collides(Circle(0.1, (7.5, 0.0)))


def test_occupancy_group_does_not_collide_in_gap_between_members():
    occupancy_group = OccupancyGroup(
        (
            CircleOccupancy(1.0, Point(0.0, 0.0)),
            RectOccupancy(Point(4.0, 0.0), width=1.0, length=2.0, orientation=0.0),
        )
    )
    collision_object = commonroad_occupancy_to_collision_object(occupancy_group)

    assert not collision_object.collides(Circle(0.1, (2.0, 0.0)))


def test_empty_occupancy_group_does_not_collide():
    collision_object = commonroad_occupancy_to_collision_object(OccupancyGroup(()))

    assert not collision_object.collides(Circle(1.0))


def test_static_obstacle_uses_initial_occupancy():
    static_obstacle = StaticObstacle(
        obstacle_id=1,
        obstacle_type=ObstacleType.PARKED_VEHICLE,
        obstacle_shape=RectObstacleShape(width=2.0, length=4.0),
        initial_state=InitialState(time_step=3, position=(10.0, 0.0), orientation=0.0),
    )
    builder = CollisionCheckerBuilder()
    add_commonroad_static_obstacle_to_builder(builder, static_obstacle)
    checker = builder.build()

    assert checker.collides_static(Rectangle(1.0, 1.0), Pose((10.0, 0.0), 0.0)).collides
    assert not checker.collides_static(Rectangle(1.0, 1.0), Pose((20.0, 0.0), 0.0)).collides
