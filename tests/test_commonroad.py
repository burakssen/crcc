from types import SimpleNamespace
from typing import cast

import commonroad.scenario.obstacle as cr_obstacle
import numpy as np
from collision_helpers import collision_status
from commonroad.common.util import Interval
from commonroad.geometry.obstacle_shapes.circle_obstacle_shape import CircleObstacleShape
from commonroad.geometry.obstacle_shapes.polygon_obstacle_shape import PolygonObstacleShape
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy_group import OccupancyGroup
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.prediction.prediction import SetBasedPrediction, TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import ObstacleType, StaticObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState
from commonroad.scenario.trajectory import Trajectory
from crcc import Circle, CollisionCheckerBuilder, DynamicObstacle, Pose, Rectangle
from crcc.commonroad import (
    add_static_obstacle,
    from_dynamic_obstacle,
    from_occupancy,
    from_pose,
    from_shape,
    road_boundary,
    scenario_builder,
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
    collision_object = from_occupancy(occupancy_group)

    assert collision_object.collides(Circle(0.1, (0.0, 0.0)))
    assert collision_object.collides(Circle(0.1, (4.0, 0.0)))
    assert collision_object.collides(Circle(0.1, (7.5, 0.0)))
    assert not collision_object.collides(Circle(0.1, (2.0, 0.0)))


def test_empty_occupancy_group():
    """Check that an empty occupancy group produces a non-colliding object."""
    collision_object = from_occupancy(OccupancyGroup(()))
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
            from_occupancy(occupancy_group),
            Circle(0.25, (10.0, 0.0)),
        ],
        time_offset=4,
    )
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(0.5)).build()

    assert collision_status(checker.collides_dynamic(trajectory, min_time=5, max_time=5)) == (True, 5)


def test_static_obstacle_conversion():
    """Verify conversion of static obstacles preserves coordinates and initial time states."""
    static_obstacle = StaticObstacle(
        obstacle_id=1,
        obstacle_type=ObstacleType.PARKED_VEHICLE,
        obstacle_shape=RectObstacleShape(width=2.0, length=4.0),
        initial_state=InitialState(time_step=3, position=np.array((10.0, 0.0)), orientation=0.0),
    )
    builder = CollisionCheckerBuilder()
    add_static_obstacle(builder, static_obstacle)
    checker = builder.build()

    assert collision_status(checker.collides_static(Rectangle(1.0, 1.0), Pose((10.0, 0.0), 0.0))) == (
        True,
        None,
    )
    assert collision_status(checker.collides_static(Rectangle(1.0, 1.0), Pose((20.0, 0.0), 0.0))) == (
        False,
        None,
    )


def test_commonroad_from_pose_conversion():
    state = InitialState(time_step=0, position=np.array((10.0, 5.0)), orientation=0.5)
    pose = from_pose(state)
    assert pose.translation == (10.0, 5.0)
    assert pose.rotation == 0.5


def test_commonroad_shape_converters_preserve_geometry():
    circle = from_shape(CircleObstacleShape(2.0))
    shifted_rectangle = from_shape(RectObstacleShape(width=2.0, length=4.0, origin_x_shift=1.0))
    polygon = from_shape(PolygonObstacleShape(((0.0, 0.0), (2.0, 0.0), (0.0, 2.0))))

    assert circle.collides(Circle(0.1, (1.9, 0.0)))
    assert shifted_rectangle.collides(Circle(0.1, (-2.5, 0.0)))
    assert not shifted_rectangle.collides(Circle(0.1, (1.5, 0.0)))
    assert polygon.collides(Circle(0.1, (0.5, 0.5)))


def test_commonroad_road_boundary_uses_lanelet_vertices(engine):
    vertices = np.array(((-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0), (-2.0, -2.0)))
    network = SimpleNamespace(lanelets=[SimpleNamespace(polygon=SimpleNamespace(vertices=vertices))])
    boundary = road_boundary(cast(LaneletNetwork, network))

    assert not boundary.collides(Circle(0.1), engine=engine)
    assert boundary.collides(Circle(0.1, (3.0, 0.0)), engine=engine)


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
        initial_state=InitialState(time_step=0, position=np.array((10.0, 0.0)), orientation=0.0),
        prediction=prediction,
    )


def test_set_based_prediction_dynamic_obstacle_conversion(engine):
    """Set-based predictions map occupancies by timestep and keep gaps non-colliding."""
    trajectory = from_dynamic_obstacle(set_based_commonroad_obstacle())
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(0.75)).build()

    assert collision_status(checker.collides_dynamic(trajectory, min_time=3, max_time=3)) == (False, None)
    assert collision_status(checker.collides_dynamic(trajectory, min_time=4, max_time=4)) == (False, None)
    assert collision_status(checker.collides_dynamic(trajectory, min_time=5, max_time=5)) == (True, 5)
    assert collision_status(checker.collides_dynamic(trajectory, min_time=7, max_time=7)) == (True, 7)
    assert collision_status(checker.collides_dynamic(trajectory, min_time=8, max_time=8)) == (True, 8)


def test_set_based_prediction_keeps_initial_occupancy_and_gap(engine):
    trajectory = from_dynamic_obstacle(set_based_commonroad_obstacle())
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(0.75, (10.0, 0.0))).build()

    assert collision_status(checker.collides_dynamic(trajectory, min_time=0, max_time=0)) == (True, 0)
    assert collision_status(checker.collides_dynamic(trajectory, min_time=1, max_time=1)) == (False, None)
    assert collision_status(checker.collides_dynamic(trajectory, min_time=2, max_time=2)) == (False, None)
    assert collision_status(checker.collides_dynamic(trajectory, min_time=3, max_time=3)) == (True, 3)


def test_set_based_prediction_gap_has_no_phantom_continuous_motion(engine):
    trajectory = from_dynamic_obstacle(set_based_commonroad_obstacle())
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(0.25, (5.0, 0.0))).build()

    assert collision_status(checker.collides_dynamic(trajectory, min_time=0, max_time=3)) == (False, None)
    prepared = checker.prepare_dynamic(trajectory)
    assert collision_status(checker.collides_dynamic_prepared(prepared, min_time=0, max_time=3)) == (False, None)


def test_trajectory_prediction_keeps_between_step_motion(engine):
    shape = RectObstacleShape(width=1.0, length=1.0)
    prediction = TrajectoryPrediction(
        Trajectory(
            1,
            [InitialState(time_step=1, position=np.array((2.0, 0.0)), orientation=0.0)],
        ),
        shape,
    )
    obstacle = cr_obstacle.DynamicObstacle(
        obstacle_id=3,
        obstacle_type=ObstacleType.CAR,
        obstacle_shape=shape,
        initial_state=InitialState(time_step=0, position=np.array((-2.0, 0.0)), orientation=0.0),
        prediction=prediction,
    )
    trajectory = from_dynamic_obstacle(obstacle)
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Circle(0.25)).build()

    assert collision_status(checker.collides_dynamic(trajectory)) == (True, 0)


def test_scenario_builder_includes_set_based_dynamic_obstacles(engine):
    scenario = SimpleNamespace(
        lanelet_network=SimpleNamespace(lanelets=[]),
        static_obstacles=[],
        dynamic_obstacles=[set_based_commonroad_obstacle()],
    )
    checker = scenario_builder(cast(Scenario, scenario), CollisionCheckerBuilder(engine=engine)).build()

    assert collision_status(checker.collides_static(Circle(0.75), min_time=5, max_time=5)) == (True, 5)
    assert collision_status(checker.collides_static(Circle(0.75), min_time=4, max_time=4)) == (False, None)


def test_polygon_occupancy_with_duplicate_vertex_does_not_poison_queries(engine):
    """Regression: a duplicated consecutive vertex once made parry reject
    every query against a scene containing the converted occupancy."""
    clean = ShapelyPolygon([(0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)])
    duplicated = ShapelyPolygon([(0.0, 0.0), (4.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)])

    def obstacle_with_occupancy(occupancy):
        prediction = SetBasedPrediction(2, {2: PolygonOccupancy(occupancy)})
        return cr_obstacle.DynamicObstacle(
            obstacle_id=7,
            obstacle_type=ObstacleType.PEDESTRIAN,
            obstacle_shape=RectObstacleShape(width=1.0, length=1.0),
            initial_state=InitialState(time_step=0, position=np.array((0.0, 0.0)), orientation=0.0),
            prediction=prediction,
        )

    def scene_with(occupancy):
        return (
            CollisionCheckerBuilder(engine=engine)
            .with_static_obstacle(Circle(0.1, (50.0, 50.0)))
            .with_dynamic_obstacle(from_dynamic_obstacle(obstacle_with_occupancy(occupancy)))
            .build()
        )

    clean_scene = scene_with(clean)
    duplicated_scene = scene_with(duplicated)

    probe = Circle(0.25, (2.0, 1.0))
    assert collision_status(duplicated_scene.collides_static(probe, min_time=2, max_time=2)) == (True, 2)
    assert collision_status(duplicated_scene.collides_static(probe, min_time=3, max_time=3)) == (False, None)
    assert collision_status(duplicated_scene.collides_static(Circle(10.0, (50.0, 50.0)), min_time=2, max_time=2)) == (
        True,
        None,
    )
    assert collision_status(duplicated_scene.collides_static(probe, min_time=2, max_time=2)) == collision_status(
        clean_scene.collides_static(probe, min_time=2, max_time=2)
    )
