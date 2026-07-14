from __future__ import annotations

import commonroad.scenario.obstacle as cr_obstacle
from commonroad.geometry.obstacle_shapes.circle_obstacle_shape import CircleObstacleShape
from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.obstacle_shapes.polygon_obstacle_shape import PolygonObstacleShape
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.prediction.prediction import SetBasedPrediction, TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState, TraceState
from shapely.geometry import MultiPolygon, Polygon as ShapelyPolygon
from shapely.geometry.base import BaseGeometry

from crcc import Circle, CollisionCheckerBuilder, CollisionObject, Compound, DynamicObstacle, Polygon, Pose, Rectangle

ROAD_BOUNDARY_SIMPLIFY_TOLERANCE = 0.01
ROAD_BOUNDARY_MIN_HOLE_AREA = 0.001


def create_collision_checker_from_scenario(
    scenario: Scenario,
    builder: CollisionCheckerBuilder | None = None,
) -> CollisionCheckerBuilder:
    """Creates a collision checker builder from a CommonRoad scenario."""
    if builder is None:
        builder = CollisionCheckerBuilder()

    builder = add_road_boundary_to_builder(builder, scenario.lanelet_network)
    for static_obstacle in scenario.static_obstacles:
        builder = add_commonroad_static_obstacle_to_builder(builder, static_obstacle)
    for dynamic_obstacle in scenario.dynamic_obstacles:
        builder = add_commonroad_dynamic_obstacle_to_builder(builder, dynamic_obstacle)
    return builder


def add_commonroad_static_obstacle_to_builder(
    builder: CollisionCheckerBuilder,
    static_obstacle: cr_obstacle.StaticObstacle,
) -> CollisionCheckerBuilder:
    """Adds a CommonRoad static obstacle to the builder."""
    collision_object = commonroad_occupancy(static_obstacle.occupancy_at_time(static_obstacle.initial_state.time_step))
    builder.with_static_obstacle(collision_object)
    return builder


def add_commonroad_dynamic_obstacle_to_builder(
    builder: CollisionCheckerBuilder,
    dynamic_obstacle: cr_obstacle.DynamicObstacle,
) -> CollisionCheckerBuilder:
    """Adds a CommonRoad dynamic obstacle to the builder."""
    builder.with_dynamic_obstacle(commonroad_dynamic_obstacle(dynamic_obstacle))
    return builder


def commonroad_dynamic_obstacle(dynamic_obstacle: cr_obstacle.DynamicObstacle) -> DynamicObstacle:
    """Converts a CommonRoad dynamic obstacle to a local crcc DynamicObstacle."""
    initial_time = dynamic_obstacle.initial_state.time_step
    if isinstance(dynamic_obstacle.prediction, TrajectoryPrediction):
        trajectory = dynamic_obstacle.prediction.trajectory
        states = [dynamic_obstacle.initial_state] + trajectory.state_list
        poses = [commonroad_state_to_pose(state) for state in states]
        shape = commonroad_shape(dynamic_obstacle.obstacle_shape)
        return DynamicObstacle(shape, poses, initial_time)
    if isinstance(dynamic_obstacle.prediction, SetBasedPrediction):
        prediction = dynamic_obstacle.prediction
        time_steps = range(prediction.initial_time_step, _prediction_final_time_step(prediction) + 1)
        obstacles = [
            commonroad_occupancy(occupancy)
            if (occupancy := prediction.occupancy_at_time_step(time_step))
            else Compound([])
            for time_step in time_steps
        ]
        return DynamicObstacle.from_time_variant(obstacles, prediction.initial_time_step)

    prediction_type = type(dynamic_obstacle.prediction).__name__
    raise NotImplementedError(f"Unsupported dynamic obstacle prediction type: {prediction_type}")


def _prediction_final_time_step(prediction: SetBasedPrediction) -> int:
    final_time_step = prediction.final_time_step
    return final_time_step.end if hasattr(final_time_step, "end") else final_time_step


def add_road_boundary_to_builder(
    builder: CollisionCheckerBuilder,
    lanelet_network: LaneletNetwork,
) -> CollisionCheckerBuilder:
    """Adds the road boundary from a lanelet network to the builder."""
    builder.with_static_obstacle(road_boundary(lanelet_network))
    return builder


def road_boundary(lanelet_network: LaneletNetwork) -> CollisionObject:
    """Creates an obstacle for all space outside the lanelet network."""
    import crcc._core.collision_checker as core

    lanelets = [[tuple(v) for v in lanelet.polygon.vertices] for lanelet in lanelet_network.lanelets]
    return core.road_boundary(lanelets)


def commonroad_polygon(polygon: ShapelyPolygon) -> CollisionObject:
    """Convert a Shapely polygon, including holes, to a collision object."""
    return Polygon(
        exterior=[tuple(v) for v in polygon.exterior.coords],
        interiors=[[tuple(v) for v in interior.coords] for interior in polygon.interiors],
    )


def commonroad_shape(shape: ObstacleShape) -> CollisionObject:
    """Converts a CommonRoad obstacle shape to a local crcc CollisionObject."""
    if isinstance(shape, CircleObstacleShape):
        return Circle(shape.radius)
    if isinstance(shape, RectObstacleShape):
        return Rectangle(shape.length, shape.width, center=(-shape.origin_x_shift, 0.0))
    if isinstance(shape, PolygonObstacleShape):
        return Polygon([tuple(v) for v in shape.vertices], [])

    return commonroad_occupancy(shape.compute_occupancy_for_state(InitialState(position=(0.0, 0.0), orientation=0.0)))


def commonroad_occupancy(occupancy: Occupancy) -> CollisionObject:
    """Converts a CommonRoad occupancy to a world-positioned crcc CollisionObject."""
    if isinstance(occupancy, CircleOccupancy):
        return Circle(occupancy.radius, (occupancy.circle_center.x, occupancy.circle_center.y))
    if isinstance(occupancy, RectOccupancy):
        return Rectangle(
            occupancy.length,
            occupancy.width,
            occupancy.orientation,
            (occupancy.rect_center.x, occupancy.rect_center.y),
        )
    if isinstance(occupancy, PolygonOccupancy):
        return commonroad_polygon(occupancy.shapely_object)

    return shapely_geometry(occupancy.shapely_object)


def shapely_geometry(geometry: BaseGeometry) -> CollisionObject:
    """Convert an empty, Polygon, or MultiPolygon Shapely geometry."""
    if geometry.is_empty:
        return Compound([])
    if isinstance(geometry, ShapelyPolygon):
        return commonroad_polygon(geometry)
    if isinstance(geometry, MultiPolygon):
        return Compound([commonroad_polygon(polygon) for polygon in geometry.geoms])
    raise ValueError(f"Unknown occupancy geometry type {type(geometry)}")


def commonroad_state_to_pose(state: TraceState) -> Pose:
    """Converts a CommonRoad state to a crcc Pose."""
    return Pose(
        translation=(state.position[0], state.position[1]),
        angle=state.orientation,
    )
