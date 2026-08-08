from __future__ import annotations

from numbers import Real
from typing import Any, cast

import commonroad.scenario.obstacle as cr_obstacle
import numpy as np
from commonroad.common.util import Interval
from commonroad.geometry.obstacle_shapes.circle_obstacle_shape import CircleObstacleShape
from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.obstacle_shapes.polygon_obstacle_shape import PolygonObstacleShape
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState, TraceState
from shapely.geometry import MultiPolygon, Polygon as ShapelyPolygon
from shapely.geometry.base import BaseGeometry

import crcc._core.collision_checker as core
from crcc import Circle, CollisionCheckerBuilder, CollisionObject, Compound, DynamicObstacle, Polygon, Pose, Rectangle

ROAD_BOUNDARY_SIMPLIFY_TOLERANCE = 0.01
ROAD_BOUNDARY_MIN_HOLE_AREA = 0.001


def _exact_time_step(value: object) -> int:
    if not isinstance(value, int):
        raise ValueError(f"Expected an exact integer time step, got {value!r}")
    return value


def _point(vertex: Any) -> tuple[float, float]:
    v: Any = vertex
    return float(v[0]), float(v[1])


def scenario_builder(
    scenario: Scenario,
    builder: CollisionCheckerBuilder | None = None,
) -> CollisionCheckerBuilder:
    """Creates a collision checker builder from a CommonRoad scenario."""
    if builder is None:
        builder = CollisionCheckerBuilder()

    builder = add_road_boundary(builder, scenario.lanelet_network)
    for static_obstacle in scenario.static_obstacles:
        builder = add_static_obstacle(builder, static_obstacle)
    for dynamic_obstacle in scenario.dynamic_obstacles:
        builder = add_dynamic_obstacle(builder, dynamic_obstacle)
    return builder


def add_static_obstacle(
    builder: CollisionCheckerBuilder,
    static_obstacle: cr_obstacle.StaticObstacle,
) -> CollisionCheckerBuilder:
    """Adds a CommonRoad static obstacle to the builder."""
    collision_object = from_occupancy(
        static_obstacle.occupancy_at_time(_exact_time_step(static_obstacle.initial_state.time_step))
    )
    builder.with_static_obstacle(collision_object)
    return builder


def add_dynamic_obstacle(
    builder: CollisionCheckerBuilder,
    dynamic_obstacle: cr_obstacle.DynamicObstacle,
) -> CollisionCheckerBuilder:
    """Adds a CommonRoad dynamic obstacle to the builder."""
    builder.with_dynamic_obstacle(from_dynamic_obstacle(dynamic_obstacle))
    return builder


def from_dynamic_obstacle(dynamic_obstacle: cr_obstacle.DynamicObstacle) -> DynamicObstacle:
    """Converts a CommonRoad dynamic obstacle to a local crcc DynamicObstacle."""
    initial_time = _exact_time_step(dynamic_obstacle.initial_state.time_step)
    prediction: Any = dynamic_obstacle.prediction
    if isinstance(prediction, TrajectoryPrediction):
        final_time = _prediction_final_time_step(prediction)
        states = {_exact_time_step(state.time_step): state for state in prediction.trajectory.state_list}
        initial_shape = from_shape(dynamic_obstacle.obstacle_shape)
        predicted_shape = from_shape(prediction.shape)
        obstacles: list[CollisionObject] = []
        positions: list[Pose] = []

        for time_step in range(initial_time, final_time + 1):
            if time_step == initial_time:
                obstacles.append(initial_shape)
                positions.append(from_pose(dynamic_obstacle.initial_state))
            elif (state := states.get(time_step)) is not None:
                obstacles.append(predicted_shape)
                positions.append(from_pose(state))
            else:
                obstacles.append(Compound([]))
                positions.append(Pose.identity())

        return DynamicObstacle.from_time_variant(obstacles, initial_time, positions)

    final_time = initial_time if prediction is None else _prediction_final_time_step(prediction)
    if final_time < initial_time:
        raise ValueError("CommonRoad prediction ends before the obstacle's initial state")

    obstacles = [
        from_occupancy(occupancy) if (occupancy := dynamic_obstacle.occupancy_at_time(time_step)) else Compound([])
        for time_step in range(initial_time, final_time + 1)
    ]
    return DynamicObstacle.from_time_variant(obstacles, initial_time)


def _prediction_final_time_step(prediction: Any) -> int:
    final_time_step = prediction.final_time_step
    if isinstance(final_time_step, Interval):
        final_time_step = final_time_step.end
    return _exact_time_step(final_time_step)


def add_road_boundary(
    builder: CollisionCheckerBuilder,
    lanelet_network: LaneletNetwork,
) -> CollisionCheckerBuilder:
    """Adds the road boundary from a lanelet network to the builder."""
    if lanelet_network.lanelets:
        # An absent map supplies no road-boundary constraint.
        builder.with_static_obstacle(road_boundary(lanelet_network))
    return builder


def road_boundary(lanelet_network: LaneletNetwork) -> CollisionObject:
    """Creates an obstacle for all space outside the lanelet network."""
    lanelets = [[_point(vertex) for vertex in lanelet.polygon.vertices] for lanelet in lanelet_network.lanelets]
    return core.road_boundary(lanelets)


def from_polygon(polygon: ShapelyPolygon) -> CollisionObject:
    """Convert a Shapely polygon, including holes, to a collision object."""
    return Polygon(
        exterior=[_point(vertex) for vertex in polygon.exterior.coords],
        interiors=[[_point(vertex) for vertex in interior.coords] for interior in polygon.interiors],
    )


def from_shape(shape: ObstacleShape) -> CollisionObject:
    """Converts a CommonRoad obstacle shape to a local crcc CollisionObject."""
    if isinstance(shape, CircleObstacleShape):
        return Circle(shape.radius)
    if isinstance(shape, RectObstacleShape):
        return Rectangle(shape.length, shape.width, center=(-shape.origin_x_shift, 0.0))
    if isinstance(shape, PolygonObstacleShape):
        return Polygon([_point(vertex) for vertex in shape.vertices], [])

    zero_state = InitialState(position=np.array((0.0, 0.0)), orientation=0.0)
    return from_occupancy(shape.compute_occupancy_for_state(zero_state))


def from_occupancy(occupancy: Occupancy) -> CollisionObject:
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
        return from_polygon(occupancy.shapely_object)

    return from_shapely(cast(BaseGeometry, occupancy.shapely_object))


def from_shapely(geometry: BaseGeometry) -> CollisionObject:
    """Convert an empty, Polygon, or MultiPolygon Shapely geometry."""
    if geometry.is_empty:
        return Compound([])
    if isinstance(geometry, ShapelyPolygon):
        return from_polygon(geometry)
    if isinstance(geometry, MultiPolygon):
        return Compound([from_polygon(polygon) for polygon in geometry.geoms])
    raise ValueError(f"Unknown occupancy geometry type {type(geometry)}")


def from_pose(state: TraceState) -> Pose:
    """Converts a CommonRoad state to a crcc Pose."""
    position = getattr(state, "position", None)
    orientation = getattr(state, "orientation", None)
    if not isinstance(position, np.ndarray) or position.shape != (2,):
        raise ValueError("CommonRoad state requires an exact two-dimensional position")
    if not isinstance(orientation, Real):
        raise ValueError("CommonRoad state requires an exact orientation")
    return Pose(translation=(float(position[0]), float(position[1])), angle=float(orientation))
