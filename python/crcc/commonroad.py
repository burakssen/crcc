from __future__ import annotations

import commonroad.geometry.shape as cr_shape
import commonroad.scenario.obstacle as cr_obstacle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import TraceState

from crcc.collision_checker import CollisionCheckerBuilder
from crcc.collision_object import Circle, CollisionObject, Compound, Polygon, Rectangle
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose


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
    co = commonroad_shape_to_collision_object(static_obstacle.obstacle_shape)
    builder.with_static_obstacle(co)
    return builder


def add_commonroad_dynamic_obstacle_to_builder(
    builder: CollisionCheckerBuilder,
    dynamic_obstacle: cr_obstacle.DynamicObstacle,
) -> CollisionCheckerBuilder:
    """Adds a CommonRoad dynamic obstacle to the builder."""
    initial_time = dynamic_obstacle.initial_state.time_step
    if isinstance(dynamic_obstacle.prediction, TrajectoryPrediction):
        trajectory = dynamic_obstacle.prediction.trajectory
        states = [dynamic_obstacle.initial_state] + trajectory.state_list
        poses = [commonroad_state_to_pose(state) for state in states]
        shape = commonroad_shape_to_collision_object(dynamic_obstacle.obstacle_shape)
        rust_dynamic_obstacle = DynamicObstacle(shape, poses, initial_time)
        builder.with_dynamic_obstacle(rust_dynamic_obstacle)
    else:
        raise NotImplementedError("Only TrajectoryPrediction is supported for dynamic obstacles.")
    return builder


def add_road_boundary_to_builder(
    builder: CollisionCheckerBuilder,
    lanelet_network: LaneletNetwork,
) -> CollisionCheckerBuilder:
    """Adds the road boundary from a lanelet network to the builder."""
    builder.with_road_boundary_obstacle(
        [[(v[0], v[1]) for v in lanelet.polygon.vertices] for lanelet in lanelet_network.lanelets]
    )
    return builder


def commonroad_shape_to_collision_object(shape: cr_shape.Shape) -> CollisionObject:
    """Converts a CommonRoad shape to a crcc CollisionObject."""
    objs = commonroad_shape_to_collision_objects(shape)
    if len(objs) == 1:
        return objs[0]
    else:
        return Compound(objs)


def commonroad_shape_to_collision_objects(shape: cr_shape.Shape) -> list[CollisionObject]:
    """Converts a CommonRoad shape to a list of crcc CollisionObjects."""
    if isinstance(shape, cr_shape.Circle):
        return [Circle(shape.radius, tuple(shape.center))]
    elif isinstance(shape, cr_shape.Rectangle):
        return [Rectangle(shape.length, shape.width, shape.orientation, tuple(shape.center))]
    elif isinstance(shape, cr_shape.Polygon):
        return [Polygon([tuple(v) for v in shape.vertices], [])]
    elif isinstance(shape, cr_shape.ShapeGroup):
        return [obj for s in shape.shapes for obj in commonroad_shape_to_collision_objects(s)]
    else:
        raise ValueError(f"Unknown shape type {type(shape)}")


def commonroad_state_to_pose(state: TraceState) -> Pose:
    """Converts a CommonRoad state to a crcc Pose."""
    return Pose(
        translation=(state.position[0], state.position[1]),
        angle=state.orientation,
    )
