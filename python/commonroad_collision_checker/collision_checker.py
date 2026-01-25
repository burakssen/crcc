from __future__ import annotations

import commonroad.geometry.shape as cr_shape
import commonroad.scenario.obstacle as cr_obstacle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import TraceState

import commonroad_collision_checker._core.collision_checker as core_cc
from commonroad_collision_checker._core.collision_checker import (  # noqa: F401
    CollisionChecker as CollisionChecker,
    CollisionStatus as CollisionStatus,
)
from commonroad_collision_checker.collision_object import Circle, CollisionObject, Compound, Polygon, Rectangle
from commonroad_collision_checker.dynamic_obstacle import DynamicObstacle
from commonroad_collision_checker.pose import Pose


class CollisionCheckerBuilder:
    _rust_builder: core_cc.CollisionCheckerBuilder

    def __init__(self) -> None:
        self._rust_builder = core_cc.CollisionCheckerBuilder()

    def with_static_obstacle(
        self,
        static_obstacle: CollisionObject,
    ) -> CollisionCheckerBuilder:
        self._rust_builder.with_static_obstacle(static_obstacle)
        return self

    def with_dynamic_obstacle(
        self,
        dynamic_obstacle: DynamicObstacle,
    ) -> CollisionCheckerBuilder:
        self._rust_builder.with_dynamic_obstacle(dynamic_obstacle)
        return self

    def with_commonroad_static_obstacle(self, static_obstacle: cr_obstacle.StaticObstacle) -> CollisionCheckerBuilder:
        return self.with_commonroad_shape(static_obstacle.obstacle_shape)

    def with_commonroad_dynamic_obstacle(
        self, dynamic_obstacle: cr_obstacle.DynamicObstacle
    ) -> CollisionCheckerBuilder:
        initial_time = dynamic_obstacle.initial_state.time_step
        if isinstance(dynamic_obstacle.prediction, TrajectoryPrediction):
            trajectory = dynamic_obstacle.prediction.trajectory
            states = [dynamic_obstacle.initial_state] + trajectory.state_list
            poses = [_commonroad_state_to_pose(state) for state in states]
            shape = _commonroad_shape_to_collision_object(dynamic_obstacle.obstacle_shape)
            rust_dynamic_obstacle = DynamicObstacle(shape, poses, initial_time)
            self.with_dynamic_obstacle(rust_dynamic_obstacle)
        else:
            raise NotImplementedError("Only TrajectoryPrediction is supported for dynamic obstacles.")
        return self

    def with_commonroad_shape(self, shape: cr_shape.Shape) -> CollisionCheckerBuilder:
        co = _commonroad_shape_to_collision_object(shape)
        self.with_static_obstacle(co)
        return self

    def with_road_boundary_obstacle(
        self,
        lanelet_network: LaneletNetwork,
    ) -> CollisionCheckerBuilder:
        self._rust_builder.with_road_boundary_obstacle(
            [[(v[0], v[1]) for v in lanelet.polygon.vertices] for lanelet in lanelet_network.lanelets]
        )
        return self

    def build(self) -> core_cc.CollisionChecker:
        return self._rust_builder.build()


def _commonroad_shape_to_collision_object(shape: cr_shape.Shape) -> CollisionObject:
    objs = _commonroad_shape_to_collision_objects(shape)
    if len(objs) == 1:
        return objs[0]
    else:
        return Compound(objs)


def _commonroad_shape_to_collision_objects(shape: cr_shape.Shape) -> list[CollisionObject]:
    if isinstance(shape, cr_shape.Circle):
        return [Circle(shape.radius, tuple(shape.center))]
    elif isinstance(shape, cr_shape.Rectangle):
        return [Rectangle(shape.length, shape.width, shape.orientation, tuple(shape.center))]
    elif isinstance(shape, cr_shape.Polygon):
        return [Polygon([tuple(v) for v in shape.vertices], [])]
    elif isinstance(shape, cr_shape.ShapeGroup):
        return [obj for s in shape.shapes for obj in _commonroad_shape_to_collision_objects(s)]
    else:
        raise ValueError(f"Unknown shape type {type(shape)}")


def _commonroad_state_to_pose(state: TraceState) -> Pose:
    return Pose(
        translation=(state.position[0], state.position[1]),
        angle=state.orientation,
    )
