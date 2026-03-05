from __future__ import annotations

import commonroad.geometry.shape as cr_shape
import commonroad.scenario.obstacle as cr_obstacle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import TraceState

import crcc._core.collision_checker as core
from crcc.collision_object import Circle, CollisionObject, Compound, Polygon, Rectangle
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose

# explicitly re-export classes to define the public API of this module
# this enables us to add wrappers for the Rust objects later as a non-breaking change
CollisionStatus = core.CollisionStatus
CollisionChecker = core.CollisionChecker


class CollisionCheckerBuilder:
    _rust_builder: core.CollisionCheckerBuilder

    def __init__(self) -> None:
        self._rust_builder = core.CollisionCheckerBuilder()

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

    def with_commonroad_scenario(self, scenario: Scenario) -> CollisionCheckerBuilder:
        self.with_road_boundary_obstacle(scenario.lanelet_network)
        for static_obstacle in scenario.static_obstacles:
            self.with_commonroad_static_obstacle(static_obstacle)
        for dynamic_obstacle in scenario.dynamic_obstacles:
            self.with_commonroad_dynamic_obstacle(dynamic_obstacle)
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

    def build(self) -> core.CollisionChecker:
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
