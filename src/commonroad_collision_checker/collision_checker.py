from __future__ import annotations

from commonroad.geometry.shape import Circle, Polygon, Rectangle, Shape, ShapeGroup
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import StaticObstacle

import commonroad_collision_checker._core.collision_checker as core_cc
import commonroad_collision_checker._core.collision_object as core_co


class CollisionCheckerBuilder:
    _rust_builder: core_cc.CollisionCheckerBuilder

    def __init__(self) -> None:
        self._rust_builder = core_cc.CollisionCheckerBuilder()

    def with_static_obstacle(
        self,
        static_obstacle: core_co.CollisionObject,
    ) -> CollisionCheckerBuilder:
        self._rust_builder.with_static_obstacle(static_obstacle)
        return self

    def with_commonroad_static_obstacle(self, static_obstacle: StaticObstacle) -> CollisionCheckerBuilder:
        initial_time = static_obstacle.initial_state.time_step
        occupancy = static_obstacle.occupancy_at_time(initial_time)
        return self.with_commonroad_shape(occupancy.shape)

    def with_commonroad_shape(self, shape: Shape) -> CollisionCheckerBuilder:
        co = core_co.CollisionObject(_commonroad_shape_to_simple_collision_objects(shape))
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


def _commonroad_shape_to_simple_collision_objects(shape: Shape) -> list[core_co.SimpleCollisionObject]:
    if isinstance(shape, Circle):
        return [core_co.Circle(shape.radius, tuple(shape.center))]
    elif isinstance(shape, Rectangle):
        # TODO: consider orientation
        return [core_co.Rectangle(shape.length, shape.width, tuple(shape.center))]
    elif isinstance(shape, Polygon):
        return [core_co.Polygon([tuple(v) for v in shape.vertices], [])]
    elif isinstance(shape, ShapeGroup):
        return [obj for s in shape.shapes for obj in _commonroad_shape_to_simple_collision_objects(s)]
    else:
        raise ValueError(f"Unknown shape type {type(shape)}")
