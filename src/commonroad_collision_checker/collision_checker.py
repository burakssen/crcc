from __future__ import annotations

from commonroad.geometry.shape import Circle, Polygon, Rectangle, Shape, ShapeGroup
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import StaticObstacle

import commonroad_collision_checker._core.collision_checker as core_cc
import commonroad_collision_checker._core.collision_object as core_co
import commonroad_collision_checker._core.isometry as core_iso


class CollisionCheckerBuilder:
    _rust_builder: core_cc.CollisionCheckerBuilder

    def __init__(self) -> None:
        self._rust_builder = core_cc.CollisionCheckerBuilder()

    def with_static_obstacle(
        self,
        shape: core_co.Shape,
        position: core_iso.Isometry,
    ) -> CollisionCheckerBuilder:
        self._rust_builder.with_static_obstacle(shape, position)
        return self

    def with_commonroad_static_obstacle(self, static_obstacle: StaticObstacle) -> CollisionCheckerBuilder:
        initial_time = static_obstacle.initial_state.time_step
        occupancy = static_obstacle.occupancy_at_time(initial_time)
        return self.with_commonroad_shape(occupancy.shape)

    def with_commonroad_shape(self, shape: Shape) -> CollisionCheckerBuilder:
        if isinstance(shape, Circle):
            self._rust_builder.with_static_obstacle(
                core_co.Circle(shape.radius), core_iso.Isometry.translation(tuple(shape.center))
            )
        elif isinstance(shape, Rectangle):
            self._rust_builder.with_static_obstacle(
                core_co.Rectangle(shape.length, shape.width), core_iso.Isometry(tuple(shape.center), shape.orientation)
            )
        elif isinstance(shape, Polygon):
            self._rust_builder.with_static_obstacle(
                core_co.Polygon([tuple(v) for v in shape.vertices], []), core_iso.Isometry.identity()
            )
        elif isinstance(shape, ShapeGroup):
            for s in shape.shapes:
                self.with_commonroad_shape(s)
        else:
            raise ValueError(f"Unknown shape type {type(shape)}")
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


class CollisionChecker:
    _rust_cc: core_cc.CollisionChecker

    def __init__(self, rust_cc: core_cc.CollisionChecker) -> None:
        self._rust_cc = rust_cc

    def collides_static(
        self,
        shape: core_cc.Shape,
        position: core_iso.Isometry,
    ) -> bool:
        return self._rust_cc.collides_static(
            shape,
            position,
        )
