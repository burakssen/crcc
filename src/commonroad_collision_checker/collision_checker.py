from __future__ import annotations

from commonroad.scenario.lanelet import LaneletNetwork

# import commonroad_collision_checker._core as core_cc
import commonroad_collision_checker._core.collision_checker as core_cc
import commonroad_collision_checker._core.isometry as core_isometry


class CollisionCheckerBuilder:
    _rust_builder: core_cc.CollisionCheckerBuilder

    def __init__(self) -> None:
        self._rust_builder = core_cc.CollisionCheckerBuilder()

    def with_static_obstacle(
        self,
        shape: core_cc.Shape,
        position: core_isometry.Isometry,
    ) -> CollisionCheckerBuilder:
        self._rust_builder.with_static_obstacle(shape, position)
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
        position: core_isometry.Isometry,
    ) -> bool:
        return self._rust_cc.collides_static(
            shape,
            position,
        )
