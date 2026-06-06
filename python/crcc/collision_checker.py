from __future__ import annotations

import crcc._core.collision_checker as core
from crcc.collision_object import CollisionObject
from crcc.dynamic_obstacle import DynamicObstacle

# explicitly re-export classes to define the public API of this module
# this enables us to add wrappers for the Rust objects later as a non-breaking change
CollisionStatus = core.CollisionStatus
CollisionChecker = core.CollisionChecker
CollisionEngine = core.CollisionEngine
create_road_boundary_obstacle = core.create_road_boundary_obstacle


class CollisionCheckerBuilder:
    _rust_builder: core.CollisionCheckerBuilder
    _engine: core.CollisionEngine

    def __init__(self, engine: core.CollisionEngine = core.CollisionEngine.Parry) -> None:
        self._rust_builder = core.CollisionCheckerBuilder()
        self._engine = engine

    def with_engine(self, engine: core.CollisionEngine) -> CollisionCheckerBuilder:
        self._engine = engine
        return self

    def with_static_obstacle(
        self,
        query_shape: CollisionObject,
    ) -> CollisionCheckerBuilder:
        self._rust_builder.with_static_obstacle(query_shape)
        return self

    def with_dynamic_obstacle(
        self,
        dynamic_obstacle: DynamicObstacle,
    ) -> CollisionCheckerBuilder:
        self._rust_builder.with_dynamic_obstacle(dynamic_obstacle)
        return self

    def with_road_boundary_obstacle(
        self,
        lanelets: list[list[tuple[float, float]]],
    ) -> CollisionCheckerBuilder:
        self._rust_builder.with_road_boundary_obstacle(lanelets)
        return self

    def build(self) -> core.CollisionChecker:
        return self._rust_builder.build(self._engine)
