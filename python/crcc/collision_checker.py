"""Collision checker construction, results, and engine selection."""

from __future__ import annotations

import crcc._core.collision_checker as core
from crcc.collision_object import CollisionObject
from crcc.dynamic_obstacle import DynamicObstacle

CollisionStatus = core.CollisionStatus
CollisionChecker = core.CollisionChecker
CollisionEngine = core.CollisionEngine
PreparedDynamicQuery = core.PreparedDynamicQuery
PreparedStaticQuery = core.PreparedStaticQuery
road_boundary = core.road_boundary


class CollisionCheckerBuilder:
    """Build a checker using the selected collision engine."""

    def __init__(self, engine: core.CollisionEngine | None = None) -> None:
        """Initializes an empty CollisionCheckerBuilder with the specified engine."""
        self._rust_builder = core.CollisionCheckerBuilder()
        self._engine = engine

    def with_engine(self, engine: core.CollisionEngine) -> CollisionCheckerBuilder:
        """Sets the collision backend engine to use for checker queries."""
        self._engine = engine
        return self

    def with_static_obstacle(self, query_shape: CollisionObject) -> CollisionCheckerBuilder:
        """Adds a static shape obstacle to the collision scene."""
        self._rust_builder.with_static_obstacle(query_shape)
        return self

    def with_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder:
        """Adds a dynamic moving obstacle trajectory to the collision scene."""
        self._rust_builder.with_dynamic_obstacle(dynamic_obstacle)
        return self

    def with_road_boundary(self, lanelets: list[list[tuple[float, float]]]) -> CollisionCheckerBuilder:
        """Adds road boundaries based on lanelet polygons to the static obstacle geometry."""
        self._rust_builder.with_road_boundary(lanelets)
        return self

    def build(self) -> core.CollisionChecker:
        """Builds and returns an immutable CollisionChecker with the configured scene."""
        return self._rust_builder.build(self._engine)


__all__ = [
    "CollisionChecker",
    "CollisionCheckerBuilder",
    "CollisionEngine",
    "CollisionStatus",
    "PreparedDynamicQuery",
    "PreparedStaticQuery",
    "road_boundary",
]
