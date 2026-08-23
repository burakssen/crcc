"""Collision checker construction, results, and backend selection."""

from __future__ import annotations

import warnings
from collections.abc import Sequence

import crcc._core.collision_checker as core
from crcc.collision_object import CollisionObject
from crcc.dynamic_obstacle import DynamicObstacle

CollisionStatus = core.CollisionStatus
CollisionChecker = core.CollisionChecker
CollisionBackend = core.CollisionBackend
CollisionEngine = core.CollisionBackend
PreparedDynamicQuery = core.PreparedDynamicQuery
PreparedStaticQuery = core.PreparedStaticQuery
road_boundary = core.road_boundary


class CollisionCheckerBuilder:
    """Build a checker using the selected collision backend."""

    def __init__(
        self, backend: core.CollisionBackend | None = None, *, engine: core.CollisionBackend | None = None
    ) -> None:
        """Initializes an empty builder, optionally selecting the collision backend."""
        if engine is not None:
            warnings.warn(
                "engine= is deprecated; use backend=",
                DeprecationWarning,
                stacklevel=2,
            )
            if backend is not None:
                raise TypeError("specify either backend or engine, not both")
            backend = engine
        self._rust_builder = core.CollisionCheckerBuilder()
        self._backend = backend

    def add_static_obstacle(self, query_shape: CollisionObject) -> CollisionCheckerBuilder:
        """Adds a static shape obstacle to the collision scene."""
        self._rust_builder.with_static_obstacle(query_shape)
        return self

    def add_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder:
        """Adds a dynamic moving obstacle trajectory to the collision scene."""
        self._rust_builder.with_dynamic_obstacle(dynamic_obstacle)
        return self

    def build(self) -> core.CollisionChecker:
        """Builds and returns an immutable CollisionChecker with the configured scene."""
        return self._rust_builder.build(self._backend)

    def with_engine(self, engine: core.CollisionBackend) -> CollisionCheckerBuilder:
        """Deprecated alias for passing ``backend`` to the constructor."""
        warnings.warn(
            "with_engine() is deprecated; pass backend= to the constructor",
            DeprecationWarning,
            stacklevel=2,
        )
        self._backend = engine
        return self

    def with_static_obstacle(self, query_shape: CollisionObject) -> CollisionCheckerBuilder:
        """Deprecated alias for :meth:`add_static_obstacle`."""
        warnings.warn(
            "with_static_obstacle() is deprecated; use add_static_obstacle()",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.add_static_obstacle(query_shape)

    def with_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder:
        """Deprecated alias for :meth:`add_dynamic_obstacle`."""
        warnings.warn(
            "with_dynamic_obstacle() is deprecated; use add_dynamic_obstacle()",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.add_dynamic_obstacle(dynamic_obstacle)

    def with_road_boundary(self, lanelets: Sequence[Sequence[tuple[float, float]]]) -> CollisionCheckerBuilder:
        """Deprecated convenience combining ``road_boundary`` and ``add_static_obstacle``."""
        warnings.warn(
            "with_road_boundary() is deprecated; use add_static_obstacle(road_boundary(lanelets))",
            DeprecationWarning,
            stacklevel=2,
        )
        converted = [list(exterior) for exterior in lanelets]
        self._rust_builder.with_road_boundary(converted)
        return self


__all__ = [
    "CollisionBackend",
    "CollisionChecker",
    "CollisionCheckerBuilder",
    "CollisionEngine",
    "CollisionStatus",
    "PreparedDynamicQuery",
    "PreparedStaticQuery",
    "road_boundary",
]
