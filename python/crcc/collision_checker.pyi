from __future__ import annotations

from collections.abc import Sequence

from crcc._core.collision_checker import (
    CollisionBackend,
    CollisionChecker,
    CollisionStatus,
    PreparedDynamicQuery,
    PreparedStaticQuery,
)
from crcc.collision_object import CollisionObject
from crcc.dynamic_obstacle import DynamicObstacle

CollisionEngine = CollisionBackend

class CollisionCheckerBuilder:
    def __init__(
        self,
        backend: CollisionBackend | None = None,
        *,
        engine: CollisionBackend | None = None,
    ) -> None: ...
    def add_static_obstacle(self, query_shape: CollisionObject) -> CollisionCheckerBuilder: ...
    def add_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder: ...
    def build(self) -> CollisionChecker: ...

    # Deprecated aliases retained for one release.
    def with_engine(self, engine: CollisionBackend) -> CollisionCheckerBuilder: ...
    def with_static_obstacle(self, query_shape: CollisionObject) -> CollisionCheckerBuilder: ...
    def with_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder: ...
    def with_road_boundary(
        self,
        lanelets: Sequence[Sequence[tuple[float, float]]],
    ) -> CollisionCheckerBuilder: ...

def road_boundary(lanelets: Sequence[Sequence[tuple[float, float]]]) -> CollisionObject: ...

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
