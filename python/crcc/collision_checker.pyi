from __future__ import annotations

from typing import Sequence, Tuple

from crcc._core.collision_checker import CollisionChecker, CollisionEngine, CollisionStatus
from crcc.collision_object import CollisionObject
from crcc.dynamic_obstacle import DynamicObstacle

class CollisionCheckerBuilder:
    def __init__(self, engine: CollisionEngine = CollisionEngine.Parry) -> None: ...
    def with_engine(self, engine: CollisionEngine) -> CollisionCheckerBuilder: ...
    def with_static_obstacle(self, query_shape: CollisionObject) -> CollisionCheckerBuilder: ...
    def with_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder: ...
    def with_road_boundary(
        self,
        lanelets: Sequence[Sequence[Tuple[float, float]]],
    ) -> CollisionCheckerBuilder: ...
    def build(self) -> CollisionChecker: ...

def road_boundary(lanelets: Sequence[Sequence[Tuple[float, float]]]) -> CollisionObject: ...

__all__ = [
    "CollisionStatus",
    "CollisionChecker",
    "CollisionEngine",
    "CollisionCheckerBuilder",
    "road_boundary",
]
