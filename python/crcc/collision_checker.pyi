from __future__ import annotations

from typing import List, Tuple

import crcc._core.collision_checker as core
from crcc.collision_object import CollisionObject
from crcc.dynamic_obstacle import DynamicObstacle

CollisionStatus = core.CollisionStatus
CollisionChecker = core.CollisionChecker
CollisionEngine = core.CollisionEngine

class CollisionCheckerBuilder:
    def __init__(self, engine: core.CollisionEngine = core.CollisionEngine.Parry) -> None: ...
    def with_engine(self, engine: core.CollisionEngine) -> CollisionCheckerBuilder: ...
    def with_static_obstacle(self, static_obstacle: CollisionObject) -> CollisionCheckerBuilder: ...
    def with_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder: ...
    def with_road_boundary_obstacle(
        self,
        lanelets: List[List[Tuple[float, float]]],
    ) -> CollisionCheckerBuilder: ...
    def build(self) -> core.CollisionChecker: ...

def create_road_boundary_obstacle(lanelets: List[List[Tuple[float, float]]]) -> CollisionObject: ...

__all__ = [
    "CollisionStatus",
    "CollisionChecker",
    "CollisionEngine",
    "CollisionCheckerBuilder",
    "create_road_boundary_obstacle",
]
