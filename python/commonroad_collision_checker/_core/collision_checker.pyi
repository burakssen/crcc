from __future__ import annotations

from typing import List, Optional, Tuple

from .collision_object import CollisionObject
from .dynamic_obstacle import DynamicObstacle
from .pose import Pose

class CollisionStatus:
    @staticmethod
    def NoCollision() -> CollisionStatus: ...
    @staticmethod
    def CollidesStatic() -> CollisionStatus: ...
    @staticmethod
    def CollidesDynamic(t: int) -> CollisionStatus: ...
    def collides(self) -> bool: ...
    def __str__(self) -> str: ...

class CollisionChecker:
    def collides_static(
        self,
        static_obstacle: CollisionObject,
        position: Optional[Pose] = None,
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> CollisionStatus: ...
    def collides_dynamic(
        self,
        dynamic_obstacle: DynamicObstacle,
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> CollisionStatus: ...

class CollisionCheckerBuilder:
    def __init__(self) -> None: ...
    def with_static_obstacle(self, collision_object: CollisionObject) -> CollisionCheckerBuilder: ...
    def with_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder: ...
    def with_road_boundary_obstacle(self, lanelets: List[List[Tuple[float, float]]]) -> CollisionCheckerBuilder: ...
    def build(self) -> CollisionChecker: ...

__all__ = ["CollisionStatus", "CollisionChecker", "CollisionCheckerBuilder"]
