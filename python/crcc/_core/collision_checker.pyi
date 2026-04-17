from __future__ import annotations

from typing import List, Optional, Tuple

from .collision_object import CollisionObject
from .dynamic_obstacle import DynamicObstacle
from .pose import Pose

class CollisionEngine:
    Parry: CollisionEngine
    Rhusics: CollisionEngine

class CollisionStatus:
    @staticmethod
    def NoCollision() -> CollisionStatus: ...
    @staticmethod
    def CollidesStatic() -> CollisionStatus: ...
    @staticmethod
    def CollidesDynamic(t: int) -> CollisionStatus: ...
    @property
    def collides(self) -> bool: ...
    @property
    def time_step(self) -> Optional[int]: ...
    def __str__(self) -> str: ...

class CollisionChecker:
    def collides_static(
        self,
        static_obstacle: CollisionObject,
        position: Optional[Pose] = None,
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> CollisionStatus: ...
    def par_collides_static(
        self,
        positioned_static_obstacle: List[Tuple[CollisionObject, Pose]],
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> List[CollisionStatus]: ...
    def collides_dynamic(
        self,
        dynamic_obstacle: DynamicObstacle,
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> CollisionStatus: ...
    def par_collides_dynamic(
        self,
        dynamic_obstacles: List[DynamicObstacle],
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> List[CollisionStatus]: ...

class CollisionCheckerBuilder:
    def __init__(self) -> None: ...
    def with_static_obstacle(self, collision_object: CollisionObject) -> CollisionCheckerBuilder: ...
    def with_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder: ...
    def with_road_boundary_obstacle(self, lanelets: List[List[Tuple[float, float]]]) -> CollisionCheckerBuilder: ...
    def build(self, engine: Optional[CollisionEngine] = None) -> CollisionChecker: ...

__all__ = ["CollisionEngine", "CollisionStatus", "CollisionChecker", "CollisionCheckerBuilder"]
