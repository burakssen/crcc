from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

from .collision_object import CollisionObject
from .dynamic_obstacle import DynamicObstacle
from .pose import Pose

class CollisionEngine:
    """Runtime collision backend selector."""

    Parry: CollisionEngine
    Rhusics: CollisionEngine
    Collide: CollisionEngine

class CollisionStatus:
    """Checker result identifying no, static, or first dynamic collision."""

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
    """Immutable static and dynamic collision scene."""

    @property
    def engine(self) -> CollisionEngine: ...
    def collides_static(
        self,
        static_obstacle: CollisionObject,
        position: Optional[Pose] = None,
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> CollisionStatus: ...
    def collides_static_batch(
        self,
        positioned_static_obstacle: Sequence[Tuple[CollisionObject, Pose]],
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> List[CollisionStatus]: ...
    def par_static(
        self,
        positioned_static_obstacle: Sequence[Tuple[CollisionObject, Pose]],
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> List[CollisionStatus]: ...
    def _collides_static_batch_threads(
        self,
        positioned_static_obstacle: Sequence[Tuple[CollisionObject, Pose]],
        threads: int,
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> List[CollisionStatus]: ...
    def par_static_threads(
        self,
        positioned_static_obstacle: Sequence[Tuple[CollisionObject, Pose]],
        threads: int,
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> List[CollisionStatus]: ...
    def collides_dynamic(
        self,
        dynamic_obstacle: DynamicObstacle,
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> CollisionStatus: ...
    def collides_dynamic_batch(
        self,
        dynamic_obstacles: Sequence[DynamicObstacle],
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> List[CollisionStatus]: ...
    def par_dynamic(
        self,
        dynamic_obstacles: Sequence[DynamicObstacle],
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> List[CollisionStatus]: ...

class CollisionCheckerBuilder:
    """Fluent builder for an immutable CollisionChecker."""

    def __init__(self, engine: Optional[CollisionEngine] = None) -> None: ...
    def with_engine(self, engine: CollisionEngine) -> CollisionCheckerBuilder: ...
    def with_static_obstacle(self, collision_object: CollisionObject) -> CollisionCheckerBuilder: ...
    def with_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder: ...
    def with_road_boundary(self, lanelets: Sequence[Sequence[Tuple[float, float]]]) -> CollisionCheckerBuilder: ...
    def build(self, engine: Optional[CollisionEngine] = None) -> CollisionChecker: ...

def road_boundary(lanelets: Sequence[Sequence[Tuple[float, float]]]) -> CollisionObject: ...

__all__ = [
    "CollisionEngine",
    "CollisionStatus",
    "CollisionChecker",
    "CollisionCheckerBuilder",
    "road_boundary",
]
