from __future__ import annotations

from collections.abc import Sequence

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
    def time_step(self) -> int | None: ...
    def __str__(self) -> str: ...

class CollisionChecker:
    """Immutable static and dynamic collision scene."""

    @property
    def engine(self) -> CollisionEngine: ...
    def collides_static(
        self,
        static_obstacle: CollisionObject,
        position: Pose | None = None,
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> CollisionStatus: ...
    def collides_static_batch(
        self,
        positioned_static_obstacle: Sequence[tuple[CollisionObject, Pose]],
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> list[CollisionStatus]: ...
    def par_static(
        self,
        positioned_static_obstacle: Sequence[tuple[CollisionObject, Pose]],
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> list[CollisionStatus]: ...
    def _collides_static_batch_threads(
        self,
        positioned_static_obstacle: Sequence[tuple[CollisionObject, Pose]],
        threads: int,
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> list[CollisionStatus]: ...
    def par_static_threads(
        self,
        positioned_static_obstacle: Sequence[tuple[CollisionObject, Pose]],
        threads: int,
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> list[CollisionStatus]: ...
    def collides_dynamic(
        self,
        dynamic_obstacle: DynamicObstacle,
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> CollisionStatus: ...
    def collides_dynamic_batch(
        self,
        dynamic_obstacles: Sequence[DynamicObstacle],
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> list[CollisionStatus]: ...
    def par_dynamic(
        self,
        dynamic_obstacles: Sequence[DynamicObstacle],
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> list[CollisionStatus]: ...

class CollisionCheckerBuilder:
    """Fluent builder for an immutable CollisionChecker."""

    def __init__(self, engine: CollisionEngine | None = None) -> None: ...
    def with_engine(self, engine: CollisionEngine) -> CollisionCheckerBuilder: ...
    def with_static_obstacle(self, collision_object: CollisionObject) -> CollisionCheckerBuilder: ...
    def with_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder: ...
    def with_road_boundary(self, lanelets: Sequence[Sequence[tuple[float, float]]]) -> CollisionCheckerBuilder: ...
    def build(self, engine: CollisionEngine | None = None) -> CollisionChecker: ...

def road_boundary(lanelets: Sequence[Sequence[tuple[float, float]]]) -> CollisionObject: ...

__all__ = [
    "CollisionChecker",
    "CollisionCheckerBuilder",
    "CollisionEngine",
    "CollisionStatus",
    "road_boundary",
]
