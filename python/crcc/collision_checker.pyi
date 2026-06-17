from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

from crcc._core.collision_checker import (
    CollisionChecker as _CollisionChecker,
    CollisionEngine as _CollisionEngine,
    CollisionStatus as _CollisionStatus,
)
from crcc.collision_object import CollisionObject
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose

class CollisionEngine(_CollisionEngine):
    Parry: CollisionEngine
    Rhusics: CollisionEngine
    Collide: CollisionEngine

class CollisionStatus(_CollisionStatus):
    @property
    def collides(self) -> bool: ...
    @property
    def time_step(self) -> Optional[int]: ...

class CollisionChecker(_CollisionChecker):
    def collides_static(
        self,
        query_shape: CollisionObject,
        position: Optional[Pose] = None,
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> CollisionStatus: ...
    def par_static(
        self,
        positioned_query_shapes: Sequence[Tuple[CollisionObject, Pose]],
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> List[CollisionStatus]: ...
    def collides_dynamic(
        self,
        dynamic_obstacle: DynamicObstacle,
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> CollisionStatus: ...
    def par_dynamic(
        self,
        dynamic_obstacles: Sequence[DynamicObstacle],
        min_time: Optional[int] = None,
        max_time: Optional[int] = None,
    ) -> List[CollisionStatus]: ...

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
