from __future__ import annotations

from typing import Sequence

from crcc._core.dynamic_obstacle import DynamicObstacle as _DynamicObstacle
from crcc.collision_object import CollisionObject
from crcc.pose import Pose

__all__ = ["DynamicObstacle"]

class DynamicObstacle(_DynamicObstacle):
    def __init__(self, shape: CollisionObject, positions: Sequence[Pose], time_offset: int) -> None: ...
    @staticmethod
    def from_time_variant(
        obstacles: Sequence[CollisionObject],
        time_offset: int = 0,
        positions: Sequence[Pose] | None = None,
    ) -> DynamicObstacle: ...
