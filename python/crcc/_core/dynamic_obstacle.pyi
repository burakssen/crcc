from typing import List

from .collision_object import CollisionObject
from .pose import Pose

class DynamicObstacle:
    def __init__(self, shape: CollisionObject, positions: List[Pose], time_offset: int) -> None: ...

__all__ = ["DynamicObstacle"]
