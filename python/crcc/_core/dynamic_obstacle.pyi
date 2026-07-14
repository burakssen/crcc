from typing import Sequence

from .collision_object import CollisionObject
from .pose import Pose

class DynamicObstacle:
    """Discrete obstacle trajectory with conservative motion between steps."""

    def __init__(self, shape: CollisionObject, positions: Sequence[Pose], time_offset: int) -> None: ...
    @staticmethod
    def from_time_variant(
        obstacles: Sequence[CollisionObject], time_offset: int = 0, positions: Sequence[Pose] | None = None
    ) -> DynamicObstacle: ...

__all__ = ["DynamicObstacle"]
