from __future__ import annotations

from typing import List, Optional, Tuple

from .pose import Pose

class CollisionObject:
    def collides(
        self,
        other: CollisionObject,
        pos_self: Pose = Pose.identity(),
        pos_other: Pose = Pose.identity(),
    ) -> bool: ...
    def collides_continuous(
        self,
        start_pos_self: Pose,
        end_pos_self: Pose,
        other: CollisionObject,
        start_pos_other: Pose,
        end_pos_other: Pose,
    ) -> bool: ...
    def merge(self, other: CollisionObject) -> CollisionObject: ...
    @staticmethod
    def merge_all(collision_objects: List[CollisionObject]) -> CollisionObject: ...

class Compound(CollisionObject):
    def __init__(self, collision_objects: List[CollisionObject]) -> None: ...

class Circle(CollisionObject):
    def __init__(self, radius: float, center: Tuple[float, float] = (0.0, 0.0)) -> None: ...

class Empty(CollisionObject):
    def __init__(self) -> None: ...

class HalfSpace(CollisionObject):
    def __init__(self, outward_normal: Tuple[float, float], offset: float = 0.0) -> None: ...
    @staticmethod
    def from_points(p1: Tuple[float, float], p2: Tuple[float, float]) -> HalfSpace: ...
    @staticmethod
    def from_coeffs(a: float, b: float, c: float = 0.0) -> HalfSpace: ...

class Polygon(CollisionObject):
    def __init__(
        self, exterior: List[Tuple[float, float]], interiors: Optional[List[List[Tuple[float, float]]]] = None
    ) -> None: ...

class Rectangle(CollisionObject):
    def __init__(
        self, length: float, width: float, orientation: float = 0.0, center: Tuple[float, float] = (0.0, 0.0)
    ) -> None: ...

class Triangle(CollisionObject):
    def __init__(self, a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> None: ...

__all__ = [
    "CollisionObject",
    "Compound",
    "Circle",
    "Empty",
    "HalfSpace",
    "Polygon",
    "Rectangle",
    "Triangle",
]
