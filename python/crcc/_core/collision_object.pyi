from __future__ import annotations

from typing import Optional, Sequence, Tuple

from .collision_checker import CollisionEngine
from .pose import Pose

class CollisionObject:
    """Base class for queryable 2D geometry."""

    def collides(
        self,
        other: CollisionObject,
        pos_self: Pose = Pose.identity(),
        pos_other: Pose = Pose.identity(),
        engine: CollisionEngine = CollisionEngine.Parry,
    ) -> bool: ...
    def collides_continuous(
        self,
        start_pos_self: Pose,
        end_pos_self: Pose,
        other: CollisionObject,
        start_pos_other: Pose,
        end_pos_other: Pose,
        engine: CollisionEngine = CollisionEngine.Parry,
    ) -> bool: ...
    def distance(
        self,
        other: CollisionObject,
        pos_self: Pose = Pose.identity(),
        pos_other: Pose = Pose.identity(),
        engine: CollisionEngine = CollisionEngine.Parry,
    ) -> float: ...
    def merge(self, other: CollisionObject) -> CollisionObject: ...
    @staticmethod
    def merge_all(collision_objects: Sequence[CollisionObject]) -> CollisionObject: ...

class Compound(CollisionObject):
    """Union of zero or more collision objects."""

    def __init__(self, collision_objects: Sequence[CollisionObject]) -> None: ...

class Circle(CollisionObject):
    """Circle with a positive radius and optional local-space center."""

    def __init__(self, radius: float, center: Tuple[float, float] = (0.0, 0.0)) -> None: ...

class Empty(CollisionObject):
    """Geometry that never collides."""

    def __init__(self) -> None: ...

class HalfSpace(CollisionObject):
    """Region where outward_normal dot point is at most offset."""

    def __init__(self, outward_normal: Tuple[float, float], offset: float = 0.0) -> None: ...
    @staticmethod
    def from_points(p1: Tuple[float, float], p2: Tuple[float, float]) -> HalfSpace: ...
    @staticmethod
    def from_coeffs(a: float, b: float, c: float = 0.0) -> HalfSpace: ...

class FullSpace(CollisionObject):
    """Geometry occupying the entire plane."""

    def __init__(self) -> None: ...

class Polygon(CollisionObject):
    """Polygon with an exterior ring and optional interior rings."""

    def __init__(
        self,
        exterior: Sequence[Tuple[float, float]],
        interiors: Optional[Sequence[Sequence[Tuple[float, float]]]] = None,
    ) -> None: ...

class Rectangle(CollisionObject):
    """Oriented rectangle specified by length, width, angle, and center."""

    def __init__(
        self, length: float, width: float, orientation: float = 0.0, center: Tuple[float, float] = (0.0, 0.0)
    ) -> None: ...

class Triangle(CollisionObject):
    """Triangle specified by three finite vertices."""

    def __init__(self, a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> None: ...

__all__ = [
    "CollisionObject",
    "Compound",
    "Circle",
    "Empty",
    "HalfSpace",
    "FullSpace",
    "Polygon",
    "Rectangle",
    "Triangle",
]
