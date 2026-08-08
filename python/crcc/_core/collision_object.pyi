from __future__ import annotations

from collections.abc import Sequence

from .collision_checker import CollisionEngine
from .pose import Pose

class CollisionObject:
    """Base class for queryable 2D geometry."""

    def collides(
        self,
        other: CollisionObject,
        pos_self: Pose = Pose.identity(),
        pos_other: Pose = Pose.identity(),
        engine: CollisionEngine | None = None,
    ) -> bool: ...
    def collides_continuous(
        self,
        start_pos_self: Pose,
        end_pos_self: Pose,
        other: CollisionObject,
        start_pos_other: Pose,
        end_pos_other: Pose,
        engine: CollisionEngine | None = None,
    ) -> bool: ...
    def distance(
        self,
        other: CollisionObject,
        pos_self: Pose = Pose.identity(),
        pos_other: Pose = Pose.identity(),
        engine: CollisionEngine | None = None,
    ) -> float: ...
    def merge(self, other: CollisionObject) -> CollisionObject: ...
    @staticmethod
    def merge_all(collision_objects: Sequence[CollisionObject]) -> CollisionObject: ...

class Compound(CollisionObject):
    """Union of zero or more collision objects."""

    def __init__(self, collision_objects: Sequence[CollisionObject]) -> None: ...

class Circle(CollisionObject):
    """Circle with a positive radius and optional local-space center."""

    def __init__(self, radius: float, center: tuple[float, float] = (0.0, 0.0)) -> None: ...

class Empty(CollisionObject):
    """Geometry that never collides."""

    def __init__(self) -> None: ...

class HalfSpace(CollisionObject):
    """Region where outward_normal dot point is at most offset."""

    def __init__(self, outward_normal: tuple[float, float], offset: float = 0.0) -> None: ...
    @staticmethod
    def from_points(point_1: tuple[float, float], point_2: tuple[float, float]) -> HalfSpace: ...
    @staticmethod
    def from_coeffs(a: float, b: float, c: float = 0.0) -> HalfSpace: ...

class FullSpace(CollisionObject):
    """Geometry occupying the entire plane."""

    def __init__(self) -> None: ...

class Polygon(CollisionObject):
    """Polygon with an exterior ring and optional interior rings."""

    def __init__(
        self,
        exterior: Sequence[tuple[float, float]],
        interiors: Sequence[Sequence[tuple[float, float]]] | None = None,
    ) -> None: ...

class Rectangle(CollisionObject):
    """Oriented rectangle specified by length, width, angle, and center."""

    def __init__(
        self, length: float, width: float, orientation: float = 0.0, center: tuple[float, float] = (0.0, 0.0)
    ) -> None: ...

class Triangle(CollisionObject):
    """Triangle specified by three finite vertices."""

    def __init__(
        self,
        point_a: tuple[float, float],
        point_b: tuple[float, float],
        point_c: tuple[float, float],
    ) -> None: ...

__all__ = [
    "Circle",
    "CollisionObject",
    "Compound",
    "Empty",
    "FullSpace",
    "HalfSpace",
    "Polygon",
    "Rectangle",
    "Triangle",
]
