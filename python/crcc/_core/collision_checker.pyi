from collections.abc import Sequence
from typing import overload

from .collision_object import CollisionObject
from .dynamic_obstacle import DynamicObstacle
from .pose import Pose

class CollisionBackend:
    """Runtime collision backend selector."""

    Parry: CollisionBackend
    Rhusics: CollisionBackend
    Collide: CollisionBackend

class CollisionStatus:
    """Checker result identifying no, static, or first dynamic collision.

    Read collides instead of relying on generated enum truthiness. Dynamic
    status time is a sample/interval start, not exact time of impact.
    """

    @staticmethod
    def NoCollision() -> CollisionStatus: ...
    @staticmethod
    def CollidesStatic() -> CollisionStatus: ...
    @staticmethod
    def CollidesDynamic(time_step: int, /) -> CollisionStatus: ...
    @property
    def collides(self) -> bool: ...
    @property
    def time_step(self) -> int | None: ...
    def __str__(self) -> str: ...
    def __repr__(self) -> str: ...

class PreparedStaticQuery:
    @property
    def backend(self) -> CollisionBackend: ...
    @property
    def engine(self) -> CollisionBackend:  # Deprecated: use backend.
        ...

class PreparedDynamicQuery:
    @property
    def backend(self) -> CollisionBackend: ...
    @property
    def engine(self) -> CollisionBackend:  # Deprecated: use backend.
        ...

class CollisionChecker:
    """Immutable static and dynamic collision scene."""

    @property
    def backend(self) -> CollisionBackend: ...
    @property
    def engine(self) -> CollisionBackend:  # Deprecated: use backend.
        ...
    def prepare_static(self, query_shape: CollisionObject) -> PreparedStaticQuery: ...
    def prepare_dynamic(self, dynamic_obstacle: DynamicObstacle) -> PreparedDynamicQuery: ...
    @overload
    def collides_static(
        self,
        query: CollisionObject,
        position: Pose | None = None,
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> CollisionStatus: ...
    @overload
    def collides_static(
        self,
        query: PreparedStaticQuery,
        position: Pose | None = None,
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> CollisionStatus: ...
    def collides_static_batch(
        self,
        queries: Sequence[tuple[CollisionObject | PreparedStaticQuery, Pose]],
        min_time: int | None = None,
        max_time: int | None = None,
        parallel: bool = False,
    ) -> list[CollisionStatus]: ...
    @overload
    def collides_dynamic(
        self,
        query: DynamicObstacle,
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> CollisionStatus: ...
    @overload
    def collides_dynamic(
        self,
        query: PreparedDynamicQuery,
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> CollisionStatus: ...
    def collides_dynamic_batch(
        self,
        queries: Sequence[DynamicObstacle | PreparedDynamicQuery],
        min_time: int | None = None,
        max_time: int | None = None,
        parallel: bool = False,
    ) -> list[CollisionStatus]: ...

    # Deprecated compatibility aliases; use canonical methods above.
    def collides_static_prepared(
        self,
        query: PreparedStaticQuery,
        position: Pose | None = None,
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> CollisionStatus: ...
    def collides_static_prepared_batch(
        self,
        query: PreparedStaticQuery,
        positions: Sequence[Pose],
        min_time: int | None = None,
        max_time: int | None = None,
        parallel: bool = False,
    ) -> list[CollisionStatus]: ...
    def par_static(
        self,
        positioned_query_shapes: Sequence[tuple[CollisionObject, Pose]],
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> list[CollisionStatus]: ...
    def collides_dynamic_prepared(
        self,
        query: PreparedDynamicQuery,
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> CollisionStatus: ...
    def collides_dynamic_prepared_batch(
        self,
        queries: Sequence[PreparedDynamicQuery],
        min_time: int | None = None,
        max_time: int | None = None,
        parallel: bool = False,
    ) -> list[CollisionStatus]: ...
    def par_dynamic(
        self,
        dynamic_obstacles: Sequence[DynamicObstacle],
        min_time: int | None = None,
        max_time: int | None = None,
    ) -> list[CollisionStatus]: ...

class CollisionCheckerBuilder:
    """Native builder; the public Python facade uses backend= and add_* instead."""

    def __init__(
        self,
        engine: CollisionBackend | None = None,
    ) -> None: ...
    def build(self, engine: CollisionBackend | None = None) -> CollisionChecker: ...
    def with_engine(self, engine: CollisionBackend) -> CollisionCheckerBuilder: ...
    def with_static_obstacle(self, collision_object: CollisionObject) -> CollisionCheckerBuilder: ...
    def with_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder: ...
    def with_road_boundary(
        self,
        lanelets: Sequence[Sequence[tuple[float, float]]],
    ) -> CollisionCheckerBuilder: ...

def road_boundary(lanelets: Sequence[Sequence[tuple[float, float]]]) -> CollisionObject: ...
