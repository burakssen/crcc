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

CollisionEngine = CollisionBackend

class CollisionStatus:
    """Checker result identifying no, static, or first dynamic collision.

    Instances are returned by checker queries; they are not constructed
    directly.
    """

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

    # Deprecated aliases retained for one release.
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
    """Builder for an immutable CollisionChecker."""

    def __init__(
        self,
        backend: CollisionBackend | None = None,
        *,
        engine: CollisionBackend | None = None,
    ) -> None: ...
    def add_static_obstacle(self, query_shape: CollisionObject) -> CollisionCheckerBuilder: ...
    def add_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder: ...
    def build(self, backend: CollisionBackend | None = None) -> CollisionChecker: ...

    # Deprecated aliases retained for one release.
    def with_engine(self, engine: CollisionBackend) -> CollisionCheckerBuilder: ...
    def with_static_obstacle(self, query_shape: CollisionObject) -> CollisionCheckerBuilder: ...
    def with_dynamic_obstacle(self, dynamic_obstacle: DynamicObstacle) -> CollisionCheckerBuilder: ...
    def with_road_boundary(
        self,
        lanelets: Sequence[Sequence[tuple[float, float]]],
    ) -> CollisionCheckerBuilder: ...

def road_boundary(lanelets: Sequence[Sequence[tuple[float, float]]]) -> CollisionObject: ...

__all__ = [
    "CollisionBackend",
    "CollisionChecker",
    "CollisionCheckerBuilder",
    "CollisionEngine",
    "CollisionStatus",
    "PreparedDynamicQuery",
    "PreparedStaticQuery",
    "road_boundary",
]
