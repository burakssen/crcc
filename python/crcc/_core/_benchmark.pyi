from .collision_checker import CollisionChecker, CollisionStatus
from .collision_object import CollisionObject
from .pose import Pose

def collides_static_batch_fresh_pool(
    checker: CollisionChecker,
    positioned_query_shapes: list[tuple[CollisionObject, Pose]],
    threads: int,
    min_time: int | None = None,
    max_time: int | None = None,
) -> list[CollisionStatus]: ...

__all__ = ["collides_static_batch_fresh_pool"]
