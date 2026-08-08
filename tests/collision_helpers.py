from crcc import CollisionEngine, CollisionObject, CollisionStatus, Pose, Rectangle


def collision_status(result: CollisionStatus) -> tuple[bool, int | None]:
    return result.collides, result.time_step


def assert_collides(
    left: CollisionObject,
    right: CollisionObject,
    expected: bool,
    *,
    engine: CollisionEngine,
    pos_left: Pose | None = None,
    pos_right: Pose | None = None,
) -> None:
    pos_left = Pose.identity() if pos_left is None else pos_left
    pos_right = Pose.identity() if pos_right is None else pos_right
    assert left.collides(right, pos_self=pos_left, pos_other=pos_right, engine=engine) is expected
    assert right.collides(left, pos_self=pos_right, pos_other=pos_left, engine=engine) is expected


def axis_aligned_rectangle(radius_x: float, radius_y: float, center_x: float, center_y: float) -> Rectangle:
    return Rectangle(2.0 * radius_x, 2.0 * radius_y, 0.0, (center_x, center_y))


def oriented_rectangle(
    radius_x: float, radius_y: float, orientation: float, center_x: float, center_y: float
) -> Rectangle:
    return Rectangle(2.0 * radius_x, 2.0 * radius_y, orientation, (center_x, center_y))
