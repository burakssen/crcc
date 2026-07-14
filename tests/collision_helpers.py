from crcc import Pose, Rectangle


def assert_collides(left, right, expected, *, engine, pos_left=None, pos_right=None):
    pos_left = Pose.identity() if pos_left is None else pos_left
    pos_right = Pose.identity() if pos_right is None else pos_right
    assert left.collides(right, pos_self=pos_left, pos_other=pos_right, engine=engine) is expected
    assert right.collides(left, pos_self=pos_right, pos_other=pos_left, engine=engine) is expected


def axis_aligned_rectangle(radius_x, radius_y, center_x, center_y):
    return Rectangle(2.0 * radius_x, 2.0 * radius_y, 0.0, (center_x, center_y))


def oriented_rectangle(radius_x, radius_y, orientation, center_x, center_y):
    return Rectangle(2.0 * radius_x, 2.0 * radius_y, orientation, (center_x, center_y))
