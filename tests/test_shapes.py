import math

import pytest
from collision_helpers import assert_collides
from crcc.collision_object import Circle, CollisionObject, Compound, Empty, FullSpace, Polygon, Rectangle, Triangle
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose


@pytest.mark.parametrize(
    ("left", "right", "expected"),
    [
        (Circle(1.0), Circle(1.0, (1.5, 0.0)), True),
        (Circle(1.0), Circle(1.0, (3.0, 0.0)), False),
        (Rectangle(2.0, 2.0), Circle(0.5, (2.75, 0.0)), False),
        (Rectangle(2.0, 1.0), Rectangle(2.0, 1.0, math.pi / 4.0, (0.9, 0.0)), True),
        (Rectangle(2.0, 1.0), Rectangle(2.0, 1.0, 0.0, (3.5, 0.0)), False),
        (
            Polygon([(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0)]),
            Circle(0.5),
            True,
        ),
        (
            Polygon([(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0)]),
            Circle(0.5, (3.0, 0.0)),
            False,
        ),
        (Triangle((-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)), Rectangle(0.5, 0.5), True),
        (
            Triangle((-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)),
            Rectangle(0.5, 0.5, 0.0, (3.0, 0.0)),
            False,
        ),
        (Compound([Circle(0.5, (-5.0, 0.0)), Rectangle(1.0, 1.0)]), Circle(0.5, (0.25, 0.0)), True),
        (
            Compound([Circle(0.5, (-5.0, 0.0)), Rectangle(1.0, 1.0, 0.0, (5.0, 0.0))]),
            Circle(0.5),
            False,
        ),
        (Empty(), Circle(1.0), False),
        (Empty(), FullSpace(), False),
        (FullSpace(), Circle(1.0), True),
        (FullSpace(), Rectangle(1.0, 1.0, 0.0, (100.0, 100.0)), True),
    ],
)
def test_shape_collisions(left, right, expected, engine):
    assert_collides(left, right, expected, engine=engine)


def test_rectangle_and_circle_collide_with_offset_pose(engine):
    assert_collides(
        Rectangle(2.0, 2.0),
        Circle(0.75),
        True,
        engine=engine,
        pos_right=Pose.from_translation((1.0, 0.0)),
    )


def test_mixed_shape_sequences_build_compound_merge_and_time_variant_obstacles():
    shapes = [Circle(0.5), Rectangle(1.0, 1.0)]

    compound = Compound(shapes)
    merged = CollisionObject.merge_all(shapes)
    trajectory = DynamicObstacle.from_time_variant(shapes)

    assert compound.collides(Circle(0.1))
    assert merged.collides(Circle(0.1))
    assert trajectory is not None


def test_pose_composition_and_multiplication():
    pose1 = Pose((1.0, 2.0), math.pi / 2.0)
    pose2 = Pose((3.0, 4.0), 0.0)

    # Using compose() method
    composed = pose1.compose(pose2)
    # Using * operator
    multiplied = pose1 * pose2

    assert composed.translation == multiplied.translation
    assert composed.rotation == multiplied.rotation

    # Verify values: translation of pose2 (3, 4) rotated by pi/2 is (-4, 3), added to (1, 2) is (-3, 5)
    assert math.isclose(composed.translation[0], -3.0, abs_tol=1e-7)
    assert math.isclose(composed.translation[1], 5.0, abs_tol=1e-7)
    assert math.isclose(composed.rotation, math.pi / 2.0, abs_tol=1e-7)


def test_collides_continuous(engine):
    circle = Circle(1.0)
    obstacle = Circle(1.0)

    # Moving circle from (-3, 0) to (3, 0), should sweep through obstacle at (0, 0)
    start_pos_self = Pose((-3.0, 0.0), 0.0)
    end_pos_self = Pose((3.0, 0.0), 0.0)
    pos_other = Pose((0.0, 0.0), 0.0)

    assert circle.collides_continuous(
        start_pos_self,
        end_pos_self,
        obstacle,
        pos_other,
        pos_other,
        engine=engine,
    )
