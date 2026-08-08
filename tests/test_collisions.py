import math

import pytest
from collision_helpers import assert_collides, axis_aligned_rectangle, oriented_rectangle
from crcc import Circle, Compound, Empty, FullSpace, Polygon, Pose, Rectangle, Triangle


@pytest.mark.parametrize(
    ("left", "right", "expected"),
    [
        # Basic Shape Collisions
        (Circle(1.0), Circle(1.0, (1.5, 0.0)), True),
        (Circle(1.0), Circle(1.0, (3.0, 0.0)), False),
        (Rectangle(2.0, 2.0), Circle(0.5, (2.75, 0.0)), False),
        (Rectangle(2.0, 1.0), Rectangle(2.0, 1.0, math.pi / 4.0, (0.9, 0.0)), True),
        (Rectangle(2.0, 1.0), Rectangle(2.0, 1.0, 0.0, (3.5, 0.0)), False),
        (Polygon([(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0)]), Circle(0.5), True),
        (Polygon([(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0)]), Circle(0.5, (3.0, 0.0)), False),
        (Triangle((-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)), Rectangle(0.5, 0.5), True),
        (Triangle((-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)), Rectangle(0.5, 0.5, 0.0, (3.0, 0.0)), False),
        (Compound([Circle(0.5, (-5.0, 0.0)), Rectangle(1.0, 1.0)]), Circle(0.5, (0.25, 0.0)), True),
        (Compound([Circle(0.5, (-5.0, 0.0)), Rectangle(1.0, 1.0, 0.0, (5.0, 0.0))]), Circle(0.5), False),
        (Empty(), Circle(1.0), False),
        (Empty(), FullSpace(), False),
        (FullSpace(), Circle(1.0), True),
        (FullSpace(), Rectangle(1.0, 1.0, 0.0, (100.0, 100.0)), True),
        # Reference/Benchmark Cases (Rectangle & Circle)
        (axis_aligned_rectangle(2.0, 3.0, 3.0, 1.8), Circle(2.5, (6.0, 7.0)), True),
        (axis_aligned_rectangle(2.0, 3.0, 3.0, 1.7), Circle(2.5, (6.0, 7.0)), False),
        (oriented_rectangle(1.0, 2.0, 0.2, 9.0, 10.0), Circle(2.5, (6.0, 7.0)), False),
        (oriented_rectangle(1.0, 2.0, 0.1, 9.0, 10.0), Circle(2.5, (6.0, 7.0)), True),
        (oriented_rectangle(1.0, 2.0, 0.0, 9.0, 14.0), axis_aligned_rectangle(1.0, 1.0, 11.1, 16.0), False),
        (oriented_rectangle(1.0, 2.0, 0.2, 9.0, 14.0), axis_aligned_rectangle(1.0, 1.0, 11.1, 16.0), False),
        (oriented_rectangle(1.0, 2.0, -0.2, 9.0, 14.0), axis_aligned_rectangle(1.0, 1.0, 11.1, 16.0), True),
        # Reference Triangle Cases
        (Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)), Circle(2.5, (6.0, 7.0)), False),
        (Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)), Circle(2.5, (5.0, 7.0)), True),
        (Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)), oriented_rectangle(1.0, 2.0, 0.6, 9.0, 3.0), False),
        (Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)), oriented_rectangle(1.0, 2.0, 0.4, 9.0, 3.0), True),
        (Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)), oriented_rectangle(1.0, 1.3, 0.6, 10.8, 0.8), False),
        (Triangle((10.0, 2.0), (2.0, 2.0), (5.0, 5.0)), oriented_rectangle(1.0, 1.3, 0.6, 10.8, 0.8), False),
        (Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)), oriented_rectangle(1.0, 1.3, 0.6, 10.6, 0.6), True),
        (Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)), oriented_rectangle(1.0, 1.3, 0.6, 10.8, 0.6), True),
        (Triangle((10.0, 2.0), (2.0, 2.0), (5.0, 5.0)), oriented_rectangle(1.0, 1.3, 0.6, 10.8, 0.6), False),
        (Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)), oriented_rectangle(1.0, 1.3, 0.6, 10.6, 1.0), False),
        (Triangle((10.0, 2.0), (2.0, 2.0), (5.0, 5.0)), oriented_rectangle(1.0, 1.3, 0.6, 10.6, 1.0), True),
    ],
    ids=[
        "circle-overlap",
        "circle-separated",
        "rectangle-circle-separated",
        "rotated-rectangles-overlap",
        "rectangles-separated",
        "polygon-circle-overlap",
        "polygon-circle-separated",
        "triangle-rectangle-overlap",
        "triangle-rectangle-separated",
        "compound-child-overlap",
        "compound-separated",
        "empty-circle",
        "empty-full-space",
        "full-space-circle",
        "full-space-rectangle",
        "reference-rectangle-circle-hit",
        "reference-rectangle-circle-clear",
        "reference-oriented-rectangle-circle-clear",
        "reference-oriented-rectangle-circle-hit",
        "reference-rectangles-clear-axis-aligned",
        "reference-rectangles-clear-rotated",
        "reference-rectangles-hit-rotated",
        "reference-triangle-circle-clear",
        "reference-triangle-circle-hit",
        "reference-triangle-rectangle-clear",
        "reference-triangle-rectangle-hit",
        "reference-triangle-rectangle-clear-far",
        "reference-second-triangle-rectangle-clear-far",
        "reference-triangle-rectangle-hit-near",
        "reference-triangle-rectangle-hit-offset",
        "reference-second-triangle-rectangle-clear-offset",
        "reference-triangle-rectangle-clear-high",
        "reference-second-triangle-rectangle-hit-high",
    ],
)
def test_static_shape_collisions(left, right, expected, engine):
    """Verify exact collision checker queries for various shape primitives."""
    assert_collides(left, right, expected, engine=engine)


@pytest.mark.parametrize(
    ("query", "expected"),
    [
        (oriented_rectangle(1.0, 1.3, 0.6, 10.8, 0.8), False),
        (oriented_rectangle(1.0, 1.3, 0.6, 10.6, 0.6), True),
        (oriented_rectangle(1.0, 1.3, 0.6, 10.8, 0.6), True),
        (oriented_rectangle(1.0, 1.3, 0.6, 10.6, 1.0), True),
    ],
    ids=["clear", "first-triangle-hit", "first-triangle-offset-hit", "second-triangle-hit"],
)
def test_polygon_represented_as_compound_triangles(query, expected, engine):
    """Test multi-triangle compound shapes against oriented rectangles."""
    polygon = Compound(
        [
            Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)),
            Triangle((10.0, 2.0), (2.0, 2.0), (5.0, 5.0)),
        ]
    )
    assert_collides(polygon, query, expected, engine=engine)


def test_rectangle_and_circle_with_offset_pose(engine):
    """Ensure shape collisions respect non-identity relative poses."""
    assert_collides(
        Rectangle(2.0, 2.0),
        Circle(0.75),
        True,
        engine=engine,
        pos_right=Pose.from_translation((1.0, 0.0)),
    )


def test_polygon_with_hole_collisions(engine):
    """Verify collision queries on complex polygon definitions containing interior holes."""
    polygon = Polygon(
        [(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0), (-3.0, -3.0)],
        [[(-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5), (0.5, -0.5), (-0.5, -0.5)]],
    )
    assert_collides(polygon, Circle(0.1), False, engine=engine)
    assert_collides(polygon, Circle(0.1, (2.0, 0.0)), True, engine=engine)


def test_pose_composition_and_multiplication():
    """Verify 2D transformation composition methods and operators."""
    pose1 = Pose((1.0, 2.0), math.pi / 2.0)
    pose2 = Pose((3.0, 4.0), 0.0)

    composed = pose1.compose(pose2)
    multiplied = pose1 * pose2

    assert composed.translation == multiplied.translation
    assert composed.rotation == multiplied.rotation

    assert math.isclose(composed.translation[0], -3.0, abs_tol=1e-7)
    assert math.isclose(composed.translation[1], 5.0, abs_tol=1e-7)
    assert math.isclose(composed.rotation, math.pi / 2.0, abs_tol=1e-7)


@pytest.mark.parametrize(
    ("self_motion", "other_motion", "expected"),
    [
        (((-5.0, 0.0), (-4.0, 0.0)), ((5.0, 0.0), (5.0, 0.0)), False),
        (((0.0, 0.0), (3.0, 0.0)), ((0.0, 0.0), (0.0, 0.0)), True),
        (((-3.0, 0.0), (3.0, 0.0)), ((3.0, 0.0), (-3.0, 0.0)), True),
    ],
    ids=["certified-clear", "endpoint-overlap", "moving-moving-crossing"],
)
def test_collides_continuous(self_motion, other_motion, expected, engine):
    """Cover CCD's certified-clear, endpoint, and two-moving-body contracts."""
    start_self, end_self = map(Pose.from_translation, self_motion)
    start_other, end_other = map(Pose.from_translation, other_motion)

    assert (
        Circle(1.0).collides_continuous(
            start_self,
            end_self,
            Circle(1.0),
            start_other,
            end_other,
            engine=engine,
        )
        is expected
    )
