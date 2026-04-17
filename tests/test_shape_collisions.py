import pytest
from collision_helpers import assert_collides, axis_aligned_rectangle, oriented_rectangle
from crcc.collision_checker import CollisionCheckerBuilder
from crcc.collision_object import Circle, Compound, Polygon, Rectangle, Triangle
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose


@pytest.mark.parametrize(
    ("left", "right", "expected"),
    [
        (axis_aligned_rectangle(2.0, 3.0, 3.0, 1.8), Circle(2.5, (6.0, 7.0)), True),
        (axis_aligned_rectangle(2.0, 3.0, 3.0, 1.7), Circle(2.5, (6.0, 7.0)), False),
        (oriented_rectangle(1.0, 2.0, 0.2, 9.0, 10.0), Circle(2.5, (6.0, 7.0)), False),
        (oriented_rectangle(1.0, 2.0, 0.1, 9.0, 10.0), Circle(2.5, (6.0, 7.0)), True),
        (
            oriented_rectangle(1.0, 2.0, 0.0, 9.0, 14.0),
            axis_aligned_rectangle(1.0, 1.0, 11.1, 16.0),
            False,
        ),
        (
            oriented_rectangle(1.0, 2.0, 0.2, 9.0, 14.0),
            axis_aligned_rectangle(1.0, 1.0, 11.1, 16.0),
            False,
        ),
        (
            oriented_rectangle(1.0, 2.0, -0.2, 9.0, 14.0),
            axis_aligned_rectangle(1.0, 1.0, 11.1, 16.0),
            True,
        ),
    ],
)
def test_reference_rectangle_and_circle_cases(left, right, expected, engine):
    assert_collides(left, right, expected, engine=engine)


@pytest.mark.parametrize(
    ("left", "right", "expected"),
    [
        (Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)), Circle(2.5, (6.0, 7.0)), False),
        (Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)), Circle(2.5, (5.0, 7.0)), True),
        (
            Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)),
            oriented_rectangle(1.0, 2.0, 0.6, 9.0, 3.0),
            False,
        ),
        (
            Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)),
            oriented_rectangle(1.0, 2.0, 0.4, 9.0, 3.0),
            True,
        ),
        (
            Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)),
            oriented_rectangle(1.0, 1.3, 0.6, 10.8, 0.8),
            False,
        ),
        (
            Triangle((10.0, 2.0), (2.0, 2.0), (5.0, 5.0)),
            oriented_rectangle(1.0, 1.3, 0.6, 10.8, 0.8),
            False,
        ),
        (
            Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)),
            oriented_rectangle(1.0, 1.3, 0.6, 10.6, 0.6),
            True,
        ),
        (
            Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)),
            oriented_rectangle(1.0, 1.3, 0.6, 10.8, 0.6),
            True,
        ),
        (
            Triangle((10.0, 2.0), (2.0, 2.0), (5.0, 5.0)),
            oriented_rectangle(1.0, 1.3, 0.6, 10.8, 0.6),
            False,
        ),
        (
            Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)),
            oriented_rectangle(1.0, 1.3, 0.6, 10.6, 1.0),
            False,
        ),
        (
            Triangle((10.0, 2.0), (2.0, 2.0), (5.0, 5.0)),
            oriented_rectangle(1.0, 1.3, 0.6, 10.6, 1.0),
            True,
        ),
    ],
)
def test_reference_triangle_cases(left, right, expected, engine):
    assert_collides(left, right, expected, engine=engine)


@pytest.mark.parametrize(
    ("query", "expected"),
    [
        (oriented_rectangle(1.0, 1.3, 0.6, 10.8, 0.8), False),
        (oriented_rectangle(1.0, 1.3, 0.6, 10.6, 0.6), True),
        (oriented_rectangle(1.0, 1.3, 0.6, 10.8, 0.6), True),
        (oriented_rectangle(1.0, 1.3, 0.6, 10.6, 1.0), True),
    ],
)
def test_reference_polygon_cases_as_compound_triangles(query, expected, engine):
    polygon = Compound(
        [
            Triangle((0.0, 0.0), (10.0, 0.0), (4.0, 5.0)),
            Triangle((10.0, 2.0), (2.0, 2.0), (5.0, 5.0)),
        ]
    )
    assert_collides(polygon, query, expected, engine=engine)


def test_reference_checker_static_dynamic_and_compound_cases(engine):
    dynamic = DynamicObstacle(
        Rectangle(4.0, 2.0),
        [
            Pose((6.0, 0.0), 1.5),
            Pose((6.0, 2.0), 1.5),
            Pose((6.0, 3.0), 1.5),
            Pose((6.0, 4.0), 1.5),
        ],
        4,
    )
    near = oriented_rectangle(2.0, 1.0, 1.5, 6.0, 0.0)
    far = oriented_rectangle(2.0, 1.0, 1.5, 6.0, 20.0)
    checker = CollisionCheckerBuilder(engine=engine).with_dynamic_obstacle(dynamic).build()

    assert checker.collides_static(near).collides
    assert not checker.collides_static(far).collides
    assert checker.collides_static(Compound([near, far])).collides


def test_reference_polygon_with_hole_cases(engine):
    polygon = Polygon(
        [(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0), (-3.0, -3.0)],
        [[(-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5), (0.5, -0.5), (-0.5, -0.5)]],
    )

    assert_collides(polygon, Circle(0.1), False, engine=engine)
    assert_collides(polygon, Circle(0.1, (2.0, 0.0)), True, engine=engine)
