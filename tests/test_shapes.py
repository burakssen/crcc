import math
import unittest

from crcc.collision_checker import CollisionEngine
from crcc.collision_object import Circle, Compound, Empty, FullSpace, Polygon, Rectangle, Triangle
from crcc.pose import Pose


class ShapeCollisionTests(unittest.TestCase):
    ENGINES = [CollisionEngine.Parry, CollisionEngine.Rhusics]

    def assert_collides(self, left, right, expected, pos_left=Pose.identity(), pos_right=Pose.identity()):
        for engine in self.ENGINES:
            with self.subTest(engine=engine, left=type(left).__name__, right=type(right).__name__, expected=expected):
                self.assertEqual(
                    left.collides(right, pos_self=pos_left, pos_other=pos_right, engine=engine),
                    expected,
                )
                self.assertEqual(
                    right.collides(left, pos_self=pos_right, pos_other=pos_left, engine=engine),
                    expected,
                )

    def test_circles_collide_when_overlapping(self):
        self.assert_collides(Circle(1.0), Circle(1.0, (1.5, 0.0)), True)

    def test_circles_do_not_collide_when_separated(self):
        self.assert_collides(Circle(1.0), Circle(1.0, (3.0, 0.0)), False)

    def test_rectangle_and_circle_collide_with_offset_pose(self):
        rectangle = Rectangle(2.0, 2.0)
        circle = Circle(0.75)

        self.assert_collides(rectangle, circle, True, pos_right=Pose.from_translation((1.0, 0.0)))

    def test_rectangle_and_circle_do_not_collide_when_separated(self):
        self.assert_collides(Rectangle(2.0, 2.0), Circle(0.5, (2.75, 0.0)), False)

    def test_rotated_rectangles_collide(self):
        rectangle = Rectangle(2.0, 1.0)
        rotated = Rectangle(2.0, 1.0, math.pi / 4.0, (0.9, 0.0))

        self.assert_collides(rectangle, rotated, True)

    def test_rectangles_do_not_collide_when_separated(self):
        self.assert_collides(Rectangle(2.0, 1.0), Rectangle(2.0, 1.0, 0.0, (3.5, 0.0)), False)

    def test_polygon_collides_with_circle_inside(self):
        square = Polygon([(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0)])

        self.assert_collides(square, Circle(0.5), True)

    def test_polygon_does_not_collide_with_separated_circle(self):
        square = Polygon([(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0)])

        self.assert_collides(square, Circle(0.5, (3.0, 0.0)), False)

    def test_triangle_collides_with_rectangle(self):
        triangle = Triangle((-1.0, -1.0), (1.0, -1.0), (0.0, 1.0))

        self.assert_collides(triangle, Rectangle(0.5, 0.5), True)

    def test_triangle_does_not_collide_with_separated_rectangle(self):
        triangle = Triangle((-1.0, -1.0), (1.0, -1.0), (0.0, 1.0))

        self.assert_collides(triangle, Rectangle(0.5, 0.5, 0.0, (3.0, 0.0)), False)

    def test_compound_collides_when_any_component_collides(self):
        compound = Compound([Circle(0.5, (-5.0, 0.0)), Rectangle(1.0, 1.0)])

        self.assert_collides(compound, Circle(0.5, (0.25, 0.0)), True)

    def test_compound_does_not_collide_when_all_components_are_separated(self):
        compound = Compound([Circle(0.5, (-5.0, 0.0)), Rectangle(1.0, 1.0, 0.0, (5.0, 0.0))])

        self.assert_collides(compound, Circle(0.5), False)

    def test_empty_never_collides(self):
        self.assert_collides(Empty(), Circle(1.0), False)
        self.assert_collides(Empty(), FullSpace(), False)

    def test_full_space_collides_with_finite_shapes(self):
        self.assert_collides(FullSpace(), Circle(1.0), True)
        self.assert_collides(FullSpace(), Rectangle(1.0, 1.0, 0.0, (100.0, 100.0)), True)


if __name__ == "__main__":
    unittest.main()
