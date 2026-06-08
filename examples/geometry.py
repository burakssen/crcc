from crcc.collision_object import Circle, Polygon, Rectangle
from crcc.pose import Pose


def run():
    """Run simple static geometry collision examples."""
    rectangle = Rectangle(2.0, 3.0)
    circle = Circle(1.0)
    print("Should collide", rectangle.collides(circle, pos_other=Pose((1.5, 0.0), 0.0)))

    outer_polygon = Polygon(
        exterior=[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)],
        interiors=[[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)]],
    )
    inner_polygon = Polygon(
        exterior=[(1.25, 1.25), (1.75, 1.25), (1.75, 1.75), (1.25, 1.75)],
        interiors=[],
    )
    overlapping_polygon = Polygon(
        exterior=[(0.5, 0.5), (3.5, 0.5), (3.5, 3.5), (0.5, 3.5)],
        interiors=[],
    )
    print("Should not collide", outer_polygon.collides(inner_polygon))
    print("Should collide", outer_polygon.collides(overlapping_polygon))


if __name__ == "__main__":
    run()
