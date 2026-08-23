import math

from crcc import (
    Circle,
    CollisionEngine,
    Compound,
    Empty,
    FullSpace,
    HalfSpace,
    Polygon,
    Pose,
    Rectangle,
    Triangle,
)

from examples.presentation import ResultRow, collision_result, print_results


def basic_results(engine: CollisionEngine) -> tuple[ResultRow, ...]:
    vehicle = Rectangle(4.0, 2.0)
    circle = Circle(1.0)
    clear_pose = Pose.from_translation((6.0, 0.0))
    yard = Polygon(
        [(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0), (-3.0, -3.0)],
        [[(-0.75, -0.75), (0.75, -0.75), (0.75, 0.75), (-0.75, 0.75), (-0.75, -0.75)]],
    )
    compound = Compound([Circle(0.75, (-1.0, 0.0)), Circle(0.75, (1.0, 0.0))])
    composed = Pose((2.0, 1.0), math.pi / 2).compose(Pose.from_translation((2.0, 0.0)))
    queries = (
        ("overlapping primitives", lambda: vehicle.collides(circle, backend=engine)),
        (
            "boundary contact",
            lambda: vehicle.collides(circle, pos_other=Pose.from_translation((3.0, 0.0)), backend=engine),
        ),
        ("separated primitives", lambda: vehicle.collides(circle, pos_other=clear_pose, backend=engine)),
        ("polygon hole", lambda: yard.collides(Circle(0.25), backend=engine)),
        ("compound child", lambda: compound.collides(Circle(0.4, (1.0, 0.0)), backend=engine)),
        (
            "triangle vs circle",
            lambda: Triangle((-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)).collides(Circle(0.4), backend=engine),
        ),
        ("half space", lambda: HalfSpace((1.0, 0.0)).collides(Circle(0.5, (-0.25, 0.0)), backend=engine)),
        ("full space", lambda: FullSpace().collides(Circle(0.5, (100.0, 100.0)), backend=engine)),
        ("empty space", lambda: Empty().collides(Circle(10.0), backend=engine)),
    )
    return (
        *(collision_result(name, query) for name, query in queries),
        (
            "separation distance",
            "clear",
            f"distance={vehicle.distance(circle, pos_other=clear_pose, backend=engine):.3f}",
        ),
        ("pose composition", "clear", f"translation={tuple(round(v, 3) for v in composed.translation)}"),
    )


def run(engine: CollisionEngine):
    results = basic_results(engine)
    print_results(f"Basic collision checking | engine={engine}", results)
    return results
