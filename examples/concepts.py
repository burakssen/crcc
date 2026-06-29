import math

from crcc.collision_checker import CollisionEngine
from crcc.collision_object import Circle, Compound, Polygon, Rectangle, Triangle
from crcc.pose import Pose


def concept_results(engine: CollisionEngine):
    ego = Rectangle(4.5, 2.0)
    obstacle = Circle(1.0)
    pose_near = Pose.from_translation((2.0, 0.0))
    pose_far = Pose.from_translation((8.0, 0.0))

    outer = Polygon(
        [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)],
        [[(2.0, 2.0), (3.0, 2.0), (3.0, 3.0), (2.0, 3.0), (2.0, 2.0)]],
    )
    inner = Circle(0.25, (2.5, 2.5))
    compound = Compound(
        [
            Triangle((0.0, 0.0), (2.0, 0.0), (1.0, 1.5)),
            Triangle((2.0, 0.0), (3.0, 1.5), (1.0, 1.5)),
        ]
    )

    return {
        "near_static_collision": ego.collides(obstacle, pos_other=pose_near, engine=engine),
        "far_static_collision": ego.collides(obstacle, pos_other=pose_far, engine=engine),
        "distance_far": ego.distance(obstacle, pos_other=pose_far, engine=engine),
        "polygon_hole_collision": outer.collides(inner, engine=engine),
        "compound_collision": compound.collides(Circle(0.5, (1.5, 0.8)), engine=engine),
        "continuous_tunnel_collision": Circle(0.5).collides_continuous(
            Pose.from_translation((-4.0, 0.0)),
            Pose.from_translation((4.0, 0.0)),
            Rectangle(0.25, 3.0),
            Pose.identity(),
            Pose.identity(),
            engine,
        ),
        "composed_pose": Pose((1.0, 2.0), math.pi / 2.0).compose(Pose.from_translation((2.0, 0.0))).translation,
    }


def run(engine: CollisionEngine):
    """Run deterministic concept examples for core collision APIs."""
    results = concept_results(engine)
    print("Concept walkthrough")
    for key, value in results.items():
        print(f"  {key}: {value}")
    return results
