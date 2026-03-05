import random
import time

import commonroad.geometry.shape
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_params import ShapeParams
from commonroad.visualization.mp_renderer import MPRenderer
from crcc.collision_checker import CollisionCheckerBuilder
from crcc.collision_object import Circle, Polygon, Rectangle
from crcc.pose import Pose
from matplotlib import pyplot as plt


def main():
    # read scenario
    scenario_path = "scenarios/ZAM_Merge-1_1_T-1.xml"
    # scenario_path = "scenarios/DEU_MerzenichRather-2_870_T-149.xml"
    # scenario_path = "scenarios/ZAM_Yield-1_1_T-1.xml"
    # scenario_path = "scenarios/USA_US101-6_1_T-1.xml"
    # scenario_path = "scenarios/ZAM_Tutorial-1_2_T-1.xml"

    scenario, planning_problems = CommonRoadFileReader(scenario_path).open()

    cc = CollisionCheckerBuilder().with_commonroad_scenario(scenario).build()

    car = Rectangle(4.5, 2.0)
    print("Collides with road boundary", cc.collides_static(car, Pose((55.29, -1.99), 1.326)))
    print("Collides between step 26 and 27", cc.collides_static(car, Pose((37.33, 4.07), -2.207)))

    r = Rectangle(2, 3)
    c = Circle(1)
    print("Should collide", r.collides(c, pos_other=Pose((1.5, 0), 0)))

    poly1 = Polygon(
        exterior=[(0, 0), (4, 0), (4, 4), (0, 4)],
        interiors=[[(1, 1), (2, 1), (2, 2), (1, 2)]],
    )
    poly2 = Polygon(
        exterior=[(1.25, 1.25), (1.75, 1.25), (1.75, 1.75), (1.25, 1.75)],
        interiors=[],
    )
    poly3 = Polygon(
        exterior=[(0.5, 0.5), (3.5, 0.5), (3.5, 3.5), (0.5, 3.5)],
        interiors=[],
    )
    print("Should not collide", poly1.collides(poly2))
    print("Should collide", poly1.collides(poly3))

    demo_parallel()


def demo_parallel():
    scenario, _ = CommonRoadFileReader("scenarios/ZAM_Merge-1_1_T-1.xml").open()

    cc = CollisionCheckerBuilder().with_commonroad_scenario(scenario).build()

    car = Rectangle(4.5, 2.0)
    poses = [Pose((p[0], p[1]), p[2]) for p in np.random.uniform([-7, -15, -np.pi], [87, 10, np.pi], (100000, 3))]
    positioned_cars = [(car, pose) for pose in poses]

    # parallel collision checking
    tic = time.perf_counter()
    result = cc.par_collides_static(positioned_cars)
    toc = time.perf_counter()
    print(f"Parallel: {toc - tic:.4f} seconds, {sum(1 if r.collides else 0 for r in result)} collisions")

    # sequential collision checking
    tic = time.perf_counter()
    result = [cc.collides_static(car, pose) for car, pose in positioned_cars]
    toc = time.perf_counter()
    print(f"Sequential: {toc - tic:.4f} seconds, {sum(1 if r.collides else 0 for r in result)} collisions")

    # visualize some results
    random_examples = random.sample(list(zip(poses, result, strict=True)), min(len(poses), 200))
    rnd = MPRenderer()
    scenario.draw(rnd)
    for pose, collides in random_examples:
        rect = commonroad.geometry.shape.Rectangle(4.5, 2.0, np.array(pose.translation), pose.rotation)
        params = ShapeParams(facecolor="red" if collides.collides else "green", opacity=0.5)
        rect.draw(rnd, params)
    rnd.render()
    plt.show()


if __name__ == "__main__":
    main()
