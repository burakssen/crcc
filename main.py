import commonroad_collision_checker._core as ccc
from commonroad.common.file_reader import CommonRoadFileReader


def main():
    # read scenario
    # scenario_path = "../../commonroad/commonroad-reach-flow/scenarios/ZAM_Merge-1_1_T-1.xml"
    scenario_path = "/home/lercher/datasets/exiD/exiD-commonroad-only6-no-merge-selected/scenarios/DEU_MerzenichRather-2_870_T-149.xml"
    # scenario_path = "scenarios/ZAM_Yield-1_1_T-1.xml"
    # scenario_path = "scenarios/USA_US101-6_1_T-1.xml"
    # scenario_path = "scenarios/ZAM_Tutorial-1_2_T-1.xml"

    scenario, planning_problems = CommonRoadFileReader(scenario_path).open()

    lanelet_network = scenario.lanelet_network

    lanelet_polygons = [list(lanelet.polygon.shapely_object.exterior.coords) for lanelet in lanelet_network.lanelets]

    rb = ccc.road_boundary.RoadBoundaryChecker(lanelet_polygons)
    print(rb.collides((55.29, -1.99), 1.326))
    print(rb.collides((37.33, 4.07), -2.207))

    r = ccc.collision_object.Rectangle(2, 3)
    c = ccc.collision_object.Circle(1)
    print(r.collides(c, pos_self=ccc.isometry.Isometry((0, 0), 0), pos_other=ccc.isometry.Isometry((1.5, 0), 0)))

    poly1 = ccc.collision_object.Polygon(
        exterior=[(0, 0), (4, 0), (4, 4), (0, 4)],
        interiors=[[(1, 1), (2, 1), (2, 2), (1, 2)]],
    )
    poly2 = ccc.collision_object.Polygon(
        exterior=[(1.25, 1.25), (1.75, 1.25), (1.75, 1.75), (1.25, 1.75)],
        interiors=[],
    )
    poly3 = ccc.collision_object.Polygon(
        exterior=[(0.5, 0.5), (3.5, 0.5), (3.5, 3.5), (0.5, 3.5)],
        interiors=[],
    )
    print(poly1.collides(poly2))
    print(poly1.collides(poly3))


if __name__ == "__main__":
    main()
