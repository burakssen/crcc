import commonroad_collision_checker as ccc
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

    rb = ccc.RoadBoundaryChecker(lanelet_polygons)
    assert rb.collides((55.29, -1.99), 1.326)
    assert not rb.collides((37.33, 4.07), -2.207)


if __name__ == "__main__":
    main()
