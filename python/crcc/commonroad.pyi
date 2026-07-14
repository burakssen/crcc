from __future__ import annotations

from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import DynamicObstacle as CommonRoadDynamicObstacle, StaticObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import TraceState
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry.base import BaseGeometry

from crcc import CollisionCheckerBuilder, CollisionObject, DynamicObstacle, Pose

ROAD_BOUNDARY_SIMPLIFY_TOLERANCE: float
ROAD_BOUNDARY_MIN_HOLE_AREA: float

def create_collision_checker_from_scenario(
    scenario: Scenario,
    builder: CollisionCheckerBuilder | None = None,
) -> CollisionCheckerBuilder: ...
def add_commonroad_static_obstacle_to_builder(
    builder: CollisionCheckerBuilder,
    static_obstacle: StaticObstacle,
) -> CollisionCheckerBuilder: ...
def add_commonroad_dynamic_obstacle_to_builder(
    builder: CollisionCheckerBuilder,
    dynamic_obstacle: CommonRoadDynamicObstacle,
) -> CollisionCheckerBuilder: ...
def commonroad_dynamic_obstacle(dynamic_obstacle: CommonRoadDynamicObstacle) -> DynamicObstacle: ...
def add_road_boundary_to_builder(
    builder: CollisionCheckerBuilder,
    lanelet_network: LaneletNetwork,
) -> CollisionCheckerBuilder: ...
def road_boundary(lanelet_network: LaneletNetwork) -> CollisionObject: ...
def commonroad_polygon(polygon: ShapelyPolygon) -> CollisionObject: ...
def commonroad_shape(shape: ObstacleShape) -> CollisionObject: ...
def commonroad_occupancy(occupancy: Occupancy) -> CollisionObject: ...
def shapely_geometry(geometry: BaseGeometry) -> CollisionObject: ...
def commonroad_state_to_pose(state: TraceState) -> Pose: ...

__all__ = [
    "create_collision_checker_from_scenario",
    "add_commonroad_static_obstacle_to_builder",
    "add_commonroad_dynamic_obstacle_to_builder",
    "commonroad_dynamic_obstacle",
    "add_road_boundary_to_builder",
    "road_boundary",
    "commonroad_polygon",
    "commonroad_shape",
    "commonroad_occupancy",
    "shapely_geometry",
    "commonroad_state_to_pose",
]
