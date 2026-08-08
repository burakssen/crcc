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

def scenario_builder(
    scenario: Scenario,
    builder: CollisionCheckerBuilder | None = None,
) -> CollisionCheckerBuilder: ...
def add_static_obstacle(
    builder: CollisionCheckerBuilder,
    static_obstacle: StaticObstacle,
) -> CollisionCheckerBuilder: ...
def add_dynamic_obstacle(
    builder: CollisionCheckerBuilder,
    dynamic_obstacle: CommonRoadDynamicObstacle,
) -> CollisionCheckerBuilder: ...
def from_dynamic_obstacle(dynamic_obstacle: CommonRoadDynamicObstacle) -> DynamicObstacle: ...
def add_road_boundary(
    builder: CollisionCheckerBuilder,
    lanelet_network: LaneletNetwork,
) -> CollisionCheckerBuilder: ...
def road_boundary(lanelet_network: LaneletNetwork) -> CollisionObject: ...
def from_polygon(polygon: ShapelyPolygon) -> CollisionObject: ...
def from_shape(shape: ObstacleShape) -> CollisionObject: ...
def from_occupancy(occupancy: Occupancy) -> CollisionObject: ...
def from_shapely(geometry: BaseGeometry) -> CollisionObject: ...
def from_pose(state: TraceState) -> Pose: ...

__all__ = [
    "add_dynamic_obstacle",
    "add_road_boundary",
    "add_static_obstacle",
    "from_dynamic_obstacle",
    "from_occupancy",
    "from_polygon",
    "from_pose",
    "from_shape",
    "from_shapely",
    "road_boundary",
    "scenario_builder",
]
