from __future__ import annotations

from typing import Any, Iterator

from crcc.collision_checker import CollisionCheckerBuilder
from crcc.collision_object import CollisionObject
from crcc.pose import Pose

ROAD_BOUNDARY_SIMPLIFY_TOLERANCE: float
ROAD_BOUNDARY_MIN_HOLE_AREA: float

def create_collision_checker_from_scenario(
    scenario: Any,
    builder: CollisionCheckerBuilder | None = None,
) -> CollisionCheckerBuilder: ...
def add_commonroad_static_obstacle_to_builder(
    builder: CollisionCheckerBuilder,
    static_obstacle: Any,
) -> CollisionCheckerBuilder: ...
def add_commonroad_dynamic_obstacle_to_builder(
    builder: CollisionCheckerBuilder,
    dynamic_obstacle: Any,
) -> CollisionCheckerBuilder: ...
def add_road_boundary_to_builder(
    builder: CollisionCheckerBuilder,
    lanelet_network: Any,
) -> CollisionCheckerBuilder: ...
def create_road_boundary_obstacle(lanelet_network: Any) -> CollisionObject: ...
def iter_shapely_polygons(geometry: Any) -> Iterator[Any]: ...
def commonroad_polygon_to_collision_object(polygon: Any) -> CollisionObject: ...
def commonroad_shape_to_collision_object(shape: Any) -> CollisionObject: ...
def commonroad_occupancy_to_collision_object(occupancy: Any) -> CollisionObject: ...
def shapely_geometry_to_collision_object(geometry: Any) -> CollisionObject: ...
def commonroad_state_to_pose(state: Any) -> Pose: ...

__all__ = [
    "create_collision_checker_from_scenario",
    "add_commonroad_static_obstacle_to_builder",
    "add_commonroad_dynamic_obstacle_to_builder",
    "add_road_boundary_to_builder",
    "create_road_boundary_obstacle",
    "iter_shapely_polygons",
    "commonroad_polygon_to_collision_object",
    "commonroad_shape_to_collision_object",
    "commonroad_occupancy_to_collision_object",
    "shapely_geometry_to_collision_object",
    "commonroad_state_to_pose",
]
