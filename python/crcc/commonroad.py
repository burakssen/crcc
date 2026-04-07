from __future__ import annotations

import commonroad.scenario.obstacle as cr_obstacle
from commonroad.geometry.obstacle_shapes.circle_obstacle_shape import CircleObstacleShape
from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.obstacle_shapes.polygon_obstacle_shape import PolygonObstacleShape
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState, TraceState
from shapely.geometry import MultiPolygon, Polygon as ShapelyPolygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

from crcc.collision_checker import CollisionCheckerBuilder
from crcc.collision_object import Circle, CollisionObject, Compound, HalfSpace, Polygon, Rectangle
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose

ROAD_BOUNDARY_SIMPLIFY_TOLERANCE = 0.01
ROAD_BOUNDARY_MIN_HOLE_AREA = 0.001


def create_collision_checker_from_scenario(
    scenario: Scenario,
    builder: CollisionCheckerBuilder | None = None,
) -> CollisionCheckerBuilder:
    """Creates a collision checker builder from a CommonRoad scenario."""
    if builder is None:
        builder = CollisionCheckerBuilder()

    builder = add_road_boundary_to_builder(builder, scenario.lanelet_network)
    for static_obstacle in scenario.static_obstacles:
        builder = add_commonroad_static_obstacle_to_builder(builder, static_obstacle)
    for dynamic_obstacle in scenario.dynamic_obstacles:
        builder = add_commonroad_dynamic_obstacle_to_builder(builder, dynamic_obstacle)
    return builder


def add_commonroad_static_obstacle_to_builder(
    builder: CollisionCheckerBuilder,
    static_obstacle: cr_obstacle.StaticObstacle,
) -> CollisionCheckerBuilder:
    """Adds a CommonRoad static obstacle to the builder."""
    co = commonroad_occupancy_to_collision_object(
        static_obstacle.occupancy_at_time(static_obstacle.initial_state.time_step)
    )
    builder.with_static_obstacle(co)
    return builder


def add_commonroad_dynamic_obstacle_to_builder(
    builder: CollisionCheckerBuilder,
    dynamic_obstacle: cr_obstacle.DynamicObstacle,
) -> CollisionCheckerBuilder:
    """Adds a CommonRoad dynamic obstacle to the builder."""
    initial_time = dynamic_obstacle.initial_state.time_step
    if isinstance(dynamic_obstacle.prediction, TrajectoryPrediction):
        trajectory = dynamic_obstacle.prediction.trajectory
        states = [dynamic_obstacle.initial_state] + trajectory.state_list
        poses = [commonroad_state_to_pose(state) for state in states]
        shape = commonroad_shape_to_collision_object(dynamic_obstacle.obstacle_shape)
        rust_dynamic_obstacle = DynamicObstacle(shape, poses, initial_time)
        builder.with_dynamic_obstacle(rust_dynamic_obstacle)
    else:
        raise NotImplementedError("Only TrajectoryPrediction is supported for dynamic obstacles.")
    return builder


def add_road_boundary_to_builder(
    builder: CollisionCheckerBuilder,
    lanelet_network: LaneletNetwork,
) -> CollisionCheckerBuilder:
    """Adds the road boundary from a lanelet network to the builder."""
    builder.with_static_obstacle(create_road_boundary_obstacle(lanelet_network))
    return builder


def create_road_boundary_obstacle(lanelet_network: LaneletNetwork) -> CollisionObject:
    """Creates an obstacle for all space outside the lanelet network."""
    road = unary_union([lanelet.polygon.shapely_object for lanelet in lanelet_network.lanelets]).simplify(
        ROAD_BOUNDARY_SIMPLIFY_TOLERANCE
    )
    if road.is_empty:
        return Compound([])

    road_convex_hull = orient(road.convex_hull, sign=1.0)
    obstacles = [
        HalfSpace.from_points(tuple(p1), tuple(p2))
        for p1, p2 in zip(road_convex_hull.exterior.coords, road_convex_hull.exterior.coords[1:])
    ]

    holes = road_convex_hull.difference(road)
    obstacles.extend(
        commonroad_polygon_to_collision_object(orient(hole, sign=1.0))
        for hole in iter_shapely_polygons(holes)
        if hole.area > ROAD_BOUNDARY_MIN_HOLE_AREA
    )
    return Compound(obstacles)


def iter_shapely_polygons(geometry):
    if geometry.is_empty:
        return
    if isinstance(geometry, ShapelyPolygon):
        yield geometry
        return
    for geom in geometry.geoms:
        yield from iter_shapely_polygons(geom)


def commonroad_polygon_to_collision_object(polygon: ShapelyPolygon) -> CollisionObject:
    return Polygon(
        exterior=[tuple(v) for v in polygon.exterior.coords],
        interiors=[[tuple(v) for v in interior.coords] for interior in polygon.interiors],
    )


def commonroad_shape_to_collision_object(shape: ObstacleShape) -> CollisionObject:
    """Converts a CommonRoad obstacle shape to a local crcc CollisionObject."""
    if isinstance(shape, CircleObstacleShape):
        return Circle(shape.radius)
    if isinstance(shape, RectObstacleShape):
        return Rectangle(shape.length, shape.width, center=(-shape.origin_x_shift, 0.0))
    if isinstance(shape, PolygonObstacleShape):
        return Polygon([tuple(v) for v in shape.vertices], [])

    return commonroad_occupancy_to_collision_object(
        shape.compute_occupancy_for_state(InitialState(position=(0.0, 0.0), orientation=0.0))
    )


def commonroad_occupancy_to_collision_object(occupancy: Occupancy) -> CollisionObject:
    """Converts a CommonRoad occupancy to a world-positioned crcc CollisionObject."""
    if isinstance(occupancy, CircleOccupancy):
        return Circle(occupancy.radius, (occupancy.circle_center.x, occupancy.circle_center.y))
    if isinstance(occupancy, RectOccupancy):
        return Rectangle(
            occupancy.length,
            occupancy.width,
            occupancy.orientation,
            (occupancy.rect_center.x, occupancy.rect_center.y),
        )
    if isinstance(occupancy, PolygonOccupancy):
        return commonroad_polygon_to_collision_object(occupancy.shapely_object)

    return shapely_geometry_to_collision_object(occupancy.shapely_object)


def shapely_geometry_to_collision_object(geometry) -> CollisionObject:
    if geometry.is_empty:
        return Compound([])
    if isinstance(geometry, ShapelyPolygon):
        return commonroad_polygon_to_collision_object(geometry)
    if isinstance(geometry, MultiPolygon):
        return Compound([commonroad_polygon_to_collision_object(polygon) for polygon in geometry.geoms])
    raise ValueError(f"Unknown occupancy geometry type {type(geometry)}")


def commonroad_state_to_pose(state: TraceState) -> Pose:
    """Converts a CommonRoad state to a crcc Pose."""
    return Pose(
        translation=(state.position[0], state.position[1]),
        angle=state.orientation,
    )
