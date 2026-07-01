from dataclasses import dataclass, field

import numpy as np
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from crcc.collision_checker import CollisionCheckerBuilder, CollisionEngine
from crcc.dynamic_obstacle import DynamicObstacle
from crcc.pose import Pose
from matplotlib import pyplot as plt
from matplotlib.widgets import Button, RadioButtons, Slider, TextBox

from examples.drawing import VisualShape, collision_object, demo_shapes, draw_visual_shape
from examples.utils import scenario_time_steps

COLOR_COLLIDED = "#dc2626"
COLOR_CLEAR = "#059669"
COLOR_SELECTED = "#2563eb"


def interpolate_path(path: list[Pose], num_steps: int) -> list[Pose]:
    if not path:
        return [Pose.identity()] * num_steps
    if len(path) == 1:
        return [path[0]] * num_steps
    
    interpolated = []
    n = len(path)
    for t in range(num_steps):
        val = t * (n - 1) / (num_steps - 1)
        idx = int(np.floor(val))
        if idx >= n - 1:
            interpolated.append(path[-1])
        else:
            w = val - idx
            p1 = path[idx].translation
            p2 = path[idx + 1].translation
            x = (1 - w) * p1[0] + w * p2[0]
            y = (1 - w) * p1[1] + w * p2[1]
            interpolated.append(Pose.from_translation((x, y)))
    return interpolated


def pose_at_float_index(poses: list[Pose], float_idx: float) -> Pose:
    if not poses:
        return Pose.identity()
    n = len(poses)
    idx1 = int(np.floor(float_idx))
    idx2 = int(np.ceil(float_idx))
    idx1 = min(max(0, idx1), n - 1)
    idx2 = min(max(0, idx2), n - 1)
    if idx1 == idx2:
        return poses[idx1]
    w = float_idx - idx1
    p1 = poses[idx1].translation
    p2 = poses[idx2].translation
    x = (1 - w) * p1[0] + w * p2[0]
    y = (1 - w) * p1[1] + w * p2[1]
    a1 = poses[idx1].rotation
    a2 = poses[idx2].rotation
    diff = (a2 - a1 + np.pi) % (2 * np.pi) - np.pi
    angle = a1 + w * diff
    return Pose((x, y), angle)


def make_variants(shape: VisualShape) -> list[VisualShape]:
    if shape.kind == "circle":
        r = shape.params[0]
        return [
            VisualShape("circle", (r,), "circle"),
            VisualShape("circle", (r * 1.3,), "circle_large"),
            VisualShape("circle", (r * 0.7,), "circle_small"),
        ]
    elif shape.kind == "rectangle":
        l, w, o = shape.params
        return [
            VisualShape("rectangle", (l, w, o), "rectangle"),
            VisualShape("rectangle", (l * 1.2, w * 0.8, o + 0.2), "rectangle_morph_a"),
            VisualShape("rectangle", (l * 0.8, w * 1.2, o - 0.2), "rectangle_morph_b"),
        ]
    else:
        return [
            shape,
            VisualShape("circle", (0.8,), "circle_var"),
            VisualShape("rectangle", (1.6, 0.9, 0.25), "rectangle_var"),
        ]


@dataclass
class SceneObject:
    name: str
    mode: str
    shape: VisualShape
    pose: Pose = field(default_factory=Pose.identity)
    path: list[Pose] = field(default_factory=list)
    variants: list[VisualShape] = field(default_factory=list)

    def object_at(self, time_index: float, num_steps: int = 80):
        if self.mode in {"time_variant", "time_variant_dynamic"} and self.variants:
            shape = self.variants[int(np.round(time_index)) % len(self.variants)]
        else:
            shape = self.shape

        if self.mode in {"dynamic", "time_variant_dynamic"} and self.path:
            interpolated = interpolate_path(self.path, num_steps)
            pose = pose_at_float_index(interpolated, time_index)
        else:
            pose = self.pose
        return shape, pose

    def dynamic_obstacle(self, time_offset: int, num_steps: int = 80):
        path = self.path or [self.pose]
        interpolated = interpolate_path(path, num_steps)

        if self.mode == "dynamic":
            return DynamicObstacle(collision_object(self.shape), interpolated, time_offset)
        if self.mode == "time_variant_dynamic":
            variants = self.variants or [self.shape]
            shapes = [collision_object(variants[i % len(variants)]) for i in range(num_steps)]
            return DynamicObstacle.from_time_variant(shapes, time_offset, interpolated)
        if self.mode == "time_variant":
            variants = self.variants or [self.shape]
            shapes = [collision_object(variants[i % len(variants)]) for i in range(num_steps)]
            poses = [self.pose] * num_steps
            return DynamicObstacle.from_time_variant(shapes, time_offset, poses)
        return None


@dataclass
class PlaygroundState:
    engine: CollisionEngine
    time_steps: tuple[int, ...]
    objects: list[SceneObject] = field(default_factory=list)
    selected: int | None = None
    draft_polygon: list[tuple[float, float]] = field(default_factory=list)
    draft_path: list[Pose] = field(default_factory=list)
    shape_kind: str = "circle"
    mode: str = "static"
    time_index: int = 0
    simulating: bool = False
    last_results: dict[int, object] = field(default_factory=dict)
    tool: str = "select"
    dragging_object: int | None = None
    dragging_path_point: tuple[int, int] | None = None
    drag_offset: tuple[float, float] | None = None
    scenario: object | None = None
    scenario_road_boundary: object | None = None
    scenario_static_obstacles: list[object] = field(default_factory=list)
    scenario_dynamic_obstacles: list[object] = field(default_factory=list)
    current_preset: str = "Random Mix"
    sim_speed: float = 1.0

    def set_scenario(self, scenario):
        self.scenario = scenario
        if scenario is not None:
            from crcc.commonroad import (
                road_boundary,
                commonroad_occupancy,
                commonroad_shape,
                commonroad_state_to_pose,
            )
            from commonroad.prediction.prediction import TrajectoryPrediction

            self.scenario_road_boundary = road_boundary(scenario.lanelet_network)
            self.scenario_static_obstacles = [
                commonroad_occupancy(static_obstacle.occupancy_at_time(static_obstacle.initial_state.time_step))
                for static_obstacle in scenario.static_obstacles
            ]
            self.scenario_dynamic_obstacles = []
            for dyn_obstacle in scenario.dynamic_obstacles:
                initial_time = dyn_obstacle.initial_state.time_step
                if isinstance(dyn_obstacle.prediction, TrajectoryPrediction):
                    trajectory = dyn_obstacle.prediction.trajectory
                    states = [dyn_obstacle.initial_state] + trajectory.state_list
                    poses = [commonroad_state_to_pose(state) for state in states]
                    shape = commonroad_shape(dyn_obstacle.obstacle_shape)
                    self.scenario_dynamic_obstacles.append(DynamicObstacle(shape, poses, initial_time))

    def initialize_random_objects(self, plot_limits):
        self.objects.clear()
        shapes_list = [
            VisualShape("circle", (0.9,), "circle"),
            VisualShape("rectangle", (1.8, 1.0, 0.2), "rectangle"),
            VisualShape("triangle", (((-0.8, -0.5), (0.8, -0.4), (0.0, 0.8))), "triangle"),
            VisualShape("polygon", (((-0.9, -0.6), (0.6, -0.7), (0.9, 0.2), (0.0, 0.9), (-0.8, 0.3), (-0.9, -0.6))), "polygon"),
            VisualShape("compound", (), "compound"),
        ]
        modes = ["static", "dynamic", "time_variant", "time_variant_dynamic", "static"]
        
        x_range = plot_limits[1] - plot_limits[0]
        y_range = plot_limits[3] - plot_limits[2]
        xmin, xmax = plot_limits[0] + 0.25 * x_range, plot_limits[1] - 0.25 * x_range
        ymin, ymax = plot_limits[2] + 0.25 * y_range, plot_limits[3] - 0.25 * y_range

        for i, (shape, mode) in enumerate(zip(shapes_list, modes)):
            x = np.random.uniform(xmin, xmax)
            y = np.random.uniform(ymin, ymax)
            pose = Pose.from_translation((x, y))

            path = []
            if mode in {"dynamic", "time_variant_dynamic"}:
                dest_x = x + np.random.uniform(-x_range * 0.15, x_range * 0.15)
                dest_y = y + np.random.uniform(-y_range * 0.15, y_range * 0.15)
                path = [pose, Pose.from_translation((dest_x, dest_y))]
            else:
                path = [pose]

            variants = make_variants(shape)
            obj = SceneObject(
                name=f"{mode}_{shape.kind}_{i+1}",
                mode=mode,
                shape=shape,
                pose=pose,
                path=path,
                variants=variants
            )
            self.objects.append(obj)

        self.selected = 0 if self.objects else None
        self.last_results = self.object_statuses()

    def load_preset(self, preset_name, plot_limits):
        self.current_preset = preset_name
        self.objects.clear()
        self.selected = None
        self.time_index = 0.0
        self.simulating = False
        self._last_collision_idx = None
        
        x_mid = (plot_limits[0] + plot_limits[1]) / 2.0
        y_mid = (plot_limits[2] + plot_limits[3]) / 2.0
        x_range = plot_limits[1] - plot_limits[0]
        y_range = plot_limits[3] - plot_limits[2]

        if preset_name == "Random Mix":
            self.initialize_random_objects(plot_limits)
        elif preset_name == "Intersection":
            c_shape = VisualShape("compound", (), "compound")
            obj1 = SceneObject(
                name="static_center",
                mode="static",
                shape=c_shape,
                pose=Pose.from_translation((x_mid, y_mid)),
                path=[Pose.from_translation((x_mid, y_mid))],
                variants=make_variants(c_shape)
            )
            y_start = y_mid + y_range * 0.25
            y_end = y_mid - y_range * 0.25
            cir_shape = VisualShape("circle", (0.8,), "circle")
            obj2 = SceneObject(
                name="vehicle_north",
                mode="dynamic",
                shape=cir_shape,
                pose=Pose.from_translation((x_mid - x_range * 0.05, y_start)),
                path=[Pose.from_translation((x_mid - x_range * 0.05, y_start)), Pose.from_translation((x_mid - x_range * 0.05, y_end))],
                variants=make_variants(cir_shape)
            )
            x_start = x_mid - x_range * 0.25
            x_end = x_mid + x_range * 0.25
            rect_shape = VisualShape("rectangle", (1.8, 1.0, 0.0), "rectangle")
            obj3 = SceneObject(
                name="vehicle_west",
                mode="dynamic",
                shape=rect_shape,
                pose=Pose.from_translation((x_start, y_mid + y_range * 0.05)),
                path=[Pose.from_translation((x_start, y_mid + y_range * 0.05)), Pose.from_translation((x_end, y_mid + y_range * 0.05))],
                variants=make_variants(rect_shape)
            )
            self.objects.extend([obj1, obj2, obj3])
            self.selected = 0
        elif preset_name == "Overtaking":
            x_slow_start = x_mid - x_range * 0.15
            x_slow_end = x_mid + x_range * 0.2
            rect_shape1 = VisualShape("rectangle", (1.8, 0.9, 0.0), "rectangle")
            obj1 = SceneObject(
                name="slow_lead_car",
                mode="dynamic",
                shape=rect_shape1,
                pose=Pose.from_translation((x_slow_start, y_mid)),
                path=[Pose.from_translation((x_slow_start, y_mid)), Pose.from_translation((x_slow_end, y_mid))],
                variants=make_variants(rect_shape1)
            )
            x_fast_start = x_mid - x_range * 0.3
            x_fast_w1 = x_mid - x_range * 0.1
            x_fast_w2 = x_mid + x_range * 0.1
            x_fast_end = x_mid + x_range * 0.25
            dy = y_range * 0.08
            
            rect_shape2 = VisualShape("rectangle", (1.8, 0.9, 0.0), "rectangle")
            obj2 = SceneObject(
                name="fast_overtaker",
                mode="dynamic",
                shape=rect_shape2,
                pose=Pose.from_translation((x_fast_start, y_mid)),
                path=[
                    Pose.from_translation((x_fast_start, y_mid)),
                    Pose.from_translation((x_fast_w1, y_mid + dy)),
                    Pose.from_translation((x_fast_w2, y_mid + dy)),
                    Pose.from_translation((x_fast_end, y_mid))
                ],
                variants=make_variants(rect_shape2)
            )
            self.objects.extend([obj1, obj2])
            self.selected = 0
        elif preset_name == "Clear Map":
            pass

        self.last_results = self.object_statuses()

    def add_object(self, center=(0.0, 0.0)):
        shape = shape_from_kind(self.shape_kind, self.draft_polygon if self.shape_kind == "freehand" else None)
        pose = Pose.from_translation(center)
        variants = make_variants(shape)
        path = self.draft_path[:] if self.draft_path else [pose]
        obj = SceneObject(f"{self.mode}_{len(self.objects) + 1}", self.mode, shape, pose, path, variants)
        self.objects.append(obj)
        self.selected = len(self.objects) - 1
        self.draft_polygon.clear()
        self.draft_path.clear()
        return obj

    def add_freehand_vertex(self, point):
        self.draft_polygon.append(point)

    def finalize_freehand(self):
        if len(self.draft_polygon) < 3:
            return None
        self.shape_kind = "freehand"
        return self.add_object(self.draft_polygon[0])

    def add_path_point(self, point):
        self.draft_path.append(Pose.from_translation(point))

    def delete_selected(self):
        if self.selected is None:
            return
        del self.objects[self.selected]
        self.selected = min(self.selected, len(self.objects) - 1) if self.objects else None

    def select_next(self):
        if not self.objects:
            self.selected = None
            return None
        self.selected = 0 if self.selected is None else (self.selected + 1) % len(self.objects)
        return self.objects[self.selected]

    def clear_draft(self):
        self.draft_polygon.clear()
        self.draft_path.clear()

    def status_summary(self):
        if self.selected is None or not self.objects:
            return "no selected object"
        obj = self.objects[self.selected]
        status = self.last_results.get(self.selected)
        if status is None:
            status = self.selected_status()
        colliding = sum(
            1 for result in self.last_results.values() if result is not None and result.collides
        )
        state = "running" if self.simulating else "paused"
        return f"{state} | {obj.name} | {obj.mode} | {obj.shape.label} | {status} | colliding={colliding}"

    def toggle_simulation(self):
        self.simulating = not self.simulating
        return self.simulating

    def step_simulation(self, dt: float = 1.0):
        if self.time_steps:
            self.time_index = (self.time_index + dt) % len(self.time_steps)
        self.last_results = self.object_statuses()
        return self.time_index

    def object_statuses(self):
        statuses = {}
        num_steps = len(self.time_steps)
        discrete_idx = int(np.round(self.time_index))
        discrete_idx = min(max(0, discrete_idx), len(self.time_steps) - 1)
        time_step = self.time_steps[discrete_idx]
        for index, obj in enumerate(self.objects):
            checker = self.checker_excluding(index)
            if obj.mode == "static":
                statuses[index] = checker.collides_static(
                    collision_object(obj.shape), obj.pose, min_time=time_step, max_time=time_step
                )
            else:
                statuses[index] = checker.collides_dynamic(
                    obj.dynamic_obstacle(self.time_steps[0], num_steps), min_time=time_step, max_time=time_step
                )
        return statuses

    def checker(self):
        return self.checker_excluding(self.selected)

    def checker_excluding(self, excluded_index):
        builder = CollisionCheckerBuilder(engine=self.engine)
        if self.scenario_road_boundary is not None:
            builder.with_static_obstacle(self.scenario_road_boundary)
        for static_obs in self.scenario_static_obstacles:
            builder.with_static_obstacle(static_obs)
        for dynamic_obs in self.scenario_dynamic_obstacles:
            builder.with_dynamic_obstacle(dynamic_obs)
        
        time_offset = self.time_steps[0]
        num_steps = len(self.time_steps)
        for index, obj in enumerate(self.objects):
            if index == excluded_index:
                continue
            if obj.mode == "static":
                builder.with_static_obstacle(collision_object(obj.shape, obj.pose.translation))
            else:
                builder.with_dynamic_obstacle(obj.dynamic_obstacle(time_offset, num_steps))
        return builder.build()

    def selected_status(self):
        if self.selected is None or not self.objects:
            return None
        obj = self.objects[self.selected]
        checker = self.checker_excluding(self.selected)
        discrete_idx = int(np.round(self.time_index))
        discrete_idx = min(max(0, discrete_idx), len(self.time_steps) - 1)
        time_step = self.time_steps[discrete_idx]
        num_steps = len(self.time_steps)
        if obj.mode == "static":
            return checker.collides_static(collision_object(obj.shape), obj.pose, min_time=time_step, max_time=time_step)
        return checker.collides_dynamic(obj.dynamic_obstacle(self.time_steps[0], num_steps), min_time=time_step, max_time=time_step)


def shape_from_kind(kind: str, points=None):
    if kind == "freehand":
        closed = tuple(points) + (points[0],)
        return VisualShape("polygon", closed, "freehand")
    return next(shape for shape in demo_shapes() if shape.kind == kind)


def draw_playground(ax, scenario, state: PlaygroundState, plot_limits):
    ax.clear()
    discrete_idx = int(np.round(state.time_index))
    discrete_idx = min(max(0, discrete_idx), len(state.time_steps) - 1)
    time_step = state.time_steps[discrete_idx]
    draw_params = MPDrawParams(time_begin=time_step, time_end=time_step)
    renderer = MPRenderer(draw_params=draw_params, plot_limits=list(plot_limits), ax=ax)
    scenario.draw(renderer, draw_params)
    renderer.render()

    artists = []
    num_steps = len(state.time_steps)
    state.last_results = state.object_statuses()
    for index, obj in enumerate(state.objects):
        shape, pose = obj.object_at(state.time_index, num_steps)
        state.selected = state.selected if state.selected is not None else index
        color = COLOR_SELECTED if index == state.selected else COLOR_CLEAR
        status = state.last_results.get(index)
        if status is not None and status.collides:
            color = COLOR_COLLIDED
        elif index == state.selected:
            color = COLOR_SELECTED
        artists.extend(draw_visual_shape(ax, shape, pose.translation, color, 0.68, linewidth=2.4, zorder=50))
        if obj.path:
            pts = np.array([p.translation for p in obj.path])
            artists.append(ax.plot(pts[:, 0], pts[:, 1], color=color, linestyle="--", linewidth=1.4, zorder=45)[0])
            pt_color = "#8b5cf6" if index == state.selected else color
            for waypoint_pose in obj.path:
                artists.extend(
                    draw_visual_shape(
                        ax,
                        obj.shape,
                        waypoint_pose.translation,
                        pt_color,
                        alpha=0.32,
                        linewidth=1.2,
                        zorder=46,
                    )
                )
    if state.draft_polygon:
        pts = np.array(state.draft_polygon)
        artists.append(ax.plot(pts[:, 0], pts[:, 1], color=COLOR_SELECTED, marker="o", linewidth=1.4, zorder=60)[0])
    if state.draft_path:
        pts = np.array([p.translation for p in state.draft_path])
        artists.append(ax.plot(pts[:, 0], pts[:, 1], color="#7c3aed", linestyle="--", linewidth=1.4, zorder=60)[0])
        shape_to_draw = shape_from_kind(state.shape_kind, state.draft_polygon if state.shape_kind == "freehand" else None)
        for waypoint_pose in state.draft_path:
            artists.extend(
                draw_visual_shape(
                    ax,
                    shape_to_draw,
                    waypoint_pose.translation,
                    "#7c3aed",
                    alpha=0.3,
                    linewidth=1.0,
                    zorder=60,
                )
            )

    ax.set_title("Interactive Collision Playground")
    return artists


def run(scenario, checker, scenario_path, pose_bounds):
    """Open a Matplotlib editor for adding and simulating collision objects."""
    time_steps = tuple(scenario_time_steps(scenario)[:80])
    plot_limits = (pose_bounds[0][0], pose_bounds[1][0], pose_bounds[0][1], pose_bounds[1][1])
    state = PlaygroundState(checker.engine, time_steps)
    state.set_scenario(scenario)
    center = ((plot_limits[0] + plot_limits[1]) / 2.0, (plot_limits[2] + plot_limits[3]) / 2.0)
    
    state.initialize_random_objects(plot_limits)

    import glob
    from pathlib import Path
    scenario_files = sorted(glob.glob("scenarios/*.xml"))
    scenario_names = [Path(f).name for f in scenario_files]
    current_name = Path(scenario_path).name
    try:
        active_idx = scenario_names.index(current_name)
    except ValueError:
        active_idx = 0

    fig, ax = plt.subplots(figsize=(12.8, 8.0))
    plt.subplots_adjust(left=0.22, right=0.82, bottom=0.15, top=0.94)

    # Position GUI axes
    ax_preset = plt.axes((0.02, 0.77, 0.17, 0.16))
    ax_tool = plt.axes((0.02, 0.58, 0.17, 0.14))
    ax_shape = plt.axes((0.02, 0.33, 0.17, 0.20))
    ax_mode = plt.axes((0.02, 0.14, 0.17, 0.14))
    ax_name = plt.axes((0.02, 0.04, 0.17, 0.05))
    
    ax_scenario_sel = plt.axes((0.83, 0.15, 0.16, 0.78))
    
    ax_simulate = plt.axes((0.22, 0.04, 0.08, 0.05))
    ax_slower = plt.axes((0.31, 0.04, 0.03, 0.05))
    ax_faster = plt.axes((0.35, 0.04, 0.03, 0.05))
    ax_step = plt.axes((0.39, 0.04, 0.06, 0.05))
    ax_reset = plt.axes((0.46, 0.04, 0.06, 0.05))
    ax_delete = plt.axes((0.53, 0.04, 0.08, 0.05))
    ax_clear = plt.axes((0.62, 0.04, 0.09, 0.05))
    ax_poly = plt.axes((0.72, 0.04, 0.10, 0.05))

    # Initialize widgets
    preset_buttons = RadioButtons(ax_preset, ["Random Mix", "Intersection", "Overtaking", "Clear Map"])
    tool_buttons = RadioButtons(ax_tool, ["Select/Move", "Add Object", "Add Path"])
    shape_buttons = RadioButtons(ax_shape, ["circle", "rectangle", "triangle", "polygon", "compound", "freehand"])
    mode_buttons = RadioButtons(ax_mode, ["static", "dynamic", "time_variant", "time_variant_dynamic"])
    name_box = TextBox(ax_name, "name", initial="object")
    
    scenario_buttons = RadioButtons(ax_scenario_sel, scenario_names, active=active_idx)
    
    simulate_button = Button(ax_simulate, "simulate")
    slower_button = Button(ax_slower, "<<")
    faster_button = Button(ax_faster, ">>")
    step_button = Button(ax_step, "step")
    reset_button = Button(ax_reset, "reset")
    delete_button = Button(ax_delete, "delete")
    clear_button = Button(ax_clear, "clear draft")
    polygon_button = Button(ax_poly, "finalize")

    widgets = (
        preset_buttons,
        tool_buttons,
        shape_buttons,
        mode_buttons,
        name_box,
        scenario_buttons,
        simulate_button,
        slower_button,
        faster_button,
        step_button,
        reset_button,
        delete_button,
        clear_button,
        polygon_button,
    )

    tool_mapping = {
        "Select/Move": "select",
        "Add Object": "add_object",
        "Add Path": "add_path",
    }

    def redraw(force_collision_update=False):
        discrete_idx = int(np.round(state.time_index))
        discrete_idx = min(max(0, discrete_idx), len(time_steps) - 1)
        if force_collision_update or getattr(state, "_last_collision_idx", None) != discrete_idx:
            state.last_results = state.object_statuses()
            state._last_collision_idx = discrete_idx
        draw_playground(ax, scenario, state, plot_limits)
        ax.set_title(f"Playground | Frame: {discrete_idx}/{len(time_steps)-1} | Speed: {state.sim_speed}x | {state.status_summary()}")
        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=100)

    def on_timer():
        if state.simulating:
            state.step_simulation(state.sim_speed)
            redraw()
        return True

    timer.add_callback(on_timer)

    def on_press(event):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        point = (event.xdata, event.ydata)
        threshold = 0.02 * (plot_limits[1] - plot_limits[0])

        if state.tool == "select":
            if state.selected is not None:
                obj = state.objects[state.selected]
                if obj.path:
                    for pt_idx, pose in enumerate(obj.path):
                        dist = np.hypot(point[0] - pose.translation[0], point[1] - pose.translation[1])
                        if dist < threshold:
                            state.dragging_path_point = (state.selected, pt_idx)
                            return
            for idx, obj in enumerate(state.objects):
                _, pose = obj.object_at(state.time_index, len(time_steps))
                dist = np.hypot(point[0] - pose.translation[0], point[1] - pose.translation[1])
                if dist < threshold:
                    state.selected = idx
                    state.dragging_object = idx
                    state.drag_offset = (pose.translation[0] - point[0], pose.translation[1] - point[1])
                    redraw(True)
                    return
        elif state.tool == "add_object":
            if state.shape_kind == "freehand":
                state.add_freehand_vertex(point)
            else:
                state.add_object(point)
                if name_box.text:
                    state.objects[state.selected].name = name_box.text
            redraw(True)
        elif state.tool == "add_path":
            if state.selected is not None:
                obj = state.objects[state.selected]
                if not obj.path:
                    obj.path = [obj.pose]
                obj.path.append(Pose.from_translation(point))
                if obj.mode == "static":
                    obj.mode = "dynamic"
                    mode_buttons.set_active(1)
                elif obj.mode == "time_variant":
                    obj.mode = "time_variant_dynamic"
                    mode_buttons.set_active(3)
            else:
                state.add_path_point(point)
            redraw(True)

    def on_motion(event):
        if event.xdata is None or event.ydata is None:
            return
        point = (event.xdata, event.ydata)

        if state.dragging_path_point is not None:
            obj_idx, pt_idx = state.dragging_path_point
            state.objects[obj_idx].path[pt_idx] = Pose.from_translation(point)
            redraw(True)
        elif state.dragging_object is not None:
            obj_idx = state.dragging_object
            dx, dy = state.drag_offset
            new_translation = (point[0] + dx, point[1] + dy)
            obj = state.objects[obj_idx]
            _, old_pose = obj.object_at(state.time_index, len(time_steps))
            old_translation = old_pose.translation
            shift_x = new_translation[0] - old_translation[0]
            shift_y = new_translation[1] - old_translation[1]

            if obj.mode in {"static", "time_variant"} and not obj.path:
                obj.pose = Pose.from_translation(new_translation)
            else:
                for i in range(len(obj.path)):
                    orig = obj.path[i].translation
                    obj.path[i] = Pose.from_translation((orig[0] + shift_x, orig[1] + shift_y))
                orig_pose = obj.pose.translation
                obj.pose = Pose.from_translation((orig_pose[0] + shift_x, orig_pose[1] + shift_y))
            redraw(True)

    def on_release(event):
        state.dragging_object = None
        state.dragging_path_point = None

    def on_key(event):
        if fig.canvas.widgetlock.locked():
            return

        if event.key == " ":
            state.toggle_simulation()
            if state.simulating:
                timer.start()
            else:
                timer.stop()
            redraw()
        elif event.key == "right":
            state.step_simulation(1.0)
            redraw()
        elif event.key == "left":
            state.time_index = max(0.0, state.time_index - 1.0)
            redraw()
        elif event.key == "r":
            on_reset(None)
        elif event.key in {",", "<"}:
            on_slower(None)
        elif event.key in {".", ">"}:
            on_faster(None)
        elif event.key in {"delete", "backspace", "d"}:
            state.delete_selected()
            redraw(True)
        elif event.key in {"tab", "n"}:
            state.select_next()
            redraw(True)
        elif event.key == "escape":
            state.clear_draft()
            redraw()
        elif event.key == "1":
            tool_buttons.set_active(0)
        elif event.key == "2":
            tool_buttons.set_active(1)
        elif event.key == "3":
            tool_buttons.set_active(2)

    def on_reset(_event):
        state.simulating = False
        timer.stop()
        state.time_index = 0.0
        redraw(True)

    def on_slower(_event):
        state.sim_speed = max(0.125, state.sim_speed / 2.0)
        redraw()

    def on_faster(_event):
        state.sim_speed = min(8.0, state.sim_speed * 2.0)
        redraw()

    preset_buttons.on_clicked(lambda value: (state.load_preset(value, plot_limits), redraw(True)))
    tool_buttons.on_clicked(lambda value: (setattr(state, "tool", tool_mapping[value]), redraw()))
    shape_buttons.on_clicked(lambda value: (setattr(state, "shape_kind", value), redraw()))
    mode_buttons.on_clicked(lambda value: (setattr(state, "mode", value), redraw()))
    polygon_button.on_clicked(lambda _event: (state.finalize_freehand(), redraw(True)))
    delete_button.on_clicked(lambda _event: (state.delete_selected(), redraw(True)))
    clear_button.on_clicked(lambda _event: (state.clear_draft(), redraw()))
    simulate_button.on_clicked(lambda _event: (state.toggle_simulation(), timer.start() if state.simulating else timer.stop(), redraw()))
    reset_button.on_clicked(on_reset)
    slower_button.on_clicked(on_slower)
    faster_button.on_clicked(on_faster)
    
    def on_scenario_select(value):
        nonlocal plot_limits, center, scenario, checker, scenario_path, time_steps
        from examples.utils import load_collision_checker, scenario_pose_bounds, scenario_time_steps
        
        new_path = f"scenarios/{value}"
        new_scenario, new_checker = load_collision_checker(new_path, state.engine)
        
        scenario = new_scenario
        checker = new_checker
        scenario_path = new_path
        time_steps = tuple(scenario_time_steps(scenario)[:80])
        
        state.time_steps = time_steps
        state.set_scenario(scenario)
        
        new_bounds = scenario_pose_bounds(scenario)
        plot_limits = (new_bounds[0][0], new_bounds[1][0], new_bounds[0][1], new_bounds[1][1])
        center = ((plot_limits[0] + plot_limits[1]) / 2.0, (plot_limits[2] + plot_limits[3]) / 2.0)
        
        ax.set_xlim(plot_limits[0], plot_limits[1])
        ax.set_ylim(plot_limits[2], plot_limits[3])
        
        state.load_preset(state.current_preset, plot_limits)
        redraw(True)

    scenario_buttons.on_clicked(on_scenario_select)

    def on_step(_event):
        state.step_simulation(1.0)
        redraw()

    step_button.on_clicked(on_step)

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("key_press_event", on_key)

    setattr(fig, "_crcc_widgets", widgets)
    setattr(fig, "_crcc_timer", timer)
    redraw()
    plt.show()
    return state
