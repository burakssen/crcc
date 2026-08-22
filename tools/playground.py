import math
from dataclasses import dataclass, field
from enum import Enum
from itertools import pairwise
from typing import Any, TypedDict

import numpy as np
from commonroad.scenario.scenario import Scenario
from crcc import (
    Circle,
    CollisionCheckerBuilder,
    CollisionEngine,
    Compound,
    DynamicObstacle,
    Polygon,
    Pose,
    Rectangle,
    Triangle,
)
from crcc.commonroad import scenario_builder

from examples.continuous import Sweep, evaluate_sweep

COLOR_EXACT_HIT = "#dc2626"
COLOR_POTENTIAL = "#d97706"
COLOR_CLEAR = "#059669"
COLOR_UNSUPPORTED = "#7c3aed"
COLOR_SELECTED = "#2563eb"
COLOR_PATH = "#7c3aed"


class DragState(TypedDict):
    id: int | None
    last: tuple[float, float] | None


class Verdict(Enum):
    EXACT_HIT = "exact hit"
    EXACT_CLEAR = "exact clear"
    POTENTIAL_COLLISION = "potential collision"
    CERTIFIED_CLEAR = "certified clear"
    UNSUPPORTED = "unsupported"


@dataclass(frozen=True)
class ShapeDefinition:
    kind: str
    values: tuple[Any, ...] = ()

    def collision_object(self):
        if self.kind == "circle":
            return Circle(self.values[0])
        if self.kind == "rectangle":
            return Rectangle(self.values[0], self.values[1])
        if self.kind == "triangle":
            return Triangle(*self.values)
        if self.kind == "polygon":
            return Polygon(self.values[0], self.values[1] if len(self.values) > 1 else [])
        if self.kind == "compound":
            return Compound([Circle(0.55, (-0.65, 0.0)), Circle(0.55, (0.65, 0.0))])
        raise ValueError(f"Unknown shape kind: {self.kind}")


SHAPES = {
    "circle": ShapeDefinition("circle", (0.9,)),
    "rectangle": ShapeDefinition("rectangle", (3.8, 1.8)),
    "triangle": ShapeDefinition("triangle", ((-1.1, -0.7), (1.1, -0.7), (0.0, 1.0))),
    "polygon": ShapeDefinition("polygon", (((-1.2, -0.7), (0.6, -0.9), (1.2, 0.2), (0.0, 1.0), (-1.2, -0.7)), ())),
    "compound": ShapeDefinition("compound"),
}


@dataclass
class SceneObject:
    object_id: int
    name: str
    shape: ShapeDefinition
    mode: str
    pose: Pose
    role: str = "query"
    path: list[tuple[int, Pose]] = field(default_factory=list)
    variants: list[ShapeDefinition] = field(default_factory=list)
    visible: bool = True

    def pose_at(self, time_step: int) -> Pose:
        if not self.path:
            return self.pose
        samples = sorted(self.path, key=lambda item: item[0])
        if time_step <= samples[0][0]:
            return samples[0][1]
        if time_step >= samples[-1][0]:
            return samples[-1][1]
        for (t0, p0), (t1, p1) in pairwise(samples):
            if t0 <= time_step <= t1:
                weight = (time_step - t0) / (t1 - t0)
                delta = (p1.rotation - p0.rotation + math.pi) % (2 * math.pi) - math.pi
                return Pose(
                    (
                        p0.translation[0] + weight * (p1.translation[0] - p0.translation[0]),
                        p0.translation[1] + weight * (p1.translation[1] - p0.translation[1]),
                    ),
                    p0.rotation + weight * delta,
                )
        return self.pose

    def shape_at(self, time_step: int, first_time: int) -> ShapeDefinition:
        if self.mode in {"time_variant", "time_variant_dynamic"} and self.variants:
            return self.variants[(time_step - first_time) % len(self.variants)]
        return self.shape

    def dynamic_obstacle(self, time_steps: tuple[int, ...]):
        poses = [self.pose_at(time_step) for time_step in time_steps]
        if self.mode == "dynamic":
            return DynamicObstacle(self.shape.collision_object(), poses, time_steps[0])
        shapes = [self.shape_at(time_step, time_steps[0]).collision_object() for time_step in time_steps]
        return DynamicObstacle.from_time_variant(shapes, time_steps[0], poses)


@dataclass(frozen=True)
class ObjectResult:
    verdict: Verdict
    detail: str


@dataclass
class PlaygroundState:
    engine: CollisionEngine
    time_steps: tuple[int, ...]
    scenario: Scenario | None = None
    objects: list[SceneObject] = field(default_factory=list)
    selected_id: int | None = None
    current_time: int = 0
    tool: str = "select"
    shape_kind: str = "circle"
    mode: str = "static"
    draft_path: list[Pose] = field(default_factory=list)
    draft_polygon: list[tuple[float, float]] = field(default_factory=list)
    simulating: bool = False
    speed: int = 1
    _next_id: int = 1
    _results: dict[int, ObjectResult] = field(default_factory=dict)
    _interval_results: dict[int, ObjectResult] = field(default_factory=dict)

    def __post_init__(self):
        if self.time_steps and self.current_time not in self.time_steps:
            self.current_time = self.time_steps[0]

    @property
    def selected(self):
        return next((obj for obj in self.objects if obj.object_id == self.selected_id), None)

    def add_object(self, center, *, role="query"):
        shape = SHAPES[self.shape_kind]
        if self.shape_kind == "polygon" and self.draft_polygon:
            shape, center = normalized_polygon(self.draft_polygon)
        pose = Pose.from_translation(center)
        path = []
        if self.mode in {"dynamic", "time_variant_dynamic"}:
            path = timed_path(self.draft_path or [pose], self.time_steps)
        variants = shape_variants(shape) if self.mode in {"time_variant", "time_variant_dynamic"} else []
        obj = SceneObject(
            self._next_id, f"{self.shape_kind} {self._next_id}", shape, self.mode, pose, role, path, variants
        )
        self._next_id += 1
        self.objects.append(obj)
        self.selected_id = obj.object_id
        self.clear_draft()
        return obj

    def add_path_point(self, point):
        self.draft_path.append(Pose.from_translation(point))

    def add_freehand_vertex(self, point):
        self.draft_polygon.append(point)

    def finalize_freehand(self):
        if len(self.draft_polygon) < 3:
            return None
        self.shape_kind = "polygon"
        return self.add_object(self.draft_polygon[0])

    def clear_draft(self):
        self.draft_path.clear()
        self.draft_polygon.clear()

    def delete_selected(self):
        self.objects[:] = [obj for obj in self.objects if obj.object_id != self.selected_id]
        self.selected_id = self.objects[-1].object_id if self.objects else None

    def select_next(self):
        if not self.objects:
            self.selected_id = None
            return None
        ids = [obj.object_id for obj in self.objects]
        self.selected_id = ids[0] if self.selected_id not in ids else ids[(ids.index(self.selected_id) + 1) % len(ids)]
        return self.selected

    def set_engine(self, engine):
        self.engine = engine
        self.evaluate()

    def step(self, amount=1):
        if not self.time_steps:
            return self.current_time
        index = self.time_steps.index(self.current_time)
        self.current_time = self.time_steps[(index + amount) % len(self.time_steps)]
        self.evaluate()
        return self.current_time

    def evaluate(self):
        queries = [obj for obj in self.objects if obj.role == "query"]
        environments = [obj for obj in self.objects if obj.role == "environment"]
        priority = {
            Verdict.EXACT_CLEAR: 0,
            Verdict.CERTIFIED_CLEAR: 0,
            Verdict.UNSUPPORTED: 1,
            Verdict.POTENTIAL_COLLISION: 2,
            Verdict.EXACT_HIT: 3,
        }
        for interval, results in ((False, self._results), (True, self._interval_results)):
            results.clear()
            results.update({query.object_id: self._evaluate_object(query, interval=interval) for query in queries})
            for environment in environments:
                pair_results = [self._evaluate_object(environment, obstacles=(), interval=interval)] + [
                    self._evaluate_object(query, obstacles=(environment,), include_scenario=False, interval=interval)
                    for query in queries
                ]
                if pair_results:
                    results[environment.object_id] = max(pair_results, key=lambda result: priority[result.verdict])
        return self._results

    def _evaluate_object(self, query, *, obstacles=None, include_scenario=True, interval=False):
        try:
            builder = CollisionCheckerBuilder(engine=self.engine)
            if include_scenario and self.scenario is not None:
                builder = scenario_builder(self.scenario, builder)
            candidates = obstacles if obstacles is not None else self.objects
            for obj in candidates:
                if obj.object_id == query.object_id or (obstacles is None and obj.role != "environment"):
                    continue
                if obj.mode == "static":
                    builder.with_static_obstacle(positioned_shape(obj, self.current_time, self.time_steps[0]))
                else:
                    builder.with_dynamic_obstacle(obj.dynamic_obstacle(self.time_steps))
            checker = builder.build()
            max_time = self.current_time
            if interval and self.time_steps:
                index = self.time_steps.index(self.current_time)
                max_time = self.time_steps[min(index + 1, len(self.time_steps) - 1)]
            if not interval:
                status = checker.collides_static(
                    query.shape_at(self.current_time, self.time_steps[0]).collision_object(),
                    query.pose_at(self.current_time),
                    min_time=self.current_time,
                    max_time=self.current_time,
                )
                return ObjectResult(Verdict.EXACT_HIT if status.collides else Verdict.EXACT_CLEAR, str(status))
            if query.mode == "static":
                status = checker.collides_static(
                    query.shape_at(self.current_time, self.time_steps[0]).collision_object(),
                    query.pose_at(self.current_time),
                    min_time=self.current_time,
                    max_time=max_time,
                )
                return ObjectResult(
                    Verdict.POTENTIAL_COLLISION if status.collides else Verdict.CERTIFIED_CLEAR,
                    str(status),
                )
            status = checker.collides_dynamic(query.dynamic_obstacle(self.time_steps), self.current_time, max_time)
            return ObjectResult(
                Verdict.POTENTIAL_COLLISION if status.collides else Verdict.CERTIFIED_CLEAR,
                str(status),
            )
        except Exception as error:
            return ObjectResult(Verdict.UNSUPPORTED, f"{type(error).__name__}: {error}")

    def load_preset(self, name, bounds):
        self.objects.clear()
        self.selected_id = None
        self._next_id = 1
        if name == "Empty":
            self.evaluate()
            return
        (cx, cy), angle = self._preset_frame(bounds)

        def pose(x, y=0.0, rotation=0.0):
            cosine, sine = math.cos(angle), math.sin(angle)
            return Pose((cx + cosine * x - sine * y, cy + sine * x + cosine * y), angle + rotation)

        if name == "Tunneling":
            self.shape_kind, self.mode = "rectangle", "static"
            environment = self.add_object((cx, cy), role="environment")
            environment.pose = pose(0.0)
            self.shape_kind, self.mode = "circle", "dynamic"
            self.draft_path = [pose(-8.0), pose(8.0)]
            self.add_object(pose(-8.0).translation)
        elif name == "Intersection":
            for index, (start, end) in enumerate(
                ((pose(-8.0), pose(8.0)), (pose(0.0, -8.0, math.pi / 2), pose(0.0, 8.0, math.pi / 2)))
            ):
                self.shape_kind, self.mode = "rectangle", "dynamic"
                self.draft_path = [start, end]
                self.add_object(start.translation, role="environment" if index == 0 else "query")
        elif name == "Overtaking":
            for offset, points in ((0, [pose(-5.0), pose(5.0)]), (1, [pose(-9.0), pose(0.0, 3.0), pose(7.0)])):
                self.shape_kind, self.mode = "rectangle", "dynamic"
                self.draft_path = points
                vehicle = self.add_object(points[0].translation, role="environment" if offset == 0 else "query")
                vehicle.name = "lead vehicle" if offset == 0 else "overtaking vehicle"
        self.evaluate()

    def _preset_frame(self, bounds):
        if self.scenario is not None:
            checker = scenario_builder(self.scenario, CollisionCheckerBuilder(engine=self.engine)).build()
            time_step = self.time_steps[0]
            for lanelet in self.scenario.lanelet_network.lanelets:
                points = np.asarray(lanelet.center_vertices)
                for start, end in pairwise(points):
                    direction = end - start
                    length = np.linalg.norm(direction)
                    if not length:
                        continue
                    direction /= length
                    center = (start + end) / 2
                    angle = math.atan2(direction[1], direction[0])
                    poses = [Pose(tuple(center + offset * direction), angle) for offset in (-8.0, 0.0, 8.0)]
                    shapes = [Circle(0.9), Rectangle(3.8, 1.8), Circle(0.9)]
                    if all(
                        not checker.collides_static(shape, candidate, min_time=time_step, max_time=time_step).collides
                        for shape, candidate in zip(shapes, poses, strict=True)
                    ):
                        return tuple(center), angle
        xmin, xmax, ymin, ymax = bounds
        return ((xmin + xmax) / 2, (ymin + ymax) / 2), 0.0


@dataclass(frozen=True)
class InspectorState:
    start_x: float = -4.0
    end_x: float = 4.0
    obstacle_x: float = 0.0

    def sweep(self):
        return Sweep(
            "inspector",
            Circle(0.5),
            Pose.from_translation((self.start_x, 0.0)),
            Pose.from_translation((self.end_x, 0.0)),
            Rectangle(0.3, 3.0),
            Pose.from_translation((self.obstacle_x, 0.0)),
        )


def inspect(state: InspectorState, engine: CollisionEngine):
    return evaluate_sweep(state.sweep(), engine)


def draw_inspector(ax, state: InspectorState, engine: CollisionEngine):
    from matplotlib.patches import Circle as CirclePatch, Rectangle as RectanglePatch

    results = inspect(state, engine)
    ax.clear()
    ax.set_xlim(
        min(state.start_x, state.end_x, state.obstacle_x) - 2,
        max(state.start_x, state.end_x, state.obstacle_x) + 2,
    )
    ax.set_ylim(-2.2, 2.2)
    ax.set_aspect("equal")
    ax.plot([state.start_x, state.end_x], [0, 0], "--", color=COLOR_PATH)
    ax.add_patch(RectanglePatch((state.obstacle_x - 0.15, -1.5), 0.3, 3, color="#6b7280", alpha=0.65))
    ax.add_patch(CirclePatch((state.start_x, 0), 0.5, color=COLOR_CLEAR, alpha=0.7))
    ax.add_patch(CirclePatch((state.end_x, 0), 0.5, color=COLOR_CLEAR, alpha=0.7))
    ax.set_title(results[-1][1])
    return results


def timed_path(poses, time_steps):
    if not poses:
        return []
    if len(poses) == 1 or len(time_steps) == 1:
        return [(time_steps[0], poses[0])]
    indices = np.linspace(0, len(time_steps) - 1, len(poses)).round().astype(int)
    return [(time_steps[index], pose) for index, pose in zip(indices, poses, strict=True)]


def normalized_polygon(points):
    if len(points) < 3:
        raise ValueError("a polygon needs at least three vertices")
    array = np.asarray(points, dtype=float)
    if not np.isfinite(array).all():
        raise ValueError("polygon vertices must be finite")
    center = tuple(array.mean(axis=0))
    local = [(float(x - center[0]), float(y - center[1])) for x, y in array]
    local.append(local[0])
    shape = ShapeDefinition("polygon", (tuple(local), ()))
    shape.collision_object()  # Use the library's topology validation before accepting the draft.
    return shape, center


def shape_variants(shape):
    if shape.kind == "circle":
        return [
            shape,
            ShapeDefinition("circle", (shape.values[0] * 1.35,)),
            ShapeDefinition("circle", (shape.values[0] * 0.7,)),
        ]
    if shape.kind == "rectangle":
        length, width = shape.values
        return [
            shape,
            ShapeDefinition("rectangle", (length * 1.2, width * 0.8)),
            ShapeDefinition("rectangle", (length * 0.8, width * 1.2)),
        ]
    return [shape, SHAPES["circle"], SHAPES["rectangle"]]


def positioned_shape(obj, time_step, first_time):
    definition = obj.shape_at(time_step, first_time)
    pose = obj.pose_at(time_step)
    if definition.kind == "circle":
        return Circle(definition.values[0], pose.translation)
    if definition.kind == "rectangle":
        return Rectangle(definition.values[0], definition.values[1], pose.rotation, pose.translation)
    if definition.kind in {"triangle", "polygon"}:
        points = definition.values if definition.kind == "triangle" else definition.values[0]
        cosine, sine = math.cos(pose.rotation), math.sin(pose.rotation)
        transformed = [
            (
                pose.translation[0] + cosine * x - sine * y,
                pose.translation[1] + sine * x + cosine * y,
            )
            for x, y in points
        ]
        return Triangle(*transformed) if definition.kind == "triangle" else Polygon(transformed)
    centers = [(-0.65, 0.0), (0.65, 0.0)]
    return Compound(
        [
            Circle(
                0.55,
                (
                    pose.translation[0] + math.cos(pose.rotation) * x,
                    pose.translation[1] + math.sin(pose.rotation) * x,
                ),
            )
            for x, _y in centers
        ]
    )


def verdict_color(result):
    if result is None:
        return COLOR_CLEAR
    return {
        Verdict.EXACT_HIT: COLOR_EXACT_HIT,
        Verdict.EXACT_CLEAR: COLOR_CLEAR,
        Verdict.POTENTIAL_COLLISION: COLOR_POTENTIAL,
        Verdict.CERTIFIED_CLEAR: COLOR_CLEAR,
        Verdict.UNSUPPORTED: COLOR_UNSUPPORTED,
    }[result.verdict]


def draw_shape(ax, definition, pose, color, *, selected=False, edge_color=None, alpha=0.65):
    from matplotlib import patches
    from matplotlib.transforms import Affine2D

    transform = Affine2D().rotate(pose.rotation).translate(*pose.translation) + ax.transData
    edge = COLOR_SELECTED if selected else edge_color or color
    width = 3 if selected else 1.8
    if definition.kind == "circle":
        artist = patches.Circle(
            (0, 0),
            definition.values[0],
            facecolor=color,
            edgecolor=edge,
            alpha=alpha,
            linewidth=width,
            transform=transform,
        )
    elif definition.kind == "rectangle":
        length, height = definition.values
        artist = patches.Rectangle(
            (-length / 2, -height / 2),
            length,
            height,
            facecolor=color,
            edgecolor=edge,
            alpha=alpha,
            linewidth=width,
            transform=transform,
        )
    elif definition.kind == "triangle":
        artist = patches.Polygon(
            definition.values,
            closed=True,
            facecolor=color,
            edgecolor=edge,
            alpha=alpha,
            linewidth=width,
            transform=transform,
        )
    elif definition.kind == "polygon":
        artist = patches.Polygon(
            definition.values[0],
            closed=True,
            facecolor=color,
            edgecolor=edge,
            alpha=alpha,
            linewidth=width,
            transform=transform,
        )
    else:
        artist = patches.Ellipse(
            (0, 0), 2.4, 1.1, facecolor=color, edgecolor=edge, alpha=alpha, linewidth=width, transform=transform
        )
    artist.set_zorder(50)
    ax.add_patch(artist)
    return artist


def draw_scene(ax, scenario, state, bounds, *, reset_view=False):
    view = None if reset_view else (ax.get_xlim(), ax.get_ylim())
    ax.clear()
    if scenario is not None:
        from commonroad.visualization.draw_params import MPDrawParams
        from commonroad.visualization.mp_renderer import MPRenderer

        params = MPDrawParams(time_begin=state.current_time, time_end=state.current_time)
        renderer = MPRenderer(draw_params=params, plot_limits=list(bounds), ax=ax)
        scenario.draw(renderer, params)
        renderer.render()
    state.evaluate()
    artists = []
    for obj in state.objects:
        if not obj.visible:
            continue
        pose = obj.pose_at(state.current_time)
        shape = obj.shape_at(state.current_time, state.time_steps[0])
        result = state._results.get(obj.object_id)
        interval_result = state._interval_results.get(obj.object_id)
        artists.append(
            draw_shape(
                ax,
                shape,
                pose,
                verdict_color(result),
                selected=obj.object_id == state.selected_id,
                edge_color=verdict_color(interval_result),
            )
        )
        if obj.path:
            points = np.asarray([sample_pose.translation for _time, sample_pose in obj.path])
            artists.append(
                ax.plot(
                    points[:, 0], points[:, 1], "--", color=verdict_color(interval_result), linewidth=1.2, zorder=45
                )[0]
            )
    if state.draft_path:
        points = np.asarray([pose.translation for pose in state.draft_path])
        artists.append(
            ax.plot(
                points[:, 0],
                points[:, 1],
                "--o",
                color=COLOR_PATH,
                markerfacecolor="none",
                markersize=4,
                linewidth=1.2,
                zorder=60,
            )[0]
        )
    if state.draft_polygon:
        points = np.asarray(state.draft_polygon)
        artists.append(ax.plot(points[:, 0], points[:, 1], "-o", color=COLOR_SELECTED, zorder=60)[0])
    ax.set_aspect("equal", adjustable="box")
    if view is None:
        ax.set_xlim(bounds[0], bounds[1])
        ax.set_ylim(bounds[2], bounds[3])
    else:
        ax.set_xlim(view[0])
        ax.set_ylim(view[1])
    selected = state.selected
    result = state._results.get(selected.object_id) if selected else None
    interval_result = state._interval_results.get(selected.object_id) if selected else None
    status = (
        f"current: {result.verdict.value} | next interval: {interval_result.verdict.value}"
        if result and interval_result
        else "select or add a query object"
    )
    ax.set_title(f"t={state.current_time} | {state.engine} | {status}")
    return artists


def run(scenario, checker, pose_bounds):
    # ponytail: scenario_path was never used; callers pass the engine via checker.
    from matplotlib import pyplot as plt
    from matplotlib.widgets import Button, RadioButtons, Slider

    from examples.utils import scenario_time_steps

    time_steps = tuple(scenario_time_steps(scenario))
    bounds = (pose_bounds[0][0], pose_bounds[1][0], pose_bounds[0][1], pose_bounds[1][1])
    state = PlaygroundState(checker.engine, time_steps, scenario)
    state.load_preset("Tunneling", bounds)

    fig, ax = plt.subplots(figsize=(14, 8))
    fig.subplots_adjust(left=0.2, right=0.82, bottom=0.16, top=0.94)
    preset = RadioButtons(fig.add_axes((0.015, 0.76, 0.16, 0.17)), ["Tunneling", "Intersection", "Overtaking", "Empty"])
    tool = RadioButtons(
        fig.add_axes((0.015, 0.57, 0.16, 0.15)), ["Select / move", "Add object", "Draw path", "Draw polygon"]
    )
    shape = RadioButtons(fig.add_axes((0.015, 0.31, 0.16, 0.22)), list(SHAPES))
    mode = RadioButtons(
        fig.add_axes((0.015, 0.08, 0.16, 0.19)), ["static", "dynamic", "time_variant", "time_variant_dynamic"]
    )
    engine = RadioButtons(
        fig.add_axes((0.84, 0.72, 0.14, 0.18)),
        ["rhusics", "parry", "collide"],
        active=["rhusics", "parry", "collide"].index(str(checker.engine).split(".")[-1].lower()),
    )
    role = RadioButtons(fig.add_axes((0.84, 0.54, 0.14, 0.13)), ["query", "environment"])
    play = Button(fig.add_axes((0.84, 0.43, 0.065, 0.05)), "play")
    step = Button(fig.add_axes((0.915, 0.43, 0.065, 0.05)), "step")
    delete = Button(fig.add_axes((0.84, 0.35, 0.14, 0.05)), "delete selected")
    finalize = Button(fig.add_axes((0.84, 0.27, 0.14, 0.05)), "finalize polygon")
    reset_view = Button(fig.add_axes((0.84, 0.19, 0.14, 0.05)), "reset view")
    timeline = Slider(fig.add_axes((0.24, 0.065, 0.53, 0.035)), "time", 0, len(time_steps) - 1, valinit=0, valstep=1)

    def redraw(*, reset=False):
        draw_scene(ax, scenario, state, bounds, reset_view=reset)
        fig.canvas.draw_idle()

    def set_preset(value):
        state.load_preset(value, bounds)
        redraw(reset=True)

    preset.on_clicked(set_preset)

    def set_tool(value):
        if value is not None:
            state.tool = {
                "Select / move": "select",
                "Add object": "add",
                "Draw path": "path",
                "Draw polygon": "polygon",
            }[value]

    tool.on_clicked(set_tool)
    shape.on_clicked(lambda value: setattr(state, "shape_kind", value))
    mode.on_clicked(lambda value: setattr(state, "mode", value))
    role.on_clicked(
        lambda value: (
            setattr(state.selected, "role", value) if state.selected else None,
            state.evaluate(),
            redraw(),
        )
    )

    def set_engine(value):
        if value is not None:
            state.set_engine(
                {
                    "rhusics": CollisionEngine.Rhusics,
                    "parry": CollisionEngine.Parry,
                    "collide": CollisionEngine.Collide,
                }[value]
            )
            redraw()

    engine.on_clicked(set_engine)
    timeline.on_changed(lambda value: (setattr(state, "current_time", time_steps[int(value)]), redraw()))
    delete.on_clicked(lambda _event: (state.delete_selected(), redraw()))
    finalize.on_clicked(lambda _event: (state.finalize_freehand(), redraw()))
    reset_view.on_clicked(lambda _event: redraw(reset=True))
    step.on_clicked(lambda _event: (state.step(), timeline.set_val(time_steps.index(state.current_time))))

    timer = fig.canvas.new_timer(interval=180)

    def tick():
        if state.simulating:
            state.step(state.speed)
            timeline.set_val(time_steps.index(state.current_time))

    timer.add_callback(tick)
    play.on_clicked(
        lambda _event: (
            setattr(state, "simulating", not state.simulating),
            timer.start() if state.simulating else timer.stop(),
        )
    )

    drag: DragState = {"id": None, "last": None}

    def press(event):
        if event.inaxes != ax or event.xdata is None:
            return
        point = (event.xdata, event.ydata)
        if state.tool == "add":
            state.add_object(point, role=role.value_selected or "query")
        elif state.tool == "path":
            if state.selected:
                state.selected.path.append((state.current_time, Pose.from_translation(point)))
                if state.selected.mode == "static":
                    state.selected.mode = "dynamic"
            else:
                state.add_path_point(point)
        elif state.tool == "polygon":
            state.add_freehand_vertex(point)
        else:
            clicked_objects = []
            click_geom = Circle(0.1)
            click_pose = Pose.from_translation(point)
            for obj in state.objects:
                obj_geom = obj.shape_at(state.current_time, state.time_steps[0]).collision_object()
                obj_pose = obj.pose_at(state.current_time)
                try:
                    if obj_geom.collides(click_geom, obj_pose, click_pose, engine=state.engine):
                        dist = np.linalg.norm(np.asarray(obj_pose.translation) - point)
                        clicked_objects.append((dist, obj))
                except Exception:
                    dist = np.linalg.norm(np.asarray(obj_pose.translation) - point)
                    if dist < 1.0:
                        clicked_objects.append((dist, obj))

            if clicked_objects:
                clicked_objects.sort(key=lambda x: x[0])
                nearest = clicked_objects[0][1]
                state.selected_id = nearest.object_id
                drag.update(id=nearest.object_id, last=point)
            else:
                state.selected_id = None
        redraw()

    def motion(event):
        object_id, last = drag["id"], drag["last"]
        if object_id is None or last is None or event.inaxes != ax or event.xdata is None:
            return
        obj = next(item for item in state.objects if item.object_id == object_id)
        dx, dy = event.xdata - last[0], event.ydata - last[1]
        obj.pose = Pose((obj.pose.translation[0] + dx, obj.pose.translation[1] + dy), obj.pose.rotation)
        obj.path = [
            (time, Pose((pose.translation[0] + dx, pose.translation[1] + dy), pose.rotation)) for time, pose in obj.path
        ]
        drag["last"] = (event.xdata, event.ydata)
        draw_scene(ax, scenario, state, bounds)
        fig.canvas.draw_idle()

    def release(_event):
        drag.update(id=None, last=None)
        redraw()

    fig.canvas.mpl_connect("button_press_event", press)
    fig.canvas.mpl_connect("motion_notify_event", motion)
    fig.canvas.mpl_connect("button_release_event", release)
    fig._crcc_widgets = preset, tool, shape, mode, engine, role, play, step, delete, finalize, reset_view, timeline
    fig._crcc_timer = timer
    redraw(reset=True)
    plt.show()
    return state
