import numpy as np
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from crcc.collision_object import Rectangle
from crcc.pose import Pose
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle as RectanglePatch
from matplotlib.transforms import Affine2D
from matplotlib.widgets import Slider

from examples.utils import CAR_SIZE, scenario_time_steps

COLOR_COLLIDED = "red"
COLOR_CLEAR = "green"


def run(scenario, checker, scenario_path, pose_bounds):
    """Run an interactive Matplotlib vehicle playground overlayed on the scenario network."""
    car = Rectangle(*CAR_SIZE)
    time_steps = scenario_time_steps(scenario)
    current_time_step = time_steps[0]

    fig, ax = plt.subplots(figsize=(10, 7))
    plt.subplots_adjust(bottom=0.25)  # Make room for the slider

    plot_limits = [pose_bounds[0][0], pose_bounds[1][0], pose_bounds[0][1], pose_bounds[1][1]]

    state = {
        "x": (pose_bounds[0][0] + pose_bounds[1][0]) / 2.0,
        "y": (pose_bounds[0][1] + pose_bounds[1][1]) / 2.0,
        "angle": 0.0,
        "time_step": current_time_step,
        "car_patch": None,
    }

    ax_slider = plt.axes((0.15, 0.08, 0.7, 0.03))
    slider = Slider(
        ax_slider,
        "Time Step",
        min(time_steps),
        max(time_steps),
        valinit=current_time_step,
        valfmt="%d",
        valstep=time_steps,
    )

    def draw_scene():
        ax.clear()
        draw_params = MPDrawParams(time_begin=state["time_step"], time_end=state["time_step"])
        renderer = MPRenderer(draw_params=draw_params, plot_limits=plot_limits, ax=ax)
        scenario.draw(renderer, draw_params)
        renderer.render()

        state["car_patch"] = RectanglePatch(
            (-CAR_SIZE[0] / 2.0, -CAR_SIZE[1] / 2.0),
            CAR_SIZE[0],
            CAR_SIZE[1],
            facecolor=COLOR_CLEAR,
            edgecolor=COLOR_CLEAR,
            alpha=0.6,
            zorder=20,
        )
        ax.add_patch(state["car_patch"])
        update_car_pose_and_collision()

    def update_car_pose_and_collision():
        x, y, angle = state["x"], state["y"], state["angle"]
        t = int(state["time_step"])

        pose = Pose((x, y), angle)
        status = checker.collides_static(car, position=pose, min_time=t, max_time=t)

        color = COLOR_COLLIDED if status.collides else COLOR_CLEAR
        state["car_patch"].set_facecolor(color)
        state["car_patch"].set_edgecolor(color)

        transform = Affine2D().rotate(angle).translate(x, y) + ax.transData
        state["car_patch"].set_transform(transform)

        status_text = f"COLLISION at t={t}" if status.collides else "Clear"
        ax.set_title(
            f"Interactive Ego Vehicle Playground\nMove Mouse: Translate | Scroll: Rotate | Status: {status_text}",
            color=color if status.collides else "black",
        )
        fig.canvas.draw_idle()

    def on_move(event):
        if event.inaxes != ax:
            return
        state["x"] = event.xdata
        state["y"] = event.ydata
        update_car_pose_and_collision()

    def on_scroll(event):
        if event.inaxes != ax:
            return
        rotation_delta = np.radians(5.0) if event.button == "up" else -np.radians(5.0)
        state["angle"] += rotation_delta
        update_car_pose_and_collision()

    def on_slider_change(val):
        state["time_step"] = int(val)
        draw_scene()

    slider.on_changed(on_slider_change)
    fig.canvas.mpl_connect("motion_notify_event", on_move)
    fig.canvas.mpl_connect("scroll_event", on_scroll)

    draw_scene()
    plt.show()
