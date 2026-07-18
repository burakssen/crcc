import math
from dataclasses import dataclass, field

from crcc import Circle, CollisionCheckerBuilder, CollisionEngine, DynamicObstacle, Pose, Rectangle

from examples.presentation import ResultRow, collision_result, print_results


@dataclass(frozen=True)
class Sweep:
    name: str
    moving: object
    start: Pose
    end: Pose
    obstacle: object
    obstacle_pose: Pose = field(default_factory=Pose.identity)


def evaluate_sweep(sweep: Sweep, engine: CollisionEngine) -> tuple[ResultRow, ResultRow, ResultRow]:
    start_hit = sweep.moving.collides(sweep.obstacle, sweep.start, sweep.obstacle_pose, engine)
    end_hit = sweep.moving.collides(sweep.obstacle, sweep.end, sweep.obstacle_pose, engine)
    return (
        (f"{sweep.name} start", "hit" if start_hit else "clear", f"overlap={start_hit}"),
        (f"{sweep.name} end", "hit" if end_hit else "clear", f"overlap={end_hit}"),
        collision_result(
            f"{sweep.name} interval",
            lambda: sweep.moving.collides_continuous(
                sweep.start, sweep.end, sweep.obstacle, sweep.obstacle_pose, sweep.obstacle_pose, engine
            ),
            true="potential collision",
            false="certified clear",
        ),
    )


def continuous_results(engine: CollisionEngine) -> tuple[ResultRow, ...]:
    tunneling = Sweep(
        "translation",
        Circle(0.5),
        Pose.from_translation((-4.0, 0.0)),
        Pose.from_translation((4.0, 0.0)),
        Rectangle(0.3, 3.0),
    )
    rotation = Sweep(
        "rotation",
        Rectangle(4.0, 0.4),
        Pose((0.0, 0.0), -math.pi / 2),
        Pose((0.0, 0.0), math.pi / 2),
        Circle(0.3, (1.5, 0.0)),
    )
    results = (*evaluate_sweep(tunneling, engine), *evaluate_sweep(rotation, engine))

    trajectory = DynamicObstacle(
        Circle(0.5),
        [Pose.from_translation((-3.0, 0.0)), Pose.from_translation((3.0, 0.0)), Pose.from_translation((5.0, 0.0))],
        10,
    )
    checker = CollisionCheckerBuilder(engine=engine).with_static_obstacle(Rectangle(0.3, 3.0)).build()
    for time_step in (10, 11):
        status = checker.collides_dynamic(trajectory, min_time=time_step, max_time=time_step)
        results += ((f"dynamic sample t={time_step}", "hit" if status.collides else "clear", str(status)),)
    return results


def run(engine: CollisionEngine):
    results = continuous_results(engine)
    print_results(f"Continuous collision detection | engine={engine}", results)
    print("  Contract: False certifies the whole interval is clear; True may be conservative.")
    return results
