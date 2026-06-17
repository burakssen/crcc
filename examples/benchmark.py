from crcc.collision_object import Rectangle

from examples.utils import CAR_SIZE, count_collisions, sample_poses, timed

BENCHMARK_SAMPLE_COUNT = 100_000


def run(checker, pose_bounds, sample_count=BENCHMARK_SAMPLE_COUNT):
    """Run sequential vs parallel batch collision checker benchmarks."""
    car = Rectangle(*CAR_SIZE)
    poses = sample_poses(sample_count, pose_bounds)
    positioned_cars = [(car, pose) for pose in poses]

    parallel_results, parallel_elapsed = timed(
        lambda: checker.par_static(positioned_cars),
    )
    print(f"Parallel any-time checks: {parallel_elapsed:.4f} seconds, {count_collisions(parallel_results)} collisions")

    sequential_results, sequential_elapsed = timed(
        lambda: [checker.collides_static(car, pose) for car, pose in positioned_cars],
    )
    print(
        f"Sequential any-time checks: {sequential_elapsed:.4f} seconds, "
        f"{count_collisions(sequential_results)} collisions"
    )
