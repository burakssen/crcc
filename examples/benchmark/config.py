import os
from dataclasses import dataclass
from pathlib import Path

from crcc.collision_checker import CollisionEngine

SCHEMA_VERSION = "3"
BENCHMARK_SAMPLE_COUNT = 20_000
BENCHMARK_REPETITIONS = 9
WARMUP_QUERY_COUNT = 100
MIN_RECOMMENDED_SAMPLE_COUNT = 100
DEFAULT_OUTPUT_DIR = Path("target/crcc-python-bench")
DEFAULT_DENSITIES = (0.0, 0.01, 0.10, 0.25, 0.50, 0.75, 1.0)
DEFAULT_SCENE_SIZES = (100, 500, 1_000, 5_000, 10_000, 50_000, 100_000, 500_000)
DEFAULT_THREAD_COUNTS = (1, 2, 4, 8, 16)

ENGINE_ITEMS = (
    ("parry", CollisionEngine.Parry),
    ("rhusics", CollisionEngine.Rhusics),
    ("collide", CollisionEngine.Collide),
)
ENGINE_BY_NAME = dict(ENGINE_ITEMS)


@dataclass(frozen=True)
class BenchmarkConfig:
    scenario_paths: tuple[Path, ...]
    sample_count: int = BENCHMARK_SAMPLE_COUNT
    repetitions: int = BENCHMARK_REPETITIONS
    output_dir: Path = DEFAULT_OUTPUT_DIR
    seed: int = 2026
    thread_counts: tuple[int, ...] = ()
    engines: tuple[str, ...] = tuple(name for name, _ in ENGINE_ITEMS)
    step: str = "all"

    @classmethod
    def from_args(
        cls,
        *,
        scenario_paths=None,
        sample_count: int = BENCHMARK_SAMPLE_COUNT,
        repetitions: int = BENCHMARK_REPETITIONS,
        output_dir=DEFAULT_OUTPUT_DIR,
        seed: int = 2026,
        thread_counts=None,
        engines=None,
        step: str = "all",
    ):
        selected_scenarios = (
            discover_scenario_paths() if scenario_paths is None else tuple(Path(path) for path in scenario_paths)
        )
        selected_engines = tuple(name.lower() for name in (engines or ENGINE_BY_NAME))
        unknown = sorted(set(selected_engines) - set(ENGINE_BY_NAME))
        if unknown:
            raise ValueError(f"unknown benchmark engine(s): {', '.join(unknown)}")
        if step not in {"run", "plot", "all"}:
            raise ValueError("benchmark step must be one of: run, plot, all")
        return cls(
            scenario_paths=tuple(selected_scenarios),
            sample_count=max(1, int(sample_count)),
            repetitions=max(1, int(repetitions)),
            output_dir=Path(output_dir),
            seed=int(seed),
            thread_counts=normalize_thread_counts(thread_counts),
            engines=selected_engines,
            step=step,
        )


def discover_scenario_paths(scenario_dir: str | Path = "scenarios"):
    return tuple(sorted(Path(scenario_dir).glob("*.xml")))


def normalize_thread_counts(thread_counts):
    if thread_counts is None:
        cpu_count = os.cpu_count() or max(DEFAULT_THREAD_COUNTS)
        counts = [threads for threads in DEFAULT_THREAD_COUNTS if threads <= cpu_count] or [1]
    else:
        counts = thread_counts
    return tuple(sorted({max(1, int(threads)) for threads in counts}))


def selected_engine_items(names: tuple[str, ...]):
    return tuple((name, ENGINE_BY_NAME[name]) for name in names)
