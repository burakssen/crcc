import os
from dataclasses import dataclass
from pathlib import Path

from crcc import CollisionEngine

SCHEMA_VERSION = "10"
BENCHMARK_SAMPLE_COUNT = 20_000
BENCHMARK_REPETITIONS = 5
WARMUP_QUERY_COUNT = 100
MIN_RECOMMENDED_SAMPLE_COUNT = 100
DEFAULT_OUTPUT_DIR = Path("target/crcc-python-bench")
DEFAULT_DENSITIES = (0.0, 0.50)
DEFAULT_SCENE_SIZES = (100, 1_000)
SPEC_SCENE_SIZES = (100, 1_000, 5_000, 10_000, 25_000, 50_000)
STRESS_SCENE_SIZES = (100_000,)
DEFAULT_SPEC_SHAPE_COUNTS = (16, 64, 256, 1_024)
DEFAULT_COMPOUND_CHILD_COUNTS = (1, 4, 16, 64, 256)
DEFAULT_UPDATE_TRANSFORMS = ("translation", "rotation", "translation_rotation", "randomized")
DEFAULT_DENSITY_LABELS = ("clear", "medium", "dense", "worst_case")
DEFAULT_THREAD_COUNTS = (1, 2, 4, 8)
MATRIX_SHAPE_FAMILIES = ("circle", "rectangle", "polygon32", "compound16_polygon32")
BENCHMARK_SUITES = (
    "pair",
    "continuous",
    "distance",
    "shape_complexity",
    "coverage_matrix",
    "scene_scaling",
    "update_proxy",
    "rebuild_update",
    "api_overhead",
    "dynamic_batch",
    "time_variant",
    "native_layers",
    "parallel",
    "density_scaling",
    "dynamic_scene",
    "scenario",
)

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
    profile: str = "smoke"
    suites: tuple[str, ...] = BENCHMARK_SUITES
    include_stress: bool = False

    @classmethod
    def from_args(
        cls,
        *,
        scenario_paths=None,
        sample_count: int | None = None,
        repetitions: int | None = None,
        output_dir=None,
        seed: int = 2026,
        thread_counts=None,
        engines=None,
        step: str = "all",
        profile: str = "smoke",
        suites=None,
        include_stress: bool = False,
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
        if profile not in {"smoke", "spec"}:
            raise ValueError("benchmark profile must be one of: smoke, spec")
        selected_suites = normalize_suites(suites)
        return cls(
            scenario_paths=tuple(selected_scenarios),
            sample_count=max(1, int(BENCHMARK_SAMPLE_COUNT if sample_count is None else sample_count)),
            repetitions=max(1, int(BENCHMARK_REPETITIONS if repetitions is None else repetitions)),
            output_dir=Path(DEFAULT_OUTPUT_DIR if output_dir is None else output_dir),
            seed=int(seed),
            thread_counts=normalize_thread_counts(thread_counts),
            engines=selected_engines,
            step=step,
            profile=profile,
            suites=selected_suites,
            include_stress=bool(include_stress),
        )


def discover_scenario_paths(scenario_dir: str | Path = "scenarios"):
    return tuple(sorted(Path(scenario_dir).glob("*.xml")))


def normalize_thread_counts(thread_counts):
    cpu_count = os.cpu_count() or max(DEFAULT_THREAD_COUNTS)
    if thread_counts is None:
        counts = [threads for threads in DEFAULT_THREAD_COUNTS if threads <= cpu_count] or [1]
    else:
        counts = [threads for threads in thread_counts if int(threads) <= cpu_count] or [cpu_count]
    return tuple(sorted({max(1, int(threads)) for threads in counts}))


def selected_engine_items(names: tuple[str, ...]):
    return tuple((name, ENGINE_BY_NAME[name]) for name in names)


def normalize_suites(suites):
    if isinstance(suites, str):
        suites = (suites,)
    if suites is None or suites == ["all"] or suites == ("all",):
        return BENCHMARK_SUITES
    selected = tuple(str(suite).lower() for suite in suites)
    if "all" in selected:
        return BENCHMARK_SUITES
    unknown = sorted(set(selected) - set(BENCHMARK_SUITES))
    if unknown:
        raise ValueError(f"unknown benchmark suite(s): {', '.join(unknown)}")
    return tuple(dict.fromkeys(selected))
