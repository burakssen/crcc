import inspect
import re
from pathlib import Path

import crcc
from commonroad.common.file_reader import CommonRoadFileReader
from crcc import Circle, CollisionCheckerBuilder, DynamicObstacle, Pose, Rectangle
from crcc.commonroad import create_collision_checker_from_scenario

ROOT = Path(__file__).resolve().parents[1]
PUBLIC_GUIDES = (ROOT / "README.md", ROOT / "docs/usage.md", ROOT / "docs/python-api.md", ROOT / "docs/rust-api.md")
MARKDOWN_LINK = re.compile(r"(?<!!)\[[^]]+\]\(([^)]+)\)")


def test_documented_python_workflows_execute():
    left = Circle(1.0)
    right = Circle(1.0)
    assert not left.collides(right, pos_other=Pose.from_translation((3.0, 0.0)))
    assert left.distance(right, pos_other=Pose.from_translation((3.0, 0.0))) == 1.0

    checker = CollisionCheckerBuilder().with_static_obstacle(Rectangle(2.0, 2.0)).build()
    assert checker.collides_static(Circle(0.25), Pose.identity()).collides
    batch = checker.par_static([(Circle(0.25), Pose.identity()), (Circle(0.25), Pose.from_translation((5.0, 0.0)))])
    assert [status.collides for status in batch] == [True, False]

    moving = DynamicObstacle(
        Circle(0.25),
        [Pose.from_translation((-2.0, 0.0)), Pose.from_translation((2.0, 0.0))],
        10,
    )
    dynamic_checker = CollisionCheckerBuilder().with_static_obstacle(Rectangle(0.25, 3.0)).build()
    assert dynamic_checker.collides_dynamic(moving, min_time=10, max_time=11).collides
    assert dynamic_checker.par_dynamic([moving], min_time=10, max_time=11)[0].collides


def test_documented_commonroad_workflow_executes():
    scenario_path = ROOT / "scenarios/DEU_MerzenichRather-2_870_T-149.xml"
    scenario, _ = CommonRoadFileReader(scenario_path).open()
    checker = create_collision_checker_from_scenario(scenario, CollisionCheckerBuilder()).build()
    assert checker.engine == crcc.CollisionEngine.Parry


def test_every_root_symbol_has_runtime_documentation():
    assert set(crcc.__all__) == {
        "Circle",
        "CollisionChecker",
        "CollisionCheckerBuilder",
        "CollisionEngine",
        "CollisionObject",
        "CollisionStatus",
        "Compound",
        "DynamicObstacle",
        "Empty",
        "FullSpace",
        "HalfSpace",
        "Polygon",
        "Pose",
        "Rectangle",
        "Triangle",
    }
    for name in crcc.__all__:
        documentation = inspect.getdoc(getattr(crcc, name))
        assert documentation is not None and len(documentation) >= 20, name


def test_local_markdown_links_resolve():
    markdown_files = (*PUBLIC_GUIDES, ROOT / "tools/benchmark/README.md")
    for source in markdown_files:
        for target in MARKDOWN_LINK.findall(source.read_text()):
            target = target.strip("<>").split("#", 1)[0]
            if not target or "://" in target or target.startswith("mailto:"):
                continue
            assert (source.parent / target).resolve().exists(), f"{source.relative_to(ROOT)} -> {target}"


def test_public_guides_exclude_internal_api_names():
    forbidden = (
        "crcc._core",
        "_collides_static_batch_threads",
        "benchmark_support",
    )
    combined = "\n".join(path.read_text() for path in PUBLIC_GUIDES)
    for name in forbidden:
        assert name not in combined
