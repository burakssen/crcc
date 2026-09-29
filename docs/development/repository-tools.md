# Examples and playground

Run these from the repository root after [source setup](building-and-testing.md). `main.py` is a checkout utility, not an installed console command.

```bash
uv run --frozen main.py basic --engine parry
uv run --frozen main.py continuous --engine rhusics
uv run --frozen main.py commonroad --scenario scenarios/ZAM_Tutorial-1_2_T-1.xml
uv run --frozen main.py playground --scenario scenarios/ZAM_Tutorial-1_2_T-1.xml
```

| Action | What it runs |
| --- | --- |
| `basic` | Geometry, poses, compounds, half-spaces, pair/scene results |
| `continuous` | Endpoint versus interval queries, rotation, dynamic interval starts |
| `commonroad` | File loading, conversion, scene query probes |
| `all` | The three tutorials; not benchmarks or playground |
| `playground` | Interactive Matplotlib GUI |
| `study` / `report` | Benchmark measurement / existing-artifact reporting |

Legacy CLI aliases: `concepts`/`shapes` select `basic`, `dynamics` selects `continuous`, and `scenario` selects `commonroad`. The CLI's default backend is **Rhusics**, unlike the installed API default (Parry when enabled). `--engine`/`--scenario` control tutorials; benchmark selectors are separate `--benchmark-*` options.

The continuous tutorial's dynamic rows select start steps and may include outgoing intervals. Its “clear” result follows the selected backend's contact/motion convention. Tutorial presentation catches query exceptions for display; use direct assertions/exception handling, rather than display labels, to validate correctness.

The CommonRoad tutorial's probe is the average of lanelet vertices, not a guaranteed interior point/polygon centroid. Its time helper gathers initial and listed trajectory-state times, not all set-based occupancy times. These conveniences do not define adapter semantics.

## Interactive viewer

The playground requires a display and an interactive Matplotlib backend. It supports geometry presets/drawing, query/environment roles, backend selection, and timeline play/step. Coloring distinguishes current occupancy from conservative interval possibilities. This is not a web UI or simulator.

For headless checks, use `tests/test_examples.py`; those tests exercise viewer logic without a GUI. For scripted package use, prefer the [guides](../guides/pair-queries.md).
