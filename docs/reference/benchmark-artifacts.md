# Benchmark artifacts and workloads

The harness is repository tooling. Setup and bounded commands are in [Benchmarking](../development/benchmarking.md); implementation lives in [`tools/benchmark/`](https://github.com/burakssen/crcc/tree/main/tools/benchmark).

## Suite inventory

Use `--benchmark-suite` followed by one or more names (or `all`):

| Suite | Measures |
| --- | --- |
| `pair` | Direct discrete object pairs |
| `continuous` | Endpoint-motion pair queries |
| `distance` | Separation distance |
| `shape_complexity` | Vertex/component complexity |
| `coverage_matrix` | Shape/operation support and expected results |
| `scene_scaling` | Immutable scene-size scaling |
| `update_proxy` | Changing query poses, not scene updates |
| `rebuild_update` | Complete checker reconstruction |
| `api_overhead` | Python/public/native costs and pool modes |
| `dynamic_batch` | Trajectory batch amortization |
| `time_variant` | Time-varying query scaling |
| `native_layers` | Converted/native versus public Rust paths |
| `parallel` | Scalar/reusable Rayon batch comparison |
| `density_scaling` | Spatial density at a profile-selected object count |
| `dynamic_scene` | Scene obstacle/time dimensions |
| `planning` | Candidate trajectory batches, map/prediction and cache modes |
| `scenario` | Converted CommonRoad scenes and scalar/batch equivalence |

`smoke` and `spec` choose dimensions; `--benchmark-samples`, `--benchmark-repetitions`, `--benchmark-seed`, `--benchmark-engines`, `--benchmark-scenarios`, and `--benchmark-thread-counts` are independent controls. Counts above detected CPU capacity are dropped (capacity is used if none remain). Regular scene scaling stops at 50,000 objects; `--benchmark-include-stress` adds isolated 100,000-object checks, not a full expanded matrix.

## Artifact bundle

Current schema version is **`12`**. Output defaults to `target/crcc-python-bench` and contains:

| File | Meaning |
| --- | --- |
| `metadata.json` | Run configuration, environment/revision and partial workload contract hash |
| `runs.csv` | Per-repetition measurements and sample quantiles |
| `summary.csv` | Aggregation by workload/backend/mode |
| `comparisons.csv` | Backend comparisons using Parry baseline |
| `mode_comparisons.csv` | API/batch execution-mode ratios |
| `layer_comparisons.csv` | Native/public/Python ratios |
| `correctness.csv` | Oracle/equivalence counts, mismatches and errors |
| `parallel_scaling.csv` | Thread scaling, relative to one-thread reusable batch |
| `memory.csv` | Heap/RSS measurements with measurement labels |
| `benchmark_report.md` | Narrative report and plot links |
| `suites/<suite>/` | Per-suite artifacts |
| `plots/` | 24 named plots, each PNG and PDF, after plotting |

The loader prefers aggregate CSVs, rejects partial aggregate bundles, and requires every CSV type even when a category only has headers. If no aggregate CSV exists, it loads per-suite bundles. It validates schema/required columns and requires summary rows. Compatible schema alone does not establish identical seeds/revisions/configuration.

### Units and interpretation

`*_ns` are nanoseconds; throughput is queries/second; memory `*_bytes` is bytes; times/trajectory lengths are sample counts. Blank optional values mean inapplicable or unavailable fields, not zero; some failed RSS measurements are unfortunately stored as zero. Counts such as collisions/errors describe the workload, not a success probability.

Read `sample_semantics` before comparing quantiles:

- `per_query`: individual operation samples.
- `call_average`: amortized work in an API call.
- `batch_average`: amortized batch timing, not individual tail latency.
- `per_frame`: complete planning-frame timing.

Pool construction can be included for fresh-pool modes and excluded for reusable-pool modes. `build_ns`, `construction_ns`, and `query_ns` distinguish setup/query paths where recorded. Planning deadline fields currently have [reporting limitations](../development/benchmarking.md#current-reporting-limitations).

## Correctness evidence

Analytic workloads, backend-specific contact expectations, and scalar/batch equivalence serve different purposes. Scenario equivalence is not an independent geometry oracle. Conservative false positives are excluded from mismatch counts only when `ccd_mode` begins with `conservative`; other modes retain them. Inspect false negatives, errors and support status before reading speedups. Native process failure can omit rows rather than producing an explicit unsupported record.

## Native binary contracts

```text
native_benchmark <parry|rhusics|collide> <native|public> <workload> [iterations]
```

Workloads: `circle_clear`, `circle_hit`, `rectangle_clear`, `rectangle_hit`, `compound_clear`, `ccd`, `tunneling`, `moving_vs_moving`, `rotation_wrap`, `endpoint_touch`, `distance`, `dynamic_fixed`, `dynamic_time_variant`. Default iterations: 1,000,000; warm-up: 10,000. CSV includes layer, backend, operation, workload, iterations, nanoseconds, checksum, trajectory steps, motion kind and shape variation. Invalid arguments/query errors write stderr and exit nonzero.

Rhusics/Collide “native distance” uses the shared public fallback; it is not a pure backend-native kernel.

```text
parallel_benchmark <parry|rhusics|collide> <static|dynamic> <batch-size> <threads> <iterations>
```

Numeric arguments must be positive. It checks full scalar/batch result equality before timing and emits scalar and `batch_reusable` rows. Pool creation is outside timed loops. Dynamic queries move to the origin; the metadata's nominal `0.5` density is not an independent measured half-hit ratio.

Rust benchmark helpers are behind `benchmarking` and hidden from ordinary Rustdoc. `crcc._core.benchmark.collides_static_batch_fresh_pool(checker, positioned_query_shapes, threads, min_time=None, max_time=None)` is internal Python instrumentation: it constructs a new pool, coerces zero threads to one, and can raise `RuntimeError` for pool construction or `ValueError` for queries. Ordinary batch calls do not use this helper.

## CommonRoad datasets

The 18 files in `scenarios/` are Git-LFS XML data with `2020a` headers; the adapter uses installed `commonroad-io>=2026.1` to read them. Materialize them with `git lfs pull`. Dataset discovery is sorted/nonrecursive and depends on running from the repository root.

| Filename | Header time-step size | Header source |
| --- | --- | --- |
| `ARG_Carcarana-10_1_T-1.xml` | 0.1 | Scenario Factory / OSM / SUMO |
| `AUS_Brisbanecentralbusinessdistrict-10_1_T-1.xml` | 0.1 | Scenario Factory 2.0 / OTS |
| `BEL_Spa-1_2_T-1.xml` | 0.15 | TheRacingHub / Assetto Corsa Competizione |
| `BRA_BeloHorizonte-40_1_T-39.xml` | 0.1 | Scenario Factory 2.0 / OTS |
| `C-DEU_B471-1_1_T-1.xml` | 0.1 | Bing Maps |
| `DEU_Aschaffenburg-14_20_T-1.xml` | 0.1 | OSM / SUMO |
| `DEU_Guetersloh-54_1_T-1.xml` | 0.1 | Scenario Factory / OSM / SUMO |
| `DEU_MerzenichRather-2_870_T-149.xml` | 0.04 | exiD |
| `DEU_Muc-2_1_T-1.xml` | 0.1 | Bing Maps |
| `RUS_Bicycle-3_1_T-1.xml`, `RUS_Bicycle-4_1_T-1.xml` | 0.1 | Technische Hochschule Ingolstadt |
| `USA_Peach-1_1_T-1.xml`, `USA_US101-10_1_T-1.xml`, `USA_US101-6_1_T-1.xml` | 0.1 | NGSIM / OSM |
| `ZAM_Merge-1_1_T-1.xml`, `ZAM_Yield-1_1_T-1.xml` | 0.1 | CommonRoad Scenario Designer |
| `ZAM_Tutorial-1_2_T-1.xml` | 0.1 | Empty source field |
| `ZAM_Urban-4_1_S-1.xml` | 0.25 | BMW CAR@TUM |

The tutorial filename contains `1_2` while its header benchmark ID contains `1_1`. Record filenames and LFS content identities, not just that ID. `git lfs ls-files --long` lists content identifiers for reproducibility. Header attribution is not a redistribution-license statement.

Scenario probes uniformly sample translations/angles within lanelet bounds padded by 5 coordinate units and use a 4.5 × 2 rectangle. They do not constitute complete scenario safety verification. Keep scenario hashes, input configuration, compiler/build profile and artifacts with reported measurements.
