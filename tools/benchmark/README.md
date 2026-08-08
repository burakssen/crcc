# Benchmark Tool

The benchmark pipeline is a repository research tool, not part of the installed CRCC package API. It measures throughput, latency, correctness, memory, scene scaling, reconstruction cost, execution layers, and Rayon batch behavior.

Return to [Development and benchmarks](../../docs/development.md) for project setup and standard checks.

## Start With a Bounded Smoke Run

The default `smoke` profile still selects every suite and uses 20,000 samples. For a quick harness check, restrict the work explicitly:

```bash
uv run main.py study \
  --benchmark-profile smoke \
  --benchmark-suite pair continuous distance \
  --benchmark-samples 100 \
  --benchmark-repetitions 2 \
  --benchmark-output target/crcc-python-bench
```

Generate plots and the Markdown report from existing artifacts:

```bash
uv run main.py report --benchmark-output target/crcc-python-bench
```

## Profiles

- `smoke`: development-oriented configuration for validating workloads and reporting.
- `spec`: publication-oriented matrix with larger sample and scaling ranges.

```bash
uv run main.py study \
  --benchmark-profile spec \
  --benchmark-output benchmark_results/spec
```

Add isolated capacity workloads only when needed:

```bash
uv run main.py study \
  --benchmark-profile spec \
  --benchmark-include-stress \
  --benchmark-output benchmark_results/spec-stress
```

Regular scene scaling stops at 50,000 objects. Stress mode adds isolated 100,000-object checks rather than expanding every matrix.

## Suites

```text
pair                  Direct discrete pair queries
continuous            Continuous pair motion
distance              Separation distance
shape_complexity      Geometry complexity scaling
coverage_matrix       Shape/operation support
scene_scaling         Immutable scene-size scaling
update_proxy          Changed query-pose proxy
rebuild_update        Full checker reconstruction
api_overhead          Python/public/native layer costs
dynamic_batch         Dynamic batch amortization
time_variant          Time-varying query scaling
native_layers         Native versus public Rust layers
parallel              Reusable scalar/batch Rayon comparison
density_scaling       Spatial-density workloads
dynamic_scene         Dynamic scene-size/time behavior
scenario              CommonRoad scenario workloads
```

Select suites with repeated values after `--benchmark-suite`, or use `all`.

## Narrow a Run

```bash
uv run main.py study \
  --benchmark-profile smoke \
  --benchmark-suite scenario parallel \
  --benchmark-engines parry rhusics \
  --benchmark-scenarios scenarios/ZAM_Tutorial-1_2_T-1.xml \
  --benchmark-thread-counts 1 2 4 \
  --benchmark-samples 1000 \
  --benchmark-repetitions 3 \
  --benchmark-seed 2026 \
  --benchmark-output target/crcc-python-bench
```

Requested thread counts above detected CPU capacity are dropped. If every requested count is too high, the detected CPU count is used.

## Separate Measurement and Reporting

```bash
uv run main.py study \
  --benchmark-step run \
  --benchmark-output target/crcc-python-bench

uv run main.py report \
  --benchmark-output target/crcc-python-bench
```

`report` always loads artifacts and plots them; it does not rerun measurements.

## Artifacts

Aggregate output contains:

```text
metadata.json
runs.csv
summary.csv
comparisons.csv
mode_comparisons.csv
layer_comparisons.csv
correctness.csv
parallel_scaling.csv
memory.csv
benchmark_report.md
plots/*.png
plots/*.pdf
suites/<suite>/...
```

All aggregate CSV types are required when loading results. Artifacts carry a schema version; mixed or obsolete schemas are rejected instead of silently merged.

## Native/Public Layer Binary

```bash
cargo run --release --locked \
  --bin native_benchmark \
  --features benchmarking,parry,rhusics,collide \
  -- parry native circle_clear 100000
```

Syntax:

```text
native_benchmark <engine> <native|public> <workload> [iterations]
```

Workloads:

```text
circle_clear circle_hit rectangle_clear rectangle_hit compound_clear
ccd tunneling moving_vs_moving rotation_wrap endpoint_touch distance
dynamic_fixed dynamic_time_variant
```

Output is CSV with execution layer, backend, operation, workload, timing, checksum, and trajectory metadata.

## Reusable Batch Binary

```bash
cargo run --release --locked \
  --bin parallel_benchmark \
  --features rayon,parry,rhusics,collide \
  -- parry static 1024 4 10
```

Syntax:

```text
parallel_benchmark <engine> <static|dynamic> <batch-size> <threads> <iterations>
```

The binary verifies scalar/batch result equality before recording timing and emits one CSV row per mode.

## Correctness Contract

- Conservative CCD false positives are permitted and recorded.
- False negatives are correctness failures.
- Query errors are not converted into clear results.
- Batch and scalar output must agree before parallel timing is accepted.
- Unsupported shape/operation combinations are explicit artifact rows, not missing data.

## Interpretation Limits

- Results are machine-, toolchain-, allocator-, and thermal-state dependent.
- Confidence intervals describe this harness's repetitions, not universal performance.
- RSS includes Python wrappers, allocator retention, and page granularity.
- Static scene scaling uses an immutable merged scene, not a mutable broad-phase world.
- `update_proxy` changes query poses; it does not mutate scene storage.
- `rebuild_update` measures complete checker reconstruction.
- Native subprocess calls have finite timeouts and can fail independently.
- Run from the repository root because scenarios and native binaries use repository-relative paths.

Do not commit fixed performance conclusions without the corresponding artifacts, provenance, and workload configuration.
