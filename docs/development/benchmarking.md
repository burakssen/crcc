# Benchmarking

CRCC's benchmark tools are repository research utilities, not installed package APIs. The Python pipeline measures throughput, latency, correctness, memory, scene scaling, reconstruction, execution layers, and Rayon batches.

## Run a bounded smoke study

Limit suites and samples when checking the harness:

```bash
uv run main.py study \
  --benchmark-profile smoke \
  --benchmark-suite pair continuous distance \
  --benchmark-samples 100 \
  --benchmark-repetitions 2 \
  --benchmark-output target/crcc-python-bench
```

Generate report and plots from existing artifacts:

```bash
uv run main.py report --benchmark-output target/crcc-python-bench
```

The `smoke` profile checks harness execution; it is not a basis for publication claims. The `spec` profile uses larger measurement and scaling ranges. Available suites include pair, continuous, distance, shape complexity, scene scaling, updates, dynamic batches, time-varying shapes, native layers, parallel scaling, density, dynamic scenes, planning, and CommonRoad scenarios.

Select suites, engines, scenarios, and thread counts explicitly when investigating one workload:

```bash
uv run main.py study \
  --benchmark-profile smoke \
  --benchmark-suite scenario parallel \
  --benchmark-engines parry rhusics \
  --benchmark-scenarios scenarios/ZAM_Tutorial-1_2_T-1.xml \
  --benchmark-thread-counts 1 2 4 \
  --benchmark-samples 100 \
  --benchmark-repetitions 2 \
  --benchmark-output target/crcc-python-bench
```

See [`tools/benchmark/README.md`](https://github.com/burakssen/crcc/blob/main/tools/benchmark/README.md) for all suite names, profiles, artifacts, and binary arguments.

## Native and Rust batch binaries

Compare converted backend objects with public Rust calls:

```bash
cargo run --release --locked \
  --bin native_benchmark \
  --features benchmarking,parry,rhusics,collide \
  -- parry native circle_clear 100000
```

Compare scalar and Rayon-backed batches:

```bash
cargo run --release --locked \
  --bin parallel_benchmark \
  --features rayon,parry,rhusics,collide \
  -- parry static 1024 4 10
```

Both binaries emit CSV. The parallel binary checks scalar/batch result equality before timing.

## Interpretation limits

- Results depend on hardware, toolchain, allocator, process order, and thermal state.
- Confidence intervals summarize repetitions in this harness, not hardware-independent performance.
- Conservative CCD false positives are permitted; false negatives and query errors are correctness failures.
- `update_proxy` changes query poses, not scene state; `rebuild_update` measures immutable-checker reconstruction.
- Memory measurements include Python wrappers, allocator retention, and page granularity.
- Scenario paths are repository-relative and their XML data must be materialized from Git LFS.

Preserve benchmark artifacts, workload configuration, and provenance with any reported result. Do not turn one run into a general backend ranking; consult [backend trade-offs](../concepts/backends.md).
