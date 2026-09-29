# Benchmarking

CRCC's benchmark tools are repository research utilities, not installed package APIs. The Python pipeline measures throughput, latency, correctness, memory, scene scaling, reconstruction, execution layers, and Rayon batches.

## Run a bounded smoke study

Limit suites and samples when checking the harness:

```bash
uv run --frozen main.py study \
  --benchmark-profile smoke \
  --benchmark-suite pair continuous distance \
  --benchmark-samples 100 \
  --benchmark-repetitions 2 \
  --benchmark-output target/crcc-python-bench
```

Generate report and plots from existing artifacts:

```bash
uv run --frozen main.py report --benchmark-output target/crcc-python-bench
```

The `smoke` profile checks harness execution, not publication-quality measurements. `spec` expands workload/scaling dimensions but does not itself raise the CLI's sample count. Bare `study` selects all 17 suites, 20,000 samples, five repetitions and seed 2026. Use explicit bounds and a fresh output directory to avoid mixing old artifacts.

For timing comparisons, first rebuild the extension with release optimization:

```bash
uv run --frozen --with "maturin>=1.0,<2.0" maturin develop --release
```

Otherwise Python measurements can time a debug extension while native subprocesses use release binaries. Record the profile, revision, compiler, hardware and thread settings with results.

Select suites, engines, scenarios, and thread counts explicitly when investigating one workload:

```bash
uv run --frozen main.py study \
  --benchmark-profile smoke \
  --benchmark-suite scenario parallel \
  --benchmark-engines parry rhusics \
  --benchmark-scenarios scenarios/ZAM_Tutorial-1_2_T-1.xml \
  --benchmark-thread-counts 1 2 4 \
  --benchmark-samples 100 \
  --benchmark-repetitions 2 \
  --benchmark-output target/crcc-python-bench
```

See [Benchmark artifacts and workloads](../reference/benchmark-artifacts.md) for all suite names, schema, binary arguments and datasets. `report` only loads/plots existing artifacts. `study --benchmark-step run` measures without generating plots, but can already write Markdown links to plots that do not yet exist.

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
- Conservative CCD false-positive acceptance is conditional on the workload's `ccd_mode`; the current synthetic continuous/coverage modes do not all declare that policy. Inspect `correctness.csv` rather than assuming every conservative positive is exempt.
- `update_proxy` changes query poses, not scene state; `rebuild_update` measures immutable-checker reconstruction.
- Memory measurements include Python wrappers, allocator retention, and page granularity.
- Scenario paths are repository-relative and their XML data must be materialized from Git LFS.

Preserve benchmark artifacts, workload configuration, and provenance with any reported result. Do not turn one run into a general backend ranking; consult [backend trade-offs](../concepts/backends.md).

## Current reporting limitations

These are implementation limitations of the harness, not performance properties of CRCC:

- A successful run/artifact write is not a correctness acceptance gate. Timing comparisons can include workloads with independently recorded correctness mismatches.
- Planning repetitions each measure multiple frames, but reported deadline-miss rates use repetition counts as the denominator. Do not interpret that field or its plotted percentage as a validated per-frame rate.
- The canonical workload hash is a partial contract fingerprint, not a complete serialization of executed inputs or scenario contents. Some canonical shape labels differ from executed geometry.
- Native build/process failures can omit suite rows. RSS probe failures can appear as zero-byte measurements. Missing data does not prove unsupported capability or zero memory use.
- The native harness assumes POSIX-style process/RSS tools and `target/release` executable paths. Library wheel support is broader than benchmark tooling support.
- Per-suite fallback loading checks schema, not equality of all run configurations. Do not merge directories with different seeds, revisions or profiles merely because they load.

Before publishing measurements, independently inspect workload identity, correctness rows, errors, missing suites and sample semantics. Confidence intervals summarize this harness's repetitions, not universal backend rankings.
