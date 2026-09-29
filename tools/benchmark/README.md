# Benchmark tool

This pipeline is repository research tooling, not an installed CRCC application API.

Authoritative documentation:

- [Setup, bounded runs, interpretation and current limitations](../../docs/development/benchmarking.md)
- [Suite inventory, artifact schema, native binary contracts and datasets](../../docs/reference/benchmark-artifacts.md)
- [Backend behavior](../../docs/concepts/backends.md)

From the repository root, after installing the extension:

```bash
uv run --frozen main.py study \
  --benchmark-profile smoke \
  --benchmark-suite pair continuous distance \
  --benchmark-samples 100 \
  --benchmark-repetitions 2 \
  --benchmark-step run \
  --benchmark-output target/crcc-python-bench

uv run --frozen main.py report --benchmark-output target/crcc-python-bench
```

Use a fresh output directory. Bare `study` selects all 17 suites with 20,000 samples and five repetitions; `smoke` alone does not bound total work. Build with `maturin develop --release` before timing comparisons with release Rust binaries.

`report` loads existing artifacts without rerunning measurements. A successful run or generated chart does not establish correctness. Conservative CCD policy is workload-dependent; planning deadline rates, partial workload hashes, missing native data and failed RSS measurements have documented interpretation limits. Keep input configuration and artifacts with any reported result.
