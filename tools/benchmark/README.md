# Benchmark Tool

The benchmark pipeline is a research tool, not part of the CRCC package API. It measures engine throughput, latency, correctness, memory, scene scaling, rebuild cost, execution layers, and Rayon batch scaling.

Return to the [documentation map](../../README.md) for library usage.

## Choose a profile

- `smoke` validates workloads quickly during development.
- `spec` runs the publication-oriented benchmark matrix.

```bash
uv run main.py study --benchmark-profile smoke
uv run main.py study --benchmark-profile spec --benchmark-output benchmark_results
```

Generate a report from existing artifacts without rerunning benchmarks:

```bash
uv run main.py report --benchmark-output benchmark_results
```

## Outputs

Generated CSV rows include provenance, query and repetition counts, correctness counters, timing samples, and explicit unsupported results. Reports derive comparisons from these artifacts instead of embedding fixed conclusions.

Regular scene scaling stops at 50,000 objects. `--benchmark-include-stress` adds an isolated 100,000-object capacity workload; it does not expand the full shape, density, and dynamic matrix.
