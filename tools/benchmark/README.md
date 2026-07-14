# Benchmark tool

The benchmark pipeline is a research tool, not part of the CRCC package API. Its schema-7 artifacts compare engines across shape families, scene modes, discrete/continuous detection, scene sizes, collision densities, rebuild costs, execution layers, and Rayon batch scaling.

Use the smoke profile to validate workloads and the spec profile for publication runs:

```bash
uv run main.py study --benchmark-profile smoke
uv run main.py study --benchmark-profile spec --benchmark-output benchmark_results
uv run main.py report --benchmark-output benchmark_results
```

Regular scene scaling stops at 50,000 objects. `--benchmark-include-stress` adds only the isolated 100,000-object capacity workload; it does not expand the full shape/density/dynamic matrix.

Generated CSV rows carry provenance, query and repetition counts, correctness counters, and explicit unsupported results. Reports derive comparisons from those artifacts rather than embedding fixed performance conclusions.
