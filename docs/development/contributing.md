# Contributing

Use the repository's validation and surrounding code patterns as the contribution baseline. Describe the intended API/behavior change, its supported feature configurations, and relevant tests in a contribution.

Before proposing a change, run the checks relevant to its boundary from [Building and testing](building-and-testing.md). Rebuild the Python extension after Rust changes. Hooks can modify files; review their diff before rerunning.

For backend changes, also test the backend feature alone and review [the extension guide](extending-backends.md). For Python or Rust API changes, update the matching [reference](../reference/python.md) or [Rust reference](../reference/rust.md) and keep examples consistent with the signatures. For changes to benchmark interpretation, preserve the generated artifacts and workload configuration; see [Benchmarking](benchmarking.md).

## Documentation maintenance

- Put constructor invariants/errors in Rustdoc/native docstrings; put workflows and cross-backend explanations in guides.
- Keep supported import paths, `.pyi` files, public exports and binding signatures aligned. Compatibility names are not a promise of a removal date.
- Make runnable `python`/`rust` fences self-contained with assertions. Use `text` for signatures, incomplete fragments, and pseudocode. The checker executes actual fences, including tab indentation, rather than a separate copy.
- Write mathematical formulas with GitHub's inline syntax, for example $`-\pi/\pi`$. The site's MathJax integration renders this notation; API names and executable expressions remain code spans.
- Keep one authoritative explanation per contract and link to it. Use relative `.md` links within the site and absolute GitHub links for repository files outside `docs/`.
- Add pages to `zensical.toml`, build strictly, and inspect diagrams/navigation. Rustdoc is generated separately and copied after site generation. There is no `mkdocs-gen-files` pipeline.
- When changing API examples, run `tests/test_documentation.py` and `tools/check_documentation.py` against the rebuilt extension.

## Releases and compatibility

GitHub tag builds verify that `vX.Y.Z` matches both manifests, build an sdist, build wheels from it, and attach release artifacts. Routine checks smoke-test wheel queries. Before publishing a new API, synchronize manifests/lockfiles, release notes, install examples and documentation. These are contributor checks, not a newly promised semantic-versioning policy.

For geometry/query work see [Extending geometry and queries](extending-geometry.md); for binding work see [Python bindings](python-bindings.md).
