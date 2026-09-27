# Contributing

CRCC's repository validation is the contribution baseline. The project does not currently define additional contribution governance, a license policy, or a security-reporting process; do not infer those from the build setup.

Before proposing a change, run the checks relevant to its boundary:

```bash
uv run --frozen pre-commit run --all-files --show-diff-on-failure
uv run --frozen pyright
uv run --frozen pytest -q
cargo test --locked --all-features
uvx --from zensical==0.0.65 zensical build --strict
```

For backend changes, also test the backend feature alone and review [the extension guide](extending-backends.md). For Python or Rust API changes, update the matching [reference](../reference/python.md) or [Rust reference](../reference/rust.md) and keep examples consistent with the signatures. For changes to benchmark interpretation, preserve the generated artifacts and workload configuration; see [Benchmarking](benchmarking.md).
