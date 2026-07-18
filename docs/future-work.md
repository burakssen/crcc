# Broad-Phase Acceleration

This note records a possible optimization for large dynamic scenes. It is not part of the current implementation roadmap.

## Current behavior

CRCC uses swept axis-aligned bounds to reject object pairs before continuous narrow-phase checks. The rejection is cheap, but candidate generation can still consider every pair and therefore grows quadratically with the number of dynamic objects.

This is acceptable for small scenes. Large sparse scenes may benefit from a spatial broad phase that avoids generating obviously unrelated pairs.

## Candidate approach

A uniform spatial hash is the simplest initial candidate:

1. Compute each object's swept bounds for the current interval.
2. Insert its identifier into every overlapping grid cell.
3. Generate object pairs that share at least one cell.
4. Deduplicate the pairs.
5. Run the existing swept-bounds and narrow-phase checks.

Sweep and prune or a dynamic AABB tree remain alternatives. The implementation should be selected from representative measurements rather than theoretical complexity alone.

## Correctness constraints

- Index swept bounds, not only endpoint bounds, so between-step collisions cannot be missed.
- Deduplicate pairs that share multiple cells.
- Handle objects spanning many cells without unbounded temporary allocation.
- Preserve deterministic result ordering where the public batch API promises input order.
- Keep the existing narrow phase as the final authority.

## Evaluation criteria

An implementation should be considered only when benchmarks demonstrate:

- Lower candidate counts and query time for large sparse scenes.
- No regression for small or dense scenes.
- Stable memory use across repeated time steps.
- Identical collision results across every supported engine.

The existing [benchmark tool](../tools/benchmark/README.md) should provide the measurement harness before a broad-phase design is chosen.
