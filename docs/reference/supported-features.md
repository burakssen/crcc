# Supported features

“Supported” means an API/path is implemented, not that all backends give identical answers or that every geometry/query pair is exact. See [backend behavior](../concepts/backends.md).

| Feature | Rust | Python | Conditions |
| --- | --- | --- | --- |
| Circle, rectangle, triangle | Yes | Yes | Validation differs for zero-size rectangles |
| Concave/holed polygon | Yes | Yes | Sanitized/validated domain input; backend decomposition |
| Compound, empty/full space, half-space | Yes | Yes | Infinite geometry has backend-specific query support |
| Discrete pair collision | Yes | Yes | At least one enabled backend |
| Separation distance | Yes | Yes | Empty unsupported; runtime Rhusics/Collide use shared fallback |
| Continuous pair collision | Yes | Yes | Conservative positives; no returned time of impact |
| Immutable scene checker | Yes | Yes | Rust typed/runtime; Python runtime only |
| Fixed-shape trajectory | Yes | Yes | Consecutive pose samples, signed 32-bit time |
| Time-varying geometry | Yes | Yes | Swept-union checking, not shape morphing |
| Query time windows | Yes | Yes | Rust ranges / Python inclusive bounds; static/dynamic interval policies differ |
| Prepared static/dynamic queries | Yes | Yes | Backend-specific, reusable across same-backend scenes |
| Ordered sequential/parallel batches | `rayon` feature | Yes | Python packaging enables Rayon; errors differ by language |
| Road-boundary geometry | Builder method | Coordinate-ring helper and adapter | Approximate simplified lanelet complement |
| CommonRoad shapes/occupancies/predictions | No model adapter | Yes | `commonroad-io>=2026.1`; Python adapter |
| CommonRoad XML loading | No | External reader | `CommonRoadFileReader`, not a CRCC parser |
| Geometry serialization | No | No | No supported persistence format |
| Scene updates after build | No | No | Rebuild the checker |
| Contact manifolds/impulses/simulation | No | No | Collision checking only |
| Runtime plugin backend loading | No | No | Backends integrated and feature-gated in source |

All three backends are implemented and enabled by default. No public “experimental” feature flag separates them. Rotational/time-varying CCD and moving half-space handling have explicitly conservative limitations; benchmark-only exports are internal research instrumentation, not stable application APIs. No roadmap is implied by an unsupported row.
