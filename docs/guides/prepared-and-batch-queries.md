# Prepared and batch queries

Scene checkers convert query geometry for their backend. Prepare geometry when the same query is reused; batch independent queries when you have many results to collect. Batches preserve input order.

## Reuse prepared geometry

```python
from crcc import Circle, CollisionCheckerBuilder, Pose

checker = CollisionCheckerBuilder().add_static_obstacle(Circle(1.0)).build()
query = Circle(0.25)
prepared = checker.prepare_static(query)

results = [
    checker.collides_static(prepared, Pose.identity()),
    checker.collides_static(prepared, Pose.from_translation((5.0, 0.0))),
]
assert [result.collides for result in results] == [True, False]
another_scene = CollisionCheckerBuilder(backend=checker.backend).add_static_obstacle(Circle(2.0)).build()
assert another_scene.collides_static(prepared).collides
```

Prepared static and dynamic queries are tied to a **backend**, not one scene. Reuse them with another checker using the same backend. Passing one to a different backend raises `ValueError` in Python or returns `CrccError::Unsupported` in Rust. Preparation caches conversion; it does not certify support for every subsequent operation.

## Run a Python batch

```python
from crcc import Circle, CollisionCheckerBuilder, Pose, Rectangle

checker = CollisionCheckerBuilder().add_static_obstacle(Rectangle(2.0, 2.0)).build()
results = checker.collides_static_batch(
    [
        (Circle(0.25), Pose.identity()),
        (Circle(0.25), Pose.from_translation((5.0, 0.0))),
    ],
    parallel=True,
)
assert [result.collides for result in results] == [True, False]
```

Python batch methods accept `parallel=False` by default; set it to `True` to request Rayon execution. Use `collides_dynamic_batch` for dynamic queries. Batch entries can mix raw objects with prepared queries. The GIL is released while native batch work runs.

Static entries are `(query, Pose)` pairs; a batch pose cannot be `None`. Dynamic entries are trajectories or prepared dynamic queries. Each entry is checked independently against the scene, not against other batch entries. A Python batch returns a complete list or raises if any entry fails; there is no partial result list. Empty inputs return `[]`, but invalid time bounds still raise.

```python
from crcc import Circle, CollisionCheckerBuilder, DynamicObstacle, Pose

checker = CollisionCheckerBuilder().add_static_obstacle(Circle(1.0)).build()
hit = DynamicObstacle(Circle(0.25), [Pose.identity()], time_offset=0)
clear = DynamicObstacle(Circle(0.25), [Pose.from_translation((5.0, 0.0))], time_offset=0)
queries = [checker.prepare_dynamic(hit), clear]
sequential = checker.collides_dynamic_batch(queries)
parallel = checker.collides_dynamic_batch(queries, parallel=True)
assert [(r.collides, r.time_step) for r in sequential] == [(True, 0), (False, None)]
assert [(r.collides, r.time_step) for r in sequential] == [(r.collides, r.time_step) for r in parallel]
```

Rayon uses its active/global pool, not a fresh pool for every ordinary batch. There is no public per-checker thread-count control or automatic batch-size threshold. Parallel overhead can outweigh savings for small batches.

## Run a Rust batch

Enable the `rayon` feature. Rust makes execution choice explicit with the final `parallel` argument:

```rust
use crcc::{CollisionCheckerBuilder, CollisionEngine, CollisionObject, Pose};

fn main() -> Result<(), crcc::CrccError> {
    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(CollisionObject::rectangle(
            geo::Rect::new((-1.0, -1.0), (1.0, 1.0)),
            0.0,
        )?)
        .build_with_engine(CollisionEngine::Parry)?;
    let query = CollisionObject::circle((0.0, 0.0), 0.25)?;
    let positioned = [
        (query.clone(), Pose::IDENTITY),
        (query, Pose::translation(5.0, 0.0)),
    ];

    let results = checker.collides_static_batch(&positioned, .., true);
    assert!(results[0].as_ref().is_ok_and(|status| status.collides()));
    assert!(results[1].as_ref().is_ok_and(|status| !status.collides()));
    Ok(())
}
```

Rust batch calls return one `CollisionResult` per input, including per-query errors. Ordering is retained for sequential and Rayon execution. See [Cargo features](../reference/rust.md#cargo-features) and the [benchmark guide](../development/benchmarking.md) for measurement tools.

Rust heterogeneous batches accept `StaticBatchQuery::{Raw, Prepared}` or `DynamicBatchQuery::{Raw, Prepared}`. A wrong-backend prepared entry fails preflight for the whole heterogeneous batch, producing `Unsupported` in every slot. The homogeneous prepared dynamic batch instead reports mismatches per slot. Both sequential and parallel Rust batch methods require the `rayon` feature.
