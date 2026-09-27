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
```

Prepared static and dynamic queries are tied to the checker backend. Passing one to a checker using another backend raises `ValueError` in Python or returns `CrccError::Unsupported` in Rust.

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
