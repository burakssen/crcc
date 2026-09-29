# Quick start

This page checks a circle against a wall using Python, then shows the equivalent Rust query. Installation options are in [Installation](installation.md).

## First query

=== "Python"

    ```python
    from crcc import Circle, CollisionBackend, CollisionCheckerBuilder, Pose, Rectangle

    checker = (
        CollisionCheckerBuilder(backend=CollisionBackend.Parry)
        .add_static_obstacle(Rectangle(0.25, 3.0))
        .build()
    )

    hit = checker.collides_static(
        Circle(0.5),
        position=Pose.from_translation((0.5, 0.0)),
    )
    clear = checker.collides_static(
        Circle(0.5),
        position=Pose.from_translation((2.0, 0.0)),
    )

    assert hit.collides
    assert not clear.collides
    ```

=== "Rust"

    ```rust
    use crcc::{CollisionCheckerBuilder, CollisionEngine, CollisionObject, Pose};

    fn main() -> Result<(), crcc::CrccError> {
        let wall = CollisionObject::rectangle(
            geo::Rect::new((-0.125, -1.5), (0.125, 1.5)),
            0.0,
        )?;
        let query = CollisionObject::circle((0.0, 0.0), 0.5)?;
        let checker = CollisionCheckerBuilder::new()
            .with_static_obstacle(wall)
            .build_with_engine(CollisionEngine::Parry)?;

        let status = checker.collides_static_pos(&query, Pose::translation(0.5, 0.0))?;
        assert!(status.collides());
        assert!(!checker.collides_static_pos(&query, Pose::translation(2.0, 0.0))?.collides());
        Ok(())
    }
    ```

Geometry constructors are fallible in Rust. Use `?` to propagate `CrccError`; see [errors and limits](../reference/errors-and-limits.md). The checker converts scene geometry when built; for repeated queries, see [scene queries](../guides/scenes-and-trajectories.md).

In Python, read `status.collides`, **not `bool(status)` or `if status:`**. Generated enum variants do not share a collision-based truthiness rule. `status.time_step` is `None` for a static-query/static-scene hit.

The Rust example uses a dependency on `crcc` with the `parry` feature and `geo = "0.32"`, as in [Installation](installation.md). Put it in your application's `src/main.rs` and run `cargo run`. Save the Python example as a script and run it in the environment containing CRCC, for example `uv run first_check.py`.

Next: [construct geometry](../concepts/geometry-and-poses.md), [check object pairs](../guides/pair-queries.md), or [add trajectories](../guides/scenes-and-trajectories.md).
