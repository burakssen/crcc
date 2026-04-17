use crcc::collision_checker::CollisionCheckerBuilder;
use crcc::collision_checker::engine::CollisionEngine;
use crcc::collision_object::CollisionObject;
use crcc::collision_object::simple::SimpleCollisionObject;
use glamx::DPose2;
use itertools::Itertools;
use rayon::prelude::*;
use std::time::Instant;

fn main() {
    demo_parallel_collision_checks();
}

fn demo_parallel_collision_checks() {
    let collision_checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 2.0).unwrap())
        .build_with_engine(CollisionEngine::Parry)
        .unwrap();

    let now = Instant::now();
    let test_objects: Vec<_> = (0..10_000)
        .into_par_iter()
        .map(|i| {
            CollisionObject::from(SimpleCollisionObject::circle((i as f64, 0.0), 1.0).unwrap())
        })
        .map(|collision_object| (collision_object, DPose2::IDENTITY))
        .collect();
    println!(
        "Creating {} test objects took {:?}",
        test_objects.len(),
        Instant::now() - now
    );
    // Clone so that LazyLock remains uninitialized for the sequential test as well
    let test_objects_seq = test_objects.clone();

    let now = Instant::now();
    let results = collision_checker.par_collides_static(&test_objects, ..);
    println!("Parallel collision checks took {:?}", Instant::now() - now);
    println!("Parallel collisions: {}", count_collisions(&results));
    let now = Instant::now();
    let results = collision_checker.par_collides_static(&test_objects, ..);
    println!(
        "Parallel collision checks (second run) took {:?}",
        Instant::now() - now
    );

    let now = Instant::now();
    let seq_results = test_objects_seq
        .iter()
        .map(|(collision_object, position)| {
            collision_checker.collides_static_range(collision_object, *position, ..)
        })
        .collect_vec();
    println!(
        "Sequential collision checks took {:?}",
        Instant::now() - now
    );
    println!("Sequential collisions: {}", count_collisions(&seq_results));
    let now = Instant::now();
    let seq_results = test_objects_seq
        .iter()
        .map(|(collision_object, position)| {
            collision_checker.collides_static_range(collision_object, *position, ..)
        })
        .collect_vec();
    println!(
        "Sequential collision checks (second run) took {:?}",
        Instant::now() - now
    );

    assert_eq!(results, seq_results);
}

fn count_collisions(results: &[crcc::collision_checker::CollisionResult]) -> usize {
    results
        .iter()
        .filter(|result| result.as_ref().is_ok_and(|status| status.collides()))
        .count()
}
