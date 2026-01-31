use commonroad_collision_checker::collision_checker::engine::parry::ParryCollisionObject;
use commonroad_collision_checker::collision_checker::parallel::ParallelCollisionChecker;
use commonroad_collision_checker::collision_checker::{CollisionCheckerBuilder, CollisionStatus};
use commonroad_collision_checker::collision_object::CollisionObject;
use commonroad_collision_checker::collision_object::simple::SimpleCollisionObject;
use commonroad_collision_checker::dynamic_obstacle::DynamicObstacle;
use commonroad_collision_checker::time::TimeStep;
use geo::Rect;
use glamx::DPose2;
use itertools::Itertools;
use rayon::prelude::*;
use std::f64::consts::FRAC_PI_2;
use std::time::Instant;

fn main() {
    let dyn_obs = DynamicObstacle::new(
        SimpleCollisionObject::circle((0.0, 0.0), 1.0).into(),
        vec![
            DPose2::translation(10.0, 10.0),
            DPose2::translation(9.0, 9.0),
            DPose2::translation(10.0, 10.0),
            DPose2::translation(0.0, 0.0),
        ],
        TimeStep(0),
    );
    let cc = CollisionCheckerBuilder::new()
        .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0))
        .with_dynamic_obstacle(dyn_obs)
        .build::<ParryCollisionObject>();

    let obj = CollisionObject::from(SimpleCollisionObject::circle((8.0, 8.0), 1.0)).into();
    let dyn_obs2 = DynamicObstacle::new(
        SimpleCollisionObject::circle((0.0, 0.0), 1.0).into(),
        vec![
            DPose2::translation(5.0, 5.0),
            DPose2::translation(15.0, -5.0),
        ],
        TimeStep(2),
    )
    .convert_repr();
    assert!(!cc.collides_static_at(&obj, TimeStep(0)).unwrap().collides());
    assert!(cc.collides_static_at(&obj, TimeStep(1)).unwrap().collides());
    assert!(!cc.collides_static_at(&obj, TimeStep(2)).unwrap().collides());
    assert!(!cc.collides_static_at(&obj, TimeStep(3)).unwrap().collides());
    assert!(!cc.collides_static_at(&obj, TimeStep(4)).unwrap().collides());
    assert_eq!(
        cc.collides_static_range(&obj, DPose2::IDENTITY, ..)
            .unwrap(),
        CollisionStatus::CollidesDynamic(TimeStep(0))
    );
    assert_eq!(
        cc.collides_static_range(&obj, DPose2::IDENTITY, TimeStep(2)..=TimeStep(4))
            .unwrap(),
        CollisionStatus::CollidesDynamic(TimeStep(2))
    );
    // dyn_obs2 has enough time to get out of the way of dyn_obs
    // this demonstrates that shape casting is more accurate than just using the convex hull
    assert_eq!(
        cc.collides_dynamic(&dyn_obs2).unwrap(),
        CollisionStatus::NoCollision
    );
    assert_ne!(
        cc.collides_static(&CollisionObject::from(SimpleCollisionObject::full_space()).into())
            .unwrap(),
        CollisionStatus::NoCollision
    );

    // Test orientation for rectangles
    let rect1 = SimpleCollisionObject::rectangle(Rect::new((0.0, 0.0), (2.0, 1.0)), 0.0);
    let rect2 = SimpleCollisionObject::rectangle(Rect::new((0.0, 1.1), (2.0, 2.1)), FRAC_PI_2);
    let cc = CollisionCheckerBuilder::new()
        .with_static_obstacle(rect1)
        .build::<ParryCollisionObject>();
    assert_ne!(
        cc.collides_static(&CollisionObject::from(rect2).into())
            .unwrap(),
        CollisionStatus::NoCollision
    );
    assert_ne!(
        cc.collides_static(&CollisionObject::from(SimpleCollisionObject::full_space()).into())
            .unwrap(),
        CollisionStatus::NoCollision
    );

    demo_parallel_collision_checks();
}

fn demo_parallel_collision_checks() {
    let cc = CollisionCheckerBuilder::new()
        .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 2.0))
        .build::<CollisionObject>();

    let now = Instant::now();
    let test_objects: Vec<_> = (0..50000)
        .into_par_iter()
        .map(|i| CollisionObject::from(SimpleCollisionObject::circle((i as f64, 0.0), 1.0)))
        .map(|co| (co, DPose2::IDENTITY))
        .collect();
    println!(
        "Creating {} test objects took {:?}",
        test_objects.len(),
        Instant::now() - now
    );
    // Clone so that LazyLock remains uninitialized for the sequential test as well
    let test_objects_seq = test_objects.clone();

    let now = Instant::now();
    let results = cc.par_collides_static(test_objects.par_iter().map(|(co, pos)| (co, *pos)), ..);
    println!("Parallel collision checks took {:?}", Instant::now() - now);
    for (i, result) in results.iter().enumerate() {
        let collides = result.as_ref().unwrap().collides();
        if i < 4 {
            assert!(collides);
        } else {
            assert!(!collides);
        }
    }
    let now = Instant::now();
    let results = cc.par_collides_static(test_objects.par_iter().map(|(co, pos)| (co, *pos)), ..);
    println!(
        "Parallel collision checks (second run) took {:?}",
        Instant::now() - now
    );

    let now = Instant::now();
    let seq_results = test_objects_seq
        .iter()
        .map(|(co, pos)| cc.collides_static_range(co, *pos, ..))
        .collect_vec();
    println!(
        "Sequential collision checks took {:?}",
        Instant::now() - now
    );
    for (i, result) in seq_results.iter().enumerate() {
        let collides = result.as_ref().unwrap().collides();
        if i < 4 {
            assert!(collides);
        } else {
            assert!(!collides);
        }
    }
    let now = Instant::now();
    let seq_results = test_objects_seq
        .iter()
        .map(|(co, pos)| cc.collides_static_range(co, *pos, ..))
        .collect_vec();
    println!(
        "Sequential collision checks (second run) took {:?}",
        Instant::now() - now
    );

    assert_eq!(results, seq_results);
}
