use commonroad_collision_checker::collision_checker::{
    CollisionCheckerBuilder, DynamicCollisionResult,
};
use commonroad_collision_checker::collision_object::CollisionObject;
use commonroad_collision_checker::collision_object::simple::SimpleCollisionObject;
use commonroad_collision_checker::dynamic_obstacle::DynamicObstacle;
use commonroad_collision_checker::time::TimeStep;
use geo::Rect;
use glamx::DPose2;
use std::f64::consts::FRAC_PI_2;

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
        .build_parry();

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
    assert!(!cc.collides_static_at(&obj, TimeStep(0)).unwrap());
    assert!(cc.collides_static_at(&obj, TimeStep(1)).unwrap());
    assert!(!cc.collides_static_at(&obj, TimeStep(2)).unwrap());
    assert!(!cc.collides_static_at(&obj, TimeStep(3)).unwrap());
    assert!(!cc.collides_static_at(&obj, TimeStep(4)).unwrap());
    assert_eq!(
        cc.collides_static_range(&obj, &DPose2::identity(), ..)
            .unwrap(),
        DynamicCollisionResult::FirstCollisionAt(TimeStep(0))
    );
    assert_eq!(
        cc.collides_static_range(&obj, &DPose2::identity(), TimeStep(2)..=TimeStep(4))
            .unwrap(),
        DynamicCollisionResult::FirstCollisionAt(TimeStep(2))
    );
    // dyn_obs2 has enough time to get out of the way of dyn_obs
    // this demonstrates that shape casting is more accurate than just using the convex hull
    assert_eq!(
        cc.collides_dynamic(&dyn_obs2).unwrap(),
        DynamicCollisionResult::NoCollision
    );
    assert_ne!(
        cc.collides_static(&CollisionObject::from(SimpleCollisionObject::full_space()).into())
            .unwrap(),
        DynamicCollisionResult::NoCollision
    );

    // Test orientation for rectangles
    let rect1 = SimpleCollisionObject::rectangle(Rect::new((0.0, 0.0), (2.0, 1.0)), 0.0);
    let rect2 = SimpleCollisionObject::rectangle(Rect::new((0.0, 1.1), (2.0, 2.1)), FRAC_PI_2);
    let cc = CollisionCheckerBuilder::new()
        .with_static_obstacle(rect1)
        .build_parry();
    assert_ne!(
        cc.collides_static(&CollisionObject::from(rect2).into())
            .unwrap(),
        DynamicCollisionResult::NoCollision
    );
    assert_ne!(
        cc.collides_static(&CollisionObject::from(SimpleCollisionObject::full_space()).into())
            .unwrap(),
        DynamicCollisionResult::NoCollision
    );
}
