use commonroad_collision_checker::collision_checker::{
    CollisionCheckerBuilder, DynamicCollisionResult,
};
use commonroad_collision_checker::collision_object::CollisionObject;
use commonroad_collision_checker::collision_object::simple::SimpleCollisionObject;
use commonroad_collision_checker::dynamic_obstacle::DynamicObstacle;
use commonroad_collision_checker::time::TimeStep;
use nalgebra::Isometry2;

fn main() {
    let dyn_obs = DynamicObstacle::new(
        SimpleCollisionObject::circle((0.0, 0.0), 1.0).into(),
        vec![
            Isometry2::translation(7.0, 7.0),
            Isometry2::translation(6.0, 6.0),
            Isometry2::translation(7.0, 7.0),
            Isometry2::translation(0.0, 0.0),
        ],
        TimeStep(0),
    );
    let cc = CollisionCheckerBuilder::new()
        .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0))
        .with_dynamic_obstacle(dyn_obs)
        .build_parry();

    let obj = CollisionObject::from(SimpleCollisionObject::circle((5.0, 5.0), 1.0)).into();
    let dyn_obs2 = DynamicObstacle::new(
        SimpleCollisionObject::circle((0.0, 0.0), 1.0).into(),
        vec![
            Isometry2::translation(-5.0, 3.0),
            Isometry2::translation(5.0, 3.0),
        ],
        TimeStep(2),
    )
    .convert_repr();
    assert!(!cc.collides_with_static(&obj).unwrap());
    assert!(!cc.collides(&obj, TimeStep(0)).unwrap());
    assert!(cc.collides(&obj, TimeStep(1)).unwrap());
    assert!(!cc.collides(&obj, TimeStep(2)).unwrap());
    assert!(!cc.collides(&obj, TimeStep(3)).unwrap());
    assert!(!cc.collides(&obj, TimeStep(4)).unwrap());
    assert_eq!(
        cc.collides_at_range(&obj, .., &Isometry2::identity())
            .unwrap(),
        DynamicCollisionResult::FirstCollisionAt(TimeStep(0))
    );
    assert_eq!(
        cc.collides_at_range(&obj, TimeStep(2)..=TimeStep(4), &Isometry2::identity())
            .unwrap(),
        DynamicCollisionResult::FirstCollisionAt(TimeStep(2))
    );
    assert_eq!(
        cc.collides_dynamic_obstacle(&dyn_obs2).unwrap(),
        DynamicCollisionResult::FirstCollisionAt(TimeStep(2))
    );
}
