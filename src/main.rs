use commonroad_collision_checker::collision_checker::CollisionCheckerBuilder;
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
        ],
        TimeStep(0),
    );
    let cc = CollisionCheckerBuilder::new()
        .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0))
        .with_dynamic_obstacle(dyn_obs)
        .build_parry();
    let obj = CollisionObject::from(SimpleCollisionObject::circle((5.0, 5.0), 1.0)).into();
    assert!(!cc.collides_with_static(&obj).unwrap());
    assert!(!cc.collides(&obj, TimeStep(0)).unwrap());
    assert!(cc.collides(&obj, TimeStep(1)).unwrap());
    assert!(!cc.collides(&obj, TimeStep(2)).unwrap());
}
