use commonroad_collision_checker::collision_checker::CollisionCheckerBuilder;
use commonroad_collision_checker::collision_object::StaticCollisionObject;
use commonroad_collision_checker::collision_object::simple::SimpleCollisionObject;
use std::collections::HashMap;

fn main() {
    println!("Hello, world!");
    let cc = CollisionCheckerBuilder::new()
        .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0))
        .build_parry();
    let obj = SimpleCollisionObject::circle((1.0, 1.0), 2.0);
    let mut cache = HashMap::new();
    cache.insert(5, StaticCollisionObject::from(obj).into());
    let collides = cc.collides(cache.get(&5).unwrap()).unwrap();
    println!("Collides: {}", collides);
}
