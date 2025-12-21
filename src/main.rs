use commonroad_collision_checker::collision_checker::CollisionCheckerBuilder;
use commonroad_collision_checker::collision_object::CollisionObject;
use std::collections::HashMap;

fn main() {
    println!("Hello, world!");
    let cc = CollisionCheckerBuilder::new()
        .with_static_obstacle(CollisionObject::circle((0.0, 0.0), 1.0))
        .build_parry();
    let obj = CollisionObject::circle((1.0, 1.0), 2.0);
    let mut cache = HashMap::new();
    cache.insert(5, obj.into());
    let collides = cc.collides(cache.get(&5).unwrap()).unwrap();
    println!("Collides: {}", collides);
}
