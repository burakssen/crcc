use crcc::{
    CollisionCheckerBuilder, CollisionEngine, CollisionObject, CollisionStatus, Compound,
    DynamicObstacle, Polygon, Pose, TimeStep,
};

fn engines() -> Vec<CollisionEngine> {
    vec![
        #[cfg(feature = "parry")]
        CollisionEngine::Parry,
        #[cfg(feature = "rhusics")]
        CollisionEngine::Rhusics,
        #[cfg(feature = "collide")]
        CollisionEngine::Collide,
    ]
}

#[test]
fn root_api_covers_pair_and_checker_queries() {
    let _: Option<Polygon> = None;
    let obstacle = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
    let query = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();
    let _: Compound = CollisionObject::merge_all([query.clone()]);
    let dynamic = DynamicObstacle::new(
        query.clone(),
        vec![Pose::translation(3.0, 0.0), Pose::IDENTITY],
        TimeStep(0),
    );

    for engine in engines() {
        assert!(
            obstacle
                .collides(&query, Pose::IDENTITY, Pose::IDENTITY, engine)
                .unwrap(),
            "{engine:?}"
        );

        let checker = CollisionCheckerBuilder::new()
            .with_static_obstacle(obstacle.clone())
            .build_with_engine(engine)
            .unwrap();
        assert_eq!(
            checker.collides_static(&query).unwrap(),
            CollisionStatus::CollidesStatic,
            "{engine:?}"
        );
        assert!(
            checker.collides_dynamic(&dynamic).unwrap().collides(),
            "{engine:?}"
        );

        #[cfg(feature = "rayon")]
        assert_eq!(
            checker.collides_static_batch(&[(query.clone(), Pose::IDENTITY)], ..),
            vec![Ok(CollisionStatus::CollidesStatic)],
            "{engine:?}"
        );
    }
}

#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
#[test]
fn module_api_supports_generic_checkers_for_each_engine() {
    use crcc::collision_checker::engine::EngineCollisionObject;
    use crcc::collision_checker::{CollisionChecker, CollisionCheckerBuilder};
    use crcc::collision_object::CollisionObject;

    let obstacle = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
    let query = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();

    fn check<E: EngineCollisionObject>(obstacle: &CollisionObject, query: &CollisionObject) {
        let checker: CollisionChecker<E> = CollisionCheckerBuilder::new()
            .with_static_obstacle(obstacle.clone())
            .build();
        assert!(
            checker
                .collides_static(&query.clone().into())
                .unwrap()
                .collides()
        );
    }

    #[cfg(feature = "parry")]
    check::<crcc::collision_checker::engine::parry::ParryCollisionObject>(&obstacle, &query);
    #[cfg(feature = "rhusics")]
    check::<crcc::collision_checker::engine::rhusics::RhusicsCoreCollisionObject>(
        &obstacle, &query,
    );
    #[cfg(feature = "collide")]
    check::<crcc::collision_checker::engine::collide::CollideCollisionObject>(&obstacle, &query);
}
