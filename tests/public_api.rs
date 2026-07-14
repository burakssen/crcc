use crcc::{
    CollisionCheckerBuilder, CollisionEngine, CollisionObject, CollisionStatus, Compound,
    DynamicObstacle, Polygon, Pose, TimeStep,
};

#[test]
fn root_api_covers_pair_and_checker_queries() {
    let _: Option<Polygon> = None;
    let obstacle = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
    let query = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();
    let _: Compound = CollisionObject::merge_all([query.clone()]);
    assert!(
        obstacle
            .collides(
                &query,
                Pose::IDENTITY,
                Pose::IDENTITY,
                CollisionEngine::default(),
            )
            .unwrap()
    );

    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(obstacle)
        .build_with_engine(CollisionEngine::default())
        .unwrap();
    assert_eq!(
        checker.collides_static(&query).unwrap(),
        CollisionStatus::CollidesStatic
    );

    let dynamic = DynamicObstacle::new(
        query.clone(),
        vec![Pose::translation(3.0, 0.0), Pose::IDENTITY],
        TimeStep(0),
    );
    assert!(checker.collides_dynamic(&dynamic).unwrap().collides());

    #[cfg(feature = "rayon")]
    assert_eq!(
        checker.collides_static_batch(&[(query, Pose::IDENTITY)], ..),
        vec![Ok(CollisionStatus::CollidesStatic)]
    );
}

#[cfg(feature = "parry")]
#[test]
fn module_api_supports_generic_and_runtime_selected_checkers() {
    use crcc::collision_checker::engine::parry::ParryCollisionObject;
    use crcc::collision_checker::{CollisionChecker, CollisionCheckerBuilder};
    use crcc::collision_object::CollisionObject;

    let obstacle = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
    let query = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();
    let checker: CollisionChecker<ParryCollisionObject> = CollisionCheckerBuilder::new()
        .with_static_obstacle(obstacle.clone())
        .build();
    assert!(
        checker
            .collides_static(&query.clone().into())
            .unwrap()
            .collides()
    );

    let selected = CollisionCheckerBuilder::new()
        .with_static_obstacle(obstacle)
        .build_with_engine(CollisionEngine::Parry)
        .unwrap();
    assert!(selected.collides_static(&query).unwrap().collides());

    #[cfg(feature = "rayon")]
    assert!(
        selected.par_static(&[(query, Pose::IDENTITY)], ..)[0]
            .as_ref()
            .unwrap()
            .collides()
    );
}
