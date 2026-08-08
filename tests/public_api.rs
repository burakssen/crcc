#![allow(clippy::unwrap_used)]

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

#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide",))]
fn check_generic_engine<E>(obstacle: &CollisionObject, query: &CollisionObject)
where
    E: crcc::collision_checker::engine::EngineCollisionObject,
{
    use crcc::collision_checker::{CollisionChecker, CollisionCheckerBuilder};

    let checker: CollisionChecker<E> = CollisionCheckerBuilder::new()
        .with_static_obstacle(obstacle.clone())
        .build();

    assert!(
        checker
            .collides_static(&query.clone().into())
            .unwrap()
            .collides(),
    );
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
    )
    .unwrap();

    for engine in engines() {
        assert!(
            obstacle
                .collides(&query, Pose::IDENTITY, Pose::IDENTITY, engine,)
                .unwrap(),
            "{engine:?}",
        );

        let checker = CollisionCheckerBuilder::new()
            .with_static_obstacle(obstacle.clone())
            .build_with_engine(engine)
            .unwrap();

        assert_eq!(
            checker.collides_static(&query).unwrap(),
            CollisionStatus::CollidesStatic,
            "{engine:?}",
        );

        assert!(
            checker.collides_dynamic(&dynamic).unwrap().collides(),
            "{engine:?}",
        );

        #[cfg(feature = "rayon")]
        assert_eq!(
            checker.collides_static_batch(&[(query.clone(), Pose::IDENTITY)], ..,),
            vec![Ok(CollisionStatus::CollidesStatic)],
            "{engine:?}",
        );
    }
}

#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide",))]
#[test]
fn module_api_supports_generic_checkers_for_each_engine() {
    let obstacle = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();

    let query = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();

    #[cfg(feature = "parry")]
    check_generic_engine::<crcc::collision_checker::engine::parry::ParryCollisionObject>(
        &obstacle, &query,
    );

    #[cfg(feature = "rhusics")]
    check_generic_engine::<crcc::collision_checker::engine::rhusics::RhusicsCoreCollisionObject>(
        &obstacle, &query,
    );

    #[cfg(feature = "collide")]
    check_generic_engine::<crcc::collision_checker::engine::collide::CollideCollisionObject>(
        &obstacle, &query,
    );
}
