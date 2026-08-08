#[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
use crcc::CrccError;
use crcc::{CollisionCheckerBuilder, CollisionEngine};
#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
use crcc::{CollisionObject, CollisionStatus, Compound, DynamicObstacle, Polygon, Pose, TimeStep};

#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
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
#[allow(clippy::unwrap_used)]
fn check_generic_engine<E>(obstacle: &CollisionObject, query: &CollisionObject)
where
    E: crcc::collision_checker::engine::EngineCollisionObject,
{
    use crcc::collision_checker::{CollisionChecker, CollisionCheckerBuilder};

    let checker: CollisionChecker<E> = CollisionCheckerBuilder::new()
        .with_static_obstacle(obstacle.clone())
        .build();

    assert_eq!(
        checker.collides_static(&query.clone().into()).unwrap(),
        CollisionStatus::CollidesStatic,
    );
}

#[cfg(any(feature = "parry", feature = "rhusics", feature = "collide"))]
#[test]
#[allow(clippy::unwrap_used)]
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
        let pair_collision = obstacle
            .collides(&query, Pose::IDENTITY, Pose::IDENTITY, engine)
            .unwrap();
        assert!(pair_collision, "{engine:?}: expected pair collision");

        let checker = CollisionCheckerBuilder::new()
            .with_static_obstacle(obstacle.clone())
            .build_with_engine(engine)
            .unwrap();

        assert_eq!(
            checker.collides_static(&query).unwrap(),
            CollisionStatus::CollidesStatic,
            "{engine:?}",
        );

        assert_eq!(
            checker.collides_dynamic(&dynamic).unwrap(),
            CollisionStatus::CollidesDynamic(TimeStep(0)),
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
#[allow(clippy::unwrap_used)]
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

#[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
#[test]
fn runtime_checker_rejects_engines_without_backend_features() {
    for engine in [
        CollisionEngine::Parry,
        CollisionEngine::Rhusics,
        CollisionEngine::Collide,
    ] {
        assert_eq!(
            CollisionCheckerBuilder::new()
                .build_with_engine(engine)
                .err(),
            Some(CrccError::Unsupported),
            "{engine:?}",
        );
    }
}
