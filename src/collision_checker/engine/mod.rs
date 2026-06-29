use crate::collision_object::CollisionObject;
use crate::error::CrccError;
use glamx::DPose2;

#[cfg(feature = "collide")]
pub mod collide;
#[cfg(feature = "parry")]
pub mod parry;
#[cfg(feature = "rhusics")]
pub mod rhusics;

pub trait EngineCollisionObject: From<CollisionObject> {
    fn collides(&self, other: &Self) -> Result<bool, CrccError> {
        self.collides_at(DPose2::IDENTITY, other, DPose2::IDENTITY)
    }

    fn collides_at(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<bool, CrccError>;

    fn collides_continuous(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> Result<bool, CrccError>;

    fn distance_at(
        &self,
        _pos_self: DPose2,
        _other: &Self,
        _pos_other: DPose2,
    ) -> Result<f64, CrccError> {
        Err(CrccError::Unsupported)
    }
}

pub fn collides(
    slf: &CollisionObject,
    pos_self: DPose2,
    other: &CollisionObject,
    pos_other: DPose2,
    engine: CollisionEngine,
) -> Result<bool, CrccError> {
    #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
    let _ = (slf, pos_self, other, pos_other);

    match engine {
        #[cfg(feature = "parry")]
        CollisionEngine::Parry => {
            use parry::ParryCollisionObject;
            let slf = ParryCollisionObject::from(slf.clone());
            let other = ParryCollisionObject::from(other.clone());
            slf.collides_at(pos_self, &other, pos_other)
        }
        #[cfg(not(feature = "parry"))]
        CollisionEngine::Parry => Err(CrccError::Unsupported),
        #[cfg(feature = "rhusics")]
        CollisionEngine::Rhusics => {
            use rhusics::RhusicsCoreCollisionObject;
            let slf = RhusicsCoreCollisionObject::from(slf.clone());
            let other = RhusicsCoreCollisionObject::from(other.clone());
            slf.collides_at(pos_self, &other, pos_other)
        }
        #[cfg(not(feature = "rhusics"))]
        CollisionEngine::Rhusics => Err(CrccError::Unsupported),
        #[cfg(feature = "collide")]
        CollisionEngine::Collide => {
            use collide::CollideCollisionObject;
            let slf = CollideCollisionObject::from(slf.clone());
            let other = CollideCollisionObject::from(other.clone());
            slf.collides_at(pos_self, &other, pos_other)
        }
        #[cfg(not(feature = "collide"))]
        CollisionEngine::Collide => Err(CrccError::Unsupported),
    }
}

pub fn collides_continuous(
    slf: &CollisionObject,
    start_pos_self: DPose2,
    end_pos_self: DPose2,
    other: &CollisionObject,
    start_pos_other: DPose2,
    end_pos_other: DPose2,
    engine: CollisionEngine,
) -> Result<bool, CrccError> {
    #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
    let _ = (
        slf,
        start_pos_self,
        end_pos_self,
        other,
        start_pos_other,
        end_pos_other,
    );

    match engine {
        #[cfg(feature = "parry")]
        CollisionEngine::Parry => {
            use parry::ParryCollisionObject;
            let slf = ParryCollisionObject::from(slf.clone());
            let other = ParryCollisionObject::from(other.clone());
            slf.collides_continuous(
                start_pos_self,
                end_pos_self,
                &other,
                start_pos_other,
                end_pos_other,
            )
        }
        #[cfg(not(feature = "parry"))]
        CollisionEngine::Parry => Err(CrccError::Unsupported),
        #[cfg(feature = "rhusics")]
        CollisionEngine::Rhusics => {
            use rhusics::RhusicsCoreCollisionObject;
            let slf = RhusicsCoreCollisionObject::from(slf.clone());
            let other = RhusicsCoreCollisionObject::from(other.clone());
            slf.collides_continuous(
                start_pos_self,
                end_pos_self,
                &other,
                start_pos_other,
                end_pos_other,
            )
        }
        #[cfg(not(feature = "rhusics"))]
        CollisionEngine::Rhusics => Err(CrccError::Unsupported),
        #[cfg(feature = "collide")]
        CollisionEngine::Collide => {
            use collide::CollideCollisionObject;
            let slf = CollideCollisionObject::from(slf.clone());
            let other = CollideCollisionObject::from(other.clone());
            slf.collides_continuous(
                start_pos_self,
                end_pos_self,
                &other,
                start_pos_other,
                end_pos_other,
            )
        }
        #[cfg(not(feature = "collide"))]
        CollisionEngine::Collide => Err(CrccError::Unsupported),
    }
}

pub fn distance(
    slf: &CollisionObject,
    pos_self: DPose2,
    other: &CollisionObject,
    pos_other: DPose2,
    engine: CollisionEngine,
) -> Result<f64, CrccError> {
    #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
    let _ = (slf, pos_self, other, pos_other);

    match engine {
        #[cfg(feature = "parry")]
        CollisionEngine::Parry => {
            use parry::ParryCollisionObject;
            let slf = ParryCollisionObject::from(slf.clone());
            let other = ParryCollisionObject::from(other.clone());
            slf.distance_at(pos_self, &other, pos_other)
        }
        #[cfg(not(feature = "parry"))]
        CollisionEngine::Parry => Err(CrccError::Unsupported),
        #[cfg(feature = "rhusics")]
        CollisionEngine::Rhusics => {
            use rhusics::RhusicsCoreCollisionObject;
            let slf = RhusicsCoreCollisionObject::from(slf.clone());
            let other = RhusicsCoreCollisionObject::from(other.clone());
            slf.distance_at(pos_self, &other, pos_other)
        }
        #[cfg(not(feature = "rhusics"))]
        CollisionEngine::Rhusics => Err(CrccError::Unsupported),
        #[cfg(feature = "collide")]
        CollisionEngine::Collide => {
            use collide::CollideCollisionObject;
            let slf = CollideCollisionObject::from(slf.clone());
            let other = CollideCollisionObject::from(other.clone());
            slf.distance_at(pos_self, &other, pos_other)
        }
        #[cfg(not(feature = "collide"))]
        CollisionEngine::Collide => Err(CrccError::Unsupported),
    }
}

#[cfg_attr(feature = "python_bindings", pyo3::pyclass(eq, eq_int))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CollisionEngine {
    Parry,
    Rhusics,
    Collide,
}

impl Default for CollisionEngine {
    fn default() -> Self {
        default_collision_engine()
    }
}

#[cfg(feature = "parry")]
fn default_collision_engine() -> CollisionEngine {
    CollisionEngine::Parry
}

#[cfg(all(not(feature = "parry"), feature = "rhusics"))]
fn default_collision_engine() -> CollisionEngine {
    CollisionEngine::Rhusics
}

#[cfg(all(not(feature = "parry"), not(feature = "rhusics"), feature = "collide"))]
fn default_collision_engine() -> CollisionEngine {
    CollisionEngine::Collide
}

#[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
fn default_collision_engine() -> CollisionEngine {
    CollisionEngine::Parry
}

#[cfg(all(test, feature = "parry", feature = "rhusics", feature = "collide"))]
mod tests {
    use super::{CollisionEngine, collides, collides_continuous, distance};
    use crate::collision_checker::{CollisionCheckerBuilder, CollisionStatus};
    use crate::collision_object::CollisionObject;
    use crate::collision_object::simple::SimpleCollisionObject;
    use crate::time::TimeStep;
    use geo::{Polygon, Rect, Triangle};
    use glamx::DPose2;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn assert_engine_parity(left: &CollisionObject, right: &CollisionObject, expected: bool) {
        assert_engine_parity_at(left, DPose2::IDENTITY, right, DPose2::IDENTITY, expected);
    }

    fn assert_engine_parity_at(
        left: &CollisionObject,
        pos_left: DPose2,
        right: &CollisionObject,
        pos_right: DPose2,
        expected: bool,
    ) {
        let parry = collides(left, pos_left, right, pos_right, CollisionEngine::Parry).unwrap();
        let rhusics = collides(left, pos_left, right, pos_right, CollisionEngine::Rhusics).unwrap();
        let collide = collides(left, pos_left, right, pos_right, CollisionEngine::Collide).unwrap();
        assert_eq!(parry, expected);
        assert_eq!(rhusics, expected);
        assert_eq!(collide, expected);
    }

    fn assert_rhusics_and_collide_collision(
        left: &CollisionObject,
        pos_left: DPose2,
        right: &CollisionObject,
        pos_right: DPose2,
        expected: bool,
    ) {
        let rhusics = collides(left, pos_left, right, pos_right, CollisionEngine::Rhusics).unwrap();
        let collide = collides(left, pos_left, right, pos_right, CollisionEngine::Collide).unwrap();
        assert_eq!(rhusics, expected);
        assert_eq!(collide, expected);
    }

    #[test]
    fn discrete_engines_match_for_basic_shapes() {
        let empty = CollisionObject::empty();
        let full = CollisionObject::full_space();
        let circle = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
        let distant_circle = CollisionObject::circle((5.0, 0.0), 1.0).unwrap();
        let rectangle =
            CollisionObject::rectangle(Rect::new((-1.0, -0.5), (1.0, 0.5)), 0.4).unwrap();
        let triangle = CollisionObject::from(
            SimpleCollisionObject::triangle(Triangle::new(
                (0.0, 0.0).into(),
                (2.0, 0.0).into(),
                (1.0, 1.0).into(),
            ))
            .unwrap(),
        );
        let convex_polygon = CollisionObject::polygon(Polygon::new(
            vec![
                (-1.0, -1.0),
                (1.0, -1.0),
                (1.0, 1.0),
                (-1.0, 1.0),
                (-1.0, -1.0),
            ]
            .into(),
            vec![],
        ))
        .unwrap();
        let non_convex_polygon = CollisionObject::polygon(Polygon::new(
            vec![
                (0.0, 0.0),
                (2.0, 0.0),
                (2.0, 2.0),
                (1.0, 1.0),
                (0.0, 2.0),
                (0.0, 0.0),
            ]
            .into(),
            vec![],
        ))
        .unwrap();
        let polygon_with_hole = CollisionObject::polygon(Polygon::new(
            vec![
                (-3.0, -3.0),
                (3.0, -3.0),
                (3.0, 3.0),
                (-3.0, 3.0),
                (-3.0, -3.0),
            ]
            .into(),
            vec![
                vec![
                    (-0.5, -0.5),
                    (-0.5, 0.5),
                    (0.5, 0.5),
                    (0.5, -0.5),
                    (-0.5, -0.5),
                ]
                .into(),
            ],
        ))
        .unwrap();

        assert_engine_parity(&empty, &full, false);
        assert_engine_parity(&full, &distant_circle, true);
        assert_engine_parity(&circle, &distant_circle, false);
        assert_engine_parity_at(
            &circle,
            DPose2::IDENTITY,
            &distant_circle,
            DPose2::translation(-3.0, 0.0),
            true,
        );
        assert_engine_parity_at(
            &circle,
            DPose2::IDENTITY,
            &distant_circle,
            DPose2::translation(-3.0 + 1e-9, 0.0),
            false,
        );
        assert_engine_parity(&circle, &rectangle, true);
        assert_engine_parity(&triangle, &circle, true);
        assert_engine_parity(&convex_polygon, &circle, true);
        assert_engine_parity(&non_convex_polygon, &circle, true);
        assert_engine_parity(&polygon_with_hole, &distant_circle, false);
    }

    #[test]
    fn parry_distance_reports_separation_for_basic_shapes() {
        let left = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
        let right = CollisionObject::circle((5.0, 0.0), 1.0).unwrap();

        let separation = distance(
            &left,
            DPose2::IDENTITY,
            &right,
            DPose2::IDENTITY,
            CollisionEngine::Parry,
        )
        .unwrap();

        assert!((separation - 3.0).abs() < 1e-9);
    }

    #[test]
    fn all_engines_distance_reports_separation_for_basic_shapes() {
        let left = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
        let right = CollisionObject::circle((5.0, 0.0), 1.0).unwrap();

        for engine in [
            CollisionEngine::Parry,
            CollisionEngine::Rhusics,
            CollisionEngine::Collide,
        ] {
            let separation =
                distance(&left, DPose2::IDENTITY, &right, DPose2::IDENTITY, engine).unwrap();
            assert!((separation - 3.0).abs() < 1e-9);
        }
    }

    #[test]
    fn discrete_engines_match_for_rectangle_extents() {
        let left = CollisionObject::rectangle(Rect::new((-2.0, -1.0), (2.0, 1.0)), 0.0).unwrap();
        let right = CollisionObject::rectangle(Rect::new((-2.0, -1.0), (2.0, 1.0)), 0.0).unwrap();

        assert_engine_parity_at(
            &left,
            DPose2::IDENTITY,
            &right,
            DPose2::translation(3.0, 0.0),
            true,
        );
    }

    #[test]
    fn discrete_engines_match_for_reproduced_static_dynamic_rectangle_collision() {
        let obstacle =
            CollisionObject::rectangle(Rect::new((-2.15, -0.9), (2.15, 0.9)), 0.0).unwrap();
        let query = CollisionObject::rectangle(Rect::new((-2.25, -1.0), (2.25, 1.0)), 0.0).unwrap();

        assert_engine_parity_at(
            &obstacle,
            DPose2::new((34.1576, 1.1173).into(), 0.2965),
            &query,
            DPose2::new((37.33, 4.07).into(), -2.207),
            true,
        );
    }

    #[test]
    fn discrete_engines_match_for_half_spaces() {
        let x_le_zero = CollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0);
        let far_inside_circle = CollisionObject::circle((-2_000_000.0, 0.0), 1.0).unwrap();
        let far_outside_circle = CollisionObject::circle((2_000_000.0, 0.0), 1.0).unwrap();
        let outside_old_tangent_extent = CollisionObject::circle((-1.0, 2_000_000.0), 1.0).unwrap();

        assert_engine_parity(&x_le_zero, &far_inside_circle, true);
        assert_engine_parity(&x_le_zero, &far_outside_circle, false);
        assert_engine_parity(&x_le_zero, &outside_old_tangent_extent, true);
        assert_engine_parity(&far_inside_circle, &x_le_zero, true);
        assert_engine_parity(&far_outside_circle, &x_le_zero, false);

        let query_circle = CollisionObject::circle((0.0, 0.0), 0.25).unwrap();
        assert_engine_parity_at(
            &x_le_zero,
            DPose2::translation(5.0, 0.0),
            &query_circle,
            DPose2::translation(4.5, 0.0),
            true,
        );
        assert_engine_parity_at(
            &x_le_zero,
            DPose2::translation(5.0, 0.0),
            &query_circle,
            DPose2::translation(6.5, 0.0),
            false,
        );
        assert_engine_parity_at(
            &x_le_zero,
            DPose2::new((0.0, 0.0).into(), FRAC_PI_2),
            &query_circle,
            DPose2::translation(0.0, -0.5),
            true,
        );
    }

    #[test]
    fn non_parry_backends_handle_large_finite_shapes_without_bound_wrappers() {
        let large_polygon = CollisionObject::polygon(Polygon::new(
            vec![
                (-1_000_000.0, -1_000_000.0),
                (1_000_000.0, -1_000_000.0),
                (1_000_000.0, 1_000_000.0),
                (-1_000_000.0, 1_000_000.0),
                (-1_000_000.0, -1_000_000.0),
            ]
            .into(),
            vec![],
        ))
        .unwrap();
        let polygon_with_hole = CollisionObject::polygon(Polygon::new(
            vec![
                (-10.0, -10.0),
                (10.0, -10.0),
                (10.0, 10.0),
                (-10.0, 10.0),
                (-10.0, -10.0),
            ]
            .into(),
            vec![
                vec![
                    (-1.0, -1.0),
                    (-1.0, 1.0),
                    (1.0, 1.0),
                    (1.0, -1.0),
                    (-1.0, -1.0),
                ]
                .into(),
            ],
        ))
        .unwrap();
        let circle = CollisionObject::circle((500_000.0, 500_000.0), 1.0).unwrap();
        let hole_circle = CollisionObject::circle((0.0, 0.0), 0.25).unwrap();

        assert_rhusics_and_collide_collision(
            &large_polygon,
            DPose2::IDENTITY,
            &circle,
            DPose2::IDENTITY,
            true,
        );
        assert_rhusics_and_collide_collision(
            &polygon_with_hole,
            DPose2::IDENTITY,
            &hole_circle,
            DPose2::IDENTITY,
            false,
        );
    }

    #[test]
    fn engines_match_for_non_convex_polygon_collision() {
        let non_convex_polygon = CollisionObject::polygon(Polygon::new(
            vec![
                (0.0, 0.0),
                (5.0, 0.0),
                (5.0, 1.0),
                (1.0, 1.0),
                (1.0, 5.0),
                (0.0, 5.0),
                (0.0, 0.0),
            ]
            .into(),
            vec![],
        ))
        .unwrap();
        let query = CollisionObject::rectangle(Rect::new((-0.75, -0.4), (0.75, 0.4)), 0.6).unwrap();

        assert_engine_parity_at(
            &non_convex_polygon,
            DPose2::IDENTITY,
            &query,
            DPose2::translation(4.4, 0.5),
            true,
        );
    }

    #[test]
    fn non_parry_backends_handle_half_space_pairs_exactly() {
        let x_le_zero = CollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0);
        let x_ge_minus_one = CollisionObject::half_space_from_coeffs(-1.0, 0.0, 1.0);
        let x_ge_one = CollisionObject::half_space_from_coeffs(-1.0, 0.0, -1.0);
        let x_le_two = CollisionObject::half_space_from_coeffs(1.0, 0.0, 2.0);
        let y_le_zero = CollisionObject::half_space_from_coeffs(0.0, 1.0, 0.0);

        assert_rhusics_and_collide_collision(
            &x_le_zero,
            DPose2::IDENTITY,
            &x_ge_minus_one,
            DPose2::IDENTITY,
            true,
        );
        assert_rhusics_and_collide_collision(
            &x_le_zero,
            DPose2::IDENTITY,
            &x_ge_one,
            DPose2::IDENTITY,
            false,
        );
        assert_rhusics_and_collide_collision(
            &x_le_zero,
            DPose2::IDENTITY,
            &x_le_two,
            DPose2::IDENTITY,
            true,
        );
        assert_rhusics_and_collide_collision(
            &x_le_zero,
            DPose2::IDENTITY,
            &y_le_zero,
            DPose2::IDENTITY,
            true,
        );
    }

    #[test]
    fn continuous_engines_detect_between_endpoint_collision() {
        let moving = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
        let fixed = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();

        for engine in [
            CollisionEngine::Parry,
            CollisionEngine::Rhusics,
            CollisionEngine::Collide,
        ] {
            assert!(
                collides_continuous(
                    &moving,
                    DPose2::translation(-5.0, 0.0),
                    DPose2::translation(5.0, 0.0),
                    &fixed,
                    DPose2::IDENTITY,
                    DPose2::IDENTITY,
                    engine,
                )
                .unwrap()
            );
        }
    }

    #[test]
    fn engines_detect_continuous_half_space_endpoint_crossing() {
        let moving = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
        let half_space = CollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0);

        for engine in [CollisionEngine::Rhusics, CollisionEngine::Collide] {
            assert!(
                collides_continuous(
                    &moving,
                    DPose2::translation(5.0, 0.0),
                    DPose2::translation(-5.0, 0.0),
                    &half_space,
                    DPose2::IDENTITY,
                    DPose2::IDENTITY,
                    engine,
                )
                .unwrap()
            )
        }
    }

    #[test]
    fn non_parry_backends_detect_continuous_half_space_between_endpoints() {
        let moving = CollisionObject::rectangle(Rect::new((-2.0, -0.1), (2.0, 0.1)), 0.0).unwrap();
        let half_space = CollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0);

        for engine in [CollisionEngine::Rhusics, CollisionEngine::Collide] {
            assert!(
                collides_continuous(
                    &moving,
                    DPose2::new((1.2, 0.0).into(), FRAC_PI_2),
                    DPose2::new((1.2, 0.0).into(), -FRAC_PI_2),
                    &half_space,
                    DPose2::IDENTITY,
                    DPose2::IDENTITY,
                    engine,
                )
                .unwrap()
            )
        }
    }

    #[test]
    fn continuous_engines_use_shortest_rotation_across_angle_wrap() {
        let moving = CollisionObject::rectangle(Rect::new((-5.0, -0.1), (5.0, 0.1)), 0.0).unwrap();
        let fixed = CollisionObject::circle((0.0, 4.0), 0.5).unwrap();

        assert!(
            !collides_continuous(
                &moving,
                DPose2::new((0.0, 0.0).into(), PI - 0.1),
                DPose2::new((0.0, 0.0).into(), -PI + 0.1),
                &fixed,
                DPose2::IDENTITY,
                DPose2::IDENTITY,
                CollisionEngine::Parry,
            )
            .unwrap()
        )
    }

    #[test]
    fn builder_can_select_each_engine() {
        for engine in [
            CollisionEngine::Parry,
            CollisionEngine::Rhusics,
            CollisionEngine::Collide,
        ] {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();
            let query = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();

            assert_eq!(
                checker.collides_static_at(&query, TimeStep(0)).unwrap(),
                CollisionStatus::CollidesStatic
            );
            assert_eq!(checker.engine(), engine);
        }
    }
}
