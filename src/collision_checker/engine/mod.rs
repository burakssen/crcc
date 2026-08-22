use crate::collision_object::CollisionObject;
use crate::error::{CrccError, CrccResult};
use glamx::DPose2;

#[cfg(feature = "collide")]
pub mod collide;
#[cfg(feature = "parry")]
pub mod parry;
#[cfg(feature = "rhusics")]
pub mod rhusics;

/// A collision object converted for use by a specific backend.
pub trait EngineCollisionObject: From<CollisionObject> {
    /// Tests for a collision at the identity pose for both objects.
    ///
    /// # Errors
    ///
    /// Returns an error if the backend does not support the supplied geometry
    /// or if the underlying collision query fails.
    fn collides(&self, other: &Self) -> CrccResult<bool> {
        self.collides_at(DPose2::IDENTITY, other, DPose2::IDENTITY)
    }

    /// Tests for a collision at the supplied poses.
    ///
    /// # Errors
    ///
    /// Returns an error if the backend does not support the supplied geometry
    /// or if the underlying collision query fails.
    fn collides_at(&self, pos_self: DPose2, other: &Self, pos_other: DPose2) -> CrccResult<bool>;

    /// Tests continuously for a collision over two object motions.
    ///
    /// # Errors
    ///
    /// Returns an error if continuous collision detection is unsupported for
    /// the supplied geometry or if the underlying query fails.
    fn collides_continuous(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> CrccResult<bool>;

    /// Returns the non-negative separation distance at the supplied poses.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] by default. Backend implementations
    /// may also return an error when the supplied geometry cannot be processed.
    fn distance_at(&self, _pos_self: DPose2, _other: &Self, _pos_other: DPose2) -> CrccResult<f64> {
        Err(CrccError::Unsupported)
    }
}

/// Tests two collision objects at the supplied poses using `engine`.
///
/// # Errors
///
/// Returns [`CrccError::Unsupported`] when the selected backend feature is
/// disabled or the backend cannot process the supplied geometry. Other backend
/// query errors are propagated unchanged.
pub fn collides(
    object: &CollisionObject,
    pos_self: DPose2,
    other: &CollisionObject,
    pos_other: DPose2,
    engine: CollisionEngine,
) -> CrccResult<bool> {
    #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
    let _ = (object, pos_self, other, pos_other);

    match engine {
        #[cfg(feature = "parry")]
        CollisionEngine::Parry => {
            use parry::ParryCollisionObject;
            let object = ParryCollisionObject::from(object.clone());
            let other = ParryCollisionObject::from(other.clone());
            object.collides_at(pos_self, &other, pos_other)
        }
        #[cfg(not(feature = "parry"))]
        CollisionEngine::Parry => Err(CrccError::Unsupported),
        #[cfg(feature = "rhusics")]
        CollisionEngine::Rhusics => {
            use rhusics::RhusicsCoreCollisionObject;
            let object = RhusicsCoreCollisionObject::from(object.clone());
            let other = RhusicsCoreCollisionObject::from(other.clone());
            object.collides_at(pos_self, &other, pos_other)
        }
        #[cfg(not(feature = "rhusics"))]
        CollisionEngine::Rhusics => Err(CrccError::Unsupported),
        #[cfg(feature = "collide")]
        CollisionEngine::Collide => {
            use collide::CollideCollisionObject;
            let object = CollideCollisionObject::from(object.clone());
            let other = CollideCollisionObject::from(other.clone());
            object.collides_at(pos_self, &other, pos_other)
        }
        #[cfg(not(feature = "collide"))]
        CollisionEngine::Collide => Err(CrccError::Unsupported),
    }
}

/// Tests continuously for a collision over two object motions using `engine`.
///
/// # Errors
///
/// Returns [`CrccError::Unsupported`] when the selected backend feature or the
/// requested continuous query is unsupported. Other backend query errors are
/// propagated unchanged.
pub fn collides_continuous(
    object: &CollisionObject,
    start_pos_self: DPose2,
    end_pos_self: DPose2,
    other: &CollisionObject,
    start_pos_other: DPose2,
    end_pos_other: DPose2,
    engine: CollisionEngine,
) -> CrccResult<bool> {
    #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
    let _ = (
        object,
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
            let object = ParryCollisionObject::from(object.clone());
            let other = ParryCollisionObject::from(other.clone());
            object.collides_continuous(
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
            let object = RhusicsCoreCollisionObject::from(object.clone());
            let other = RhusicsCoreCollisionObject::from(other.clone());
            object.collides_continuous(
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
            let object = CollideCollisionObject::from(object.clone());
            let other = CollideCollisionObject::from(other.clone());
            object.collides_continuous(
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

/// Returns the distance between two collision objects at the supplied poses.
///
/// # Errors
///
/// Returns [`CrccError::Unsupported`] when the selected backend feature is
/// disabled or distance queries are unsupported for the supplied geometry.
/// Other backend query errors are propagated unchanged.
pub fn distance(
    object: &CollisionObject,
    pos_self: DPose2,
    other: &CollisionObject,
    pos_other: DPose2,
    engine: CollisionEngine,
) -> CrccResult<f64> {
    #[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
    let _ = (object, pos_self, other, pos_other);

    match engine {
        #[cfg(feature = "parry")]
        CollisionEngine::Parry => {
            use parry::ParryCollisionObject;
            let object = ParryCollisionObject::from(object.clone());
            let other = ParryCollisionObject::from(other.clone());
            object.distance_at(pos_self, &other, pos_other)
        }
        #[cfg(not(feature = "parry"))]
        CollisionEngine::Parry => Err(CrccError::Unsupported),
        #[cfg(feature = "rhusics")]
        CollisionEngine::Rhusics => object.distance_at(pos_self, other, pos_other),
        #[cfg(not(feature = "rhusics"))]
        CollisionEngine::Rhusics => Err(CrccError::Unsupported),
        #[cfg(feature = "collide")]
        CollisionEngine::Collide => object.distance_at(pos_self, other, pos_other),
        #[cfg(not(feature = "collide"))]
        CollisionEngine::Collide => Err(CrccError::Unsupported),
    }
}

#[cfg_attr(feature = "python_bindings", pyo3::pyclass(eq, eq_int))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
/// A runtime-selectable collision backend.
///
/// Selecting an engine whose Cargo feature is disabled causes queries and
/// checker construction to return [`CrccError::Unsupported`].
pub enum CollisionEngine {
    /// The Parry backend; the default whenever the `parry` feature is enabled.
    Parry,
    /// The Rhusics backend.
    Rhusics,
    /// The Collide backend.
    Collide,
}

impl Default for CollisionEngine {
    fn default() -> Self {
        default_collision_engine()
    }
}

#[cfg(feature = "parry")]
const fn default_collision_engine() -> CollisionEngine {
    CollisionEngine::Parry
}

#[cfg(all(not(feature = "parry"), feature = "rhusics"))]
const fn default_collision_engine() -> CollisionEngine {
    CollisionEngine::Rhusics
}

#[cfg(all(not(feature = "parry"), not(feature = "rhusics"), feature = "collide"))]
const fn default_collision_engine() -> CollisionEngine {
    CollisionEngine::Collide
}

#[cfg(not(any(feature = "parry", feature = "rhusics", feature = "collide")))]
const fn default_collision_engine() -> CollisionEngine {
    CollisionEngine::Parry
}

#[cfg(all(test, any(feature = "parry", feature = "rhusics", feature = "collide")))]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::{CollisionEngine, CrccError, collides, collides_continuous, distance};
    use crate::collision_checker::{CollisionCheckerBuilder, CollisionStatus};
    use crate::collision_object::CollisionObject;
    use crate::collision_object::simple::SimpleCollisionObject;
    use crate::time::TimeStep;
    use geo::{Polygon, Rect, Triangle};
    use glamx::DPose2;
    #[cfg(any(feature = "parry", feature = "collide"))]
    use std::f64::consts::SQRT_2;
    use std::f64::consts::{FRAC_PI_2, PI};

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

    #[cfg(any(feature = "rhusics", feature = "collide"))]
    fn non_parry_engines() -> Vec<CollisionEngine> {
        vec![
            #[cfg(feature = "rhusics")]
            CollisionEngine::Rhusics,
            #[cfg(feature = "collide")]
            CollisionEngine::Collide,
        ]
    }

    #[cfg(any(feature = "parry", feature = "collide"))]
    fn analytic_circle_ccd_engines() -> Vec<CollisionEngine> {
        vec![
            #[cfg(feature = "parry")]
            CollisionEngine::Parry,
            #[cfg(feature = "collide")]
            CollisionEngine::Collide,
        ]
    }

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
        for engine in engines() {
            assert_collision_at(left, pos_left, right, pos_right, engine, expected);
        }
    }

    fn assert_collision_at(
        left: &CollisionObject,
        pos_left: DPose2,
        right: &CollisionObject,
        pos_right: DPose2,
        engine: CollisionEngine,
        expected: bool,
    ) {
        let actual = collides(left, pos_left, right, pos_right, engine).unwrap();
        assert!(
            actual == expected,
            "{engine:?}: expected {expected}, got {actual}"
        );
    }

    #[cfg(any(feature = "rhusics", feature = "collide"))]
    fn assert_rhusics_and_collide_collision(
        left: &CollisionObject,
        pos_left: DPose2,
        right: &CollisionObject,
        pos_right: DPose2,
        expected: bool,
    ) {
        for engine in non_parry_engines() {
            assert_collision_at(left, pos_left, right, pos_right, engine, expected);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn assert_continuous_collision(
        left: &CollisionObject,
        left_start: DPose2,
        left_end: DPose2,
        right: &CollisionObject,
        right_start: DPose2,
        right_end: DPose2,
        engine: CollisionEngine,
        expected: bool,
    ) {
        let actual = collides_continuous(
            left,
            left_start,
            left_end,
            right,
            right_start,
            right_end,
            engine,
        )
        .unwrap();
        assert!(
            actual == expected,
            "{engine:?}: expected {expected}, got {actual}"
        );
    }

    #[test]
    fn discrete_engines_match_for_finite_primitives() {
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

        assert_engine_parity(&circle, &distant_circle, false);
        assert_engine_parity(&circle, &rectangle, true);
        assert_engine_parity(&triangle, &circle, true);
    }

    #[test]
    fn discrete_engines_handle_empty_full_and_tangent_shapes() {
        let empty = CollisionObject::empty();
        let full = CollisionObject::full_space();
        let circle = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
        let distant_circle = CollisionObject::circle((5.0, 0.0), 1.0).unwrap();

        assert_engine_parity(&empty, &full, false);
        assert_engine_parity(&full, &distant_circle, true);

        let tangent_pose = DPose2::translation(-3.0, 0.0);
        #[cfg(feature = "parry")]
        assert_collision_at(
            &circle,
            DPose2::IDENTITY,
            &distant_circle,
            tangent_pose,
            CollisionEngine::Parry,
            true,
        );
        // Rhusics uses native GJK semantics, which exclude exact tangency.
        #[cfg(feature = "rhusics")]
        assert_collision_at(
            &circle,
            DPose2::IDENTITY,
            &distant_circle,
            tangent_pose,
            CollisionEngine::Rhusics,
            false,
        );
        #[cfg(feature = "collide")]
        assert_collision_at(
            &circle,
            DPose2::IDENTITY,
            &distant_circle,
            tangent_pose,
            CollisionEngine::Collide,
            true,
        );
        assert_engine_parity_at(
            &circle,
            DPose2::IDENTITY,
            &distant_circle,
            DPose2::translation(-3.0 + 1e-9, 0.0),
            false,
        );
    }

    #[test]
    fn discrete_engines_match_for_polygons() {
        let circle = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
        let distant_circle = CollisionObject::circle((5.0, 0.0), 1.0).unwrap();
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

        assert_engine_parity(&convex_polygon, &circle, true);
        assert_engine_parity(&non_convex_polygon, &circle, true);
        assert_engine_parity(&polygon_with_hole, &distant_circle, false);
    }

    #[test]
    fn distance_reports_separation_for_basic_shapes() {
        let left = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
        let right = CollisionObject::circle((5.0, 0.0), 1.0).unwrap();

        for engine in engines() {
            let separation =
                distance(&left, DPose2::IDENTITY, &right, DPose2::IDENTITY, engine).unwrap();
            assert!((separation - 3.0).abs() < 1e-9);
        }
    }

    #[test]
    fn distance_contract_for_empty_and_full_space() {
        let circle = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
        let empty = CollisionObject::empty();
        let full = CollisionObject::full_space();

        for engine in engines() {
            assert_eq!(
                distance(&empty, DPose2::IDENTITY, &circle, DPose2::IDENTITY, engine),
                Err(CrccError::Unsupported),
                "{engine:?}"
            );
            assert_eq!(
                distance(&full, DPose2::IDENTITY, &circle, DPose2::IDENTITY, engine),
                Ok(0.0),
                "{engine:?}"
            );
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
        let x_le_zero = CollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0).unwrap();
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

    #[cfg(any(feature = "rhusics", feature = "collide"))]
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
    fn compound_collision_matches_expanded_child_pairs() {
        let left_parts = vec![
            CollisionObject::circle((-1.0, 0.0), 0.5).unwrap(),
            CollisionObject::rectangle(Rect::new((0.0, -0.4), (1.0, 0.4)), 0.25).unwrap(),
            CollisionObject::from(
                SimpleCollisionObject::triangle(Triangle::new(
                    (1.5, -0.5).into(),
                    (2.3, 0.0).into(),
                    (1.7, 0.8).into(),
                ))
                .unwrap(),
            ),
        ];
        let right_parts = vec![
            CollisionObject::circle((0.0, 0.0), 0.35).unwrap(),
            CollisionObject::polygon(Polygon::new(
                vec![
                    (-0.5, -0.5),
                    (0.7, -0.4),
                    (0.8, 0.5),
                    (-0.3, 0.7),
                    (-0.5, -0.5),
                ]
                .into(),
                vec![],
            ))
            .unwrap(),
        ];
        let left = CollisionObject::merge_all(left_parts.clone());
        let right = CollisionObject::merge_all(right_parts.clone());
        let pos_left = DPose2::new((0.4, -0.2).into(), 0.35);
        let pos_right = DPose2::new((1.6, 0.1).into(), -0.2);

        for engine in engines() {
            let compound = collides(&left, pos_left, &right, pos_right, engine).unwrap();
            let expanded = left_parts.iter().any(|left_part| {
                right_parts.iter().any(|right_part| {
                    collides(left_part, pos_left, right_part, pos_right, engine).unwrap()
                })
            });
            assert!(compound, "{engine:?}: expected compound collision");
            assert!(expanded, "{engine:?}: expected an expanded child collision");
            assert_eq!(compound, expanded, "{engine:?}");
        }
    }

    #[test]
    fn continuous_compound_collision_matches_expanded_child_pairs() {
        let left_parts = vec![
            CollisionObject::circle((-2.0, 0.0), 0.5).unwrap(),
            CollisionObject::rectangle(Rect::new((-0.5, -0.4), (0.5, 0.4)), 0.0).unwrap(),
        ];
        let right_parts = vec![
            CollisionObject::circle((0.0, 0.0), 0.4).unwrap(),
            CollisionObject::rectangle(Rect::new((2.0, -0.2), (3.0, 0.2)), 0.1).unwrap(),
        ];
        let left = CollisionObject::merge_all(left_parts.clone());
        let right = CollisionObject::merge_all(right_parts.clone());
        let left_start = DPose2::translation(-4.0, 0.0);
        let left_end = DPose2::translation(4.0, 0.0);
        let right_start = DPose2::IDENTITY;
        let right_end = DPose2::IDENTITY;

        for engine in engines() {
            let compound = collides_continuous(
                &left,
                left_start,
                left_end,
                &right,
                right_start,
                right_end,
                engine,
            )
            .unwrap();
            let expanded = left_parts.iter().any(|left_part| {
                right_parts.iter().any(|right_part| {
                    collides_continuous(
                        left_part,
                        left_start,
                        left_end,
                        right_part,
                        right_start,
                        right_end,
                        engine,
                    )
                    .unwrap()
                })
            });
            assert!(
                compound,
                "{engine:?}: expected continuous compound collision"
            );
            assert!(
                expanded,
                "{engine:?}: expected a continuous expanded child collision"
            );
            assert_eq!(compound, expanded, "{engine:?}");
        }
    }

    #[cfg(any(feature = "rhusics", feature = "collide"))]
    #[test]
    fn non_parry_backends_handle_half_space_pairs_exactly() {
        let x_le_zero = CollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0).unwrap();
        let x_ge_minus_one = CollisionObject::half_space_from_coeffs(-1.0, 0.0, 1.0).unwrap();
        let x_ge_one = CollisionObject::half_space_from_coeffs(-1.0, 0.0, -1.0).unwrap();
        let x_le_two = CollisionObject::half_space_from_coeffs(1.0, 0.0, 2.0).unwrap();
        let y_le_zero = CollisionObject::half_space_from_coeffs(0.0, 1.0, 0.0).unwrap();

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

    #[cfg(any(feature = "rhusics", feature = "collide"))]
    #[test]
    fn non_parry_backends_do_not_separate_nonparallel_half_spaces() {
        let angle = 5e-10_f64;
        let left = CollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0).unwrap();
        let right =
            CollisionObject::half_space_from_coeffs(-angle.cos(), angle.sin(), -1.0).unwrap();

        assert_rhusics_and_collide_collision(
            &left,
            DPose2::IDENTITY,
            &right,
            DPose2::IDENTITY,
            true,
        );
    }

    #[cfg(any(feature = "rhusics", feature = "collide"))]
    #[test]
    fn non_parry_distance_is_zero_for_nonparallel_half_spaces() {
        let angle = 1e-5_f64;
        let left = CollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0).unwrap();
        let right =
            CollisionObject::half_space_from_coeffs(-angle.cos(), angle.sin(), -1.0).unwrap();

        for engine in non_parry_engines() {
            let separation =
                distance(&left, DPose2::IDENTITY, &right, DPose2::IDENTITY, engine).unwrap();
            assert!(separation.abs() <= f64::EPSILON, "{engine:?}: {separation}");
        }
    }

    #[test]
    fn continuous_engines_detect_between_endpoint_collision() {
        let moving = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
        let fixed = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();

        for engine in engines() {
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
    fn continuous_engines_reject_parallel_disjoint_motion() {
        let left = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();
        let right = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();

        for engine in engines() {
            assert_continuous_collision(
                &left,
                DPose2::translation(-5.0, 0.0),
                DPose2::translation(5.0, 0.0),
                &right,
                DPose2::translation(-5.0, 3.0),
                DPose2::translation(5.0, 3.0),
                engine,
                false,
            );
        }
    }

    #[test]
    fn continuous_engines_include_overlapping_endpoint() {
        let moving = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();
        let fixed = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();

        for engine in engines() {
            assert_continuous_collision(
                &moving,
                DPose2::translation(-3.0, 0.0),
                DPose2::translation(0.5, 0.0),
                &fixed,
                DPose2::IDENTITY,
                DPose2::IDENTITY,
                engine,
                true,
            );
        }
    }

    #[test]
    fn continuous_engines_detect_two_moving_objects_crossing() {
        let left = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();
        let right = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();

        for engine in engines() {
            assert_continuous_collision(
                &left,
                DPose2::translation(-5.0, 0.0),
                DPose2::translation(5.0, 0.0),
                &right,
                DPose2::translation(5.0, 0.0),
                DPose2::translation(-5.0, 0.0),
                engine,
                true,
            );
        }
    }

    #[cfg(any(feature = "parry", feature = "collide"))]
    #[test]
    fn analytic_circle_ccd_detects_tiny_between_endpoint_collision() {
        let moving = CollisionObject::circle((0.0, 0.0), 1e-8).unwrap();
        let fixed = CollisionObject::circle((0.0, 0.0), 1e-8).unwrap();

        for engine in analytic_circle_ccd_engines() {
            assert!(
                collides_continuous(
                    &moving,
                    DPose2::translation(3e-8, 0.0),
                    DPose2::translation(-3e-8, 0.0),
                    &fixed,
                    DPose2::IDENTITY,
                    DPose2::IDENTITY,
                    engine,
                )
                .unwrap(),
                "{engine:?}",
            );
        }
    }

    #[cfg(any(feature = "parry", feature = "collide"))]
    #[test]
    fn continuous_engines_detect_off_center_circle_rotation() {
        let rotating = CollisionObject::circle((2.0, 0.0), 0.25).unwrap();
        let fixed = CollisionObject::circle((0.0, 0.0), 0.25).unwrap();

        for engine in analytic_circle_ccd_engines() {
            assert!(
                collides_continuous(
                    &rotating,
                    DPose2::IDENTITY,
                    DPose2::new((0.0, 0.0).into(), FRAC_PI_2),
                    &fixed,
                    DPose2::translation(SQRT_2, SQRT_2),
                    DPose2::translation(SQRT_2, SQRT_2),
                    engine,
                )
                .unwrap()
            );
        }
    }

    #[cfg(feature = "rhusics")]
    #[test]
    fn rhusics_continuous_finite_checks_include_endpoints() {
        let moving = CollisionObject::rectangle(Rect::new((-2.0, -0.1), (2.0, 0.1)), 0.0).unwrap();
        let fixed = CollisionObject::circle((0.0, 0.0), 0.2).unwrap();

        assert!(
            collides_continuous(
                &moving,
                DPose2::IDENTITY,
                DPose2::new((0.0, 0.0).into(), PI),
                &fixed,
                DPose2::IDENTITY,
                DPose2::IDENTITY,
                CollisionEngine::Rhusics,
            )
            .unwrap()
        );
    }

    #[cfg(any(feature = "rhusics", feature = "collide"))]
    #[test]
    fn engines_detect_continuous_half_space_endpoint_crossing() {
        let moving = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
        let half_space = CollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0).unwrap();

        for engine in non_parry_engines() {
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
            );
        }
    }

    #[cfg(any(feature = "rhusics", feature = "collide"))]
    #[test]
    fn non_parry_backends_detect_continuous_half_space_between_endpoints() {
        let moving = CollisionObject::rectangle(Rect::new((-2.0, -0.1), (2.0, 0.1)), 0.0).unwrap();
        let half_space = CollisionObject::half_space_from_coeffs(1.0, 0.0, 0.0).unwrap();

        for engine in non_parry_engines() {
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
            );
        }
    }

    #[cfg(feature = "parry")]
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
        );
    }

    #[cfg(any(feature = "rhusics", feature = "collide"))]
    #[test]
    fn non_parry_rotational_ccd_is_conservative() {
        let moving = CollisionObject::rectangle(Rect::new((-5.0, -0.1), (5.0, 0.1)), 0.0).unwrap();
        let fixed = CollisionObject::circle((0.0, 4.0), 0.5).unwrap();

        for engine in non_parry_engines() {
            assert!(
                collides_continuous(
                    &moving,
                    DPose2::IDENTITY,
                    DPose2::new((0.0, 0.0).into(), PI),
                    &fixed,
                    DPose2::IDENTITY,
                    DPose2::IDENTITY,
                    engine,
                )
                .unwrap()
            );
        }
    }

    #[cfg(any(feature = "rhusics", feature = "collide"))]
    #[test]
    fn non_parry_rotational_ccd_rejects_disjoint_motion_bounds() {
        let moving = CollisionObject::rectangle(Rect::new((-5.0, -0.1), (5.0, 0.1)), 0.0).unwrap();
        let fixed = CollisionObject::circle((100.0, 100.0), 0.5).unwrap();

        for engine in non_parry_engines() {
            assert!(
                !collides_continuous(
                    &moving,
                    DPose2::IDENTITY,
                    DPose2::new((0.0, 0.0).into(), PI),
                    &fixed,
                    DPose2::IDENTITY,
                    DPose2::IDENTITY,
                    engine,
                )
                .unwrap(),
                "{engine:?}"
            );
        }
    }

    #[test]
    fn full_space_distance_is_zero_for_every_engine() {
        let full = CollisionObject::full_space();
        let circle = CollisionObject::circle((100.0, 100.0), 1.0).unwrap();
        for engine in engines() {
            let separation =
                distance(&full, DPose2::IDENTITY, &circle, DPose2::IDENTITY, engine).unwrap();

            assert!(
                separation.abs() < 1e-9,
                "expected zero distance for {engine:?}, got {separation}",
            );
        }
    }

    #[cfg(feature = "parry")]
    #[test]
    fn parry_continuous_includes_endpoint_contact() {
        let moving = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();
        let fixed = CollisionObject::circle((0.0, 0.0), 0.5).unwrap();

        assert!(
            collides_continuous(
                &moving,
                DPose2::translation(-3.0, 0.0),
                DPose2::translation(0.0, 0.0),
                &fixed,
                DPose2::translation(1.0, 0.0),
                DPose2::translation(1.0, 0.0),
                CollisionEngine::Parry,
            )
            .unwrap()
        );
    }

    #[test]
    fn builder_can_select_each_engine() {
        for engine in engines() {
            let checker = CollisionCheckerBuilder::new()
                .with_static_obstacle(SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap())
                .build_with_engine(engine)
                .unwrap();
            let query = CollisionObject::circle((0.0, 0.0), 1.0).unwrap();

            assert_eq!(
                checker
                    .collides_static_range(&query, DPose2::IDENTITY, TimeStep(0)..=TimeStep(0))
                    .unwrap(),
                CollisionStatus::CollidesStatic
            );
            assert_eq!(checker.engine(), engine);
        }
    }
}
