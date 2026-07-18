use crate::collision_checker::engine::{self, CollisionEngine};
use crate::collision_object::simple::SimpleCollisionObject;
use crate::collision_object::simple::SweptArea;
use crate::error::CrccResult;
use geo::{Polygon, Rect, Triangle};
use glamx::{DPose2, DVec2};
use itertools::Itertools;

#[cfg(any(feature = "rhusics", feature = "collide"))]
pub mod distance;
pub mod dynamic;
pub mod simple;

pub use dynamic::DynamicObstacle;

#[derive(Debug, Clone)]
/// A shape or compound of shapes accepted by every public query.
///
/// Constructors validate geometry and decompose complex polygons internally.
/// A merged object represents the union of its children.
pub struct CollisionObject {
    collision_objects: Vec<SimpleCollisionObject>,
}

impl CollisionObject {
    /// Performs a discrete pair collision query at two poses.
    pub fn collides(
        &self,
        other: &Self,
        pos_self: DPose2,
        pos_other: DPose2,
        engine: CollisionEngine,
    ) -> CrccResult<bool> {
        engine::collides(self, pos_self, other, pos_other, engine)
    }

    /// Checks two motions continuously between their start and end poses.
    ///
    /// `false` certifies separation over the interval; `true` can be a
    /// conservative positive for rotations and complex shapes.
    pub fn collides_continuous(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
        engine: CollisionEngine,
    ) -> CrccResult<bool> {
        engine::collides_continuous(
            self,
            start_pos_self,
            end_pos_self,
            other,
            start_pos_other,
            end_pos_other,
            engine,
        )
    }

    /// Returns the non-negative separation distance at two poses.
    ///
    /// Returns [`crate::CrccError::Unsupported`] when the engine does not support
    /// the requested shape combination.
    pub fn distance(
        &self,
        other: &Self,
        pos_self: DPose2,
        pos_other: DPose2,
        engine: CollisionEngine,
    ) -> CrccResult<f64> {
        engine::distance(self, pos_self, other, pos_other, engine)
    }

    fn new(collision_objects: Vec<SimpleCollisionObject>) -> Self {
        Self { collision_objects }
    }

    /// Creates an object that never collides.
    pub fn empty() -> Self {
        Self::new(Vec::new())
    }

    /// Creates an object occupying the entire plane.
    pub fn full_space() -> Self {
        Self::new(vec![SimpleCollisionObject::full_space()])
    }

    /// Creates the half-space `normal · point <= offset`.
    pub fn half_space(outward_normal: impl Into<DVec2>, offset: f64) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::half_space(outward_normal, offset)?.into())
    }

    /// Creates the half-space to the right of the directed line `p1 -> p2`.
    pub fn half_space_from_points(p1: (f64, f64), p2: (f64, f64)) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::half_space_from_points(p1, p2)?.into())
    }

    /// Creates the half-space `a*x + b*y <= c`.
    pub fn half_space_from_coeffs(a: f64, b: f64, c: f64) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::half_space_from_coeffs(a, b, c)?.into())
    }

    /// Creates a circle with a finite center and positive radius.
    pub fn circle(center: (f64, f64), radius: f64) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::circle(center, radius)?.into())
    }

    /// Creates an oriented rectangle from an axis-aligned base rectangle.
    pub fn rectangle(rect: impl Into<Rect>, orientation: f64) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::rectangle(rect, orientation)?.into())
    }

    /// Creates a non-degenerate triangle.
    pub fn triangle(triangle: impl Into<Triangle>) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::triangle(triangle)?.into())
    }

    /// Creates a polygon, including non-convex polygons and polygons with holes.
    pub fn polygon(polygon: impl Into<Polygon>) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::polygon(polygon)?.into())
    }

    pub fn collision_objects(&self) -> &[SimpleCollisionObject] {
        &self.collision_objects
    }

    pub fn into_collision_objects(self) -> Vec<SimpleCollisionObject> {
        self.collision_objects
    }

    /// Returns whether this object contains no geometry.
    pub fn is_empty(&self) -> bool {
        self.collision_objects.is_empty()
    }

    /// Returns whether this object occupies the full plane.
    pub fn is_full_space(&self) -> bool {
        self.collision_objects.len() == 1 && self.collision_objects[0].is_full_space()
    }

    /// Returns the union of this object and `other`.
    pub fn merge(self, other: Self) -> Self {
        Self::merge_all([self, other])
    }

    /// Returns the union of all supplied objects.
    pub fn merge_all(objects: impl IntoIterator<Item = Self>) -> Self {
        objects.into_iter().flatten().collect()
    }

    pub fn swept_areas(&self, positions: &[DPose2]) -> Vec<CollisionObject> {
        let swept_areas_by_object = self
            .collision_objects
            .iter()
            .map(|object| object.swept_areas(positions))
            .collect_vec();
        let step_count = positions.len().saturating_sub(1);
        debug_assert!(
            swept_areas_by_object
                .iter()
                .all(|areas| areas.len() == step_count)
        );
        (0..step_count)
            .map(|step| {
                CollisionObject::from(
                    swept_areas_by_object
                        .iter()
                        .map(|areas| areas[step].clone())
                        .collect_vec(),
                )
            })
            .collect()
    }

    pub fn swept_area(&self, start_pos: DPose2, end_pos: DPose2) -> CollisionObject {
        self.swept_areas(&[start_pos, end_pos])
            .pop()
            .expect("Should return exactly one area, as two positions were given.")
    }
}

impl Default for CollisionObject {
    fn default() -> Self {
        Self::empty()
    }
}

impl From<SimpleCollisionObject> for CollisionObject {
    fn from(value: SimpleCollisionObject) -> Self {
        match value {
            SimpleCollisionObject::Empty(..) => Self::empty(),
            _ => Self::new(vec![value]),
        }
    }
}

impl From<Vec<SimpleCollisionObject>> for CollisionObject {
    fn from(value: Vec<SimpleCollisionObject>) -> Self {
        value.into_iter().collect()
    }
}

impl FromIterator<SimpleCollisionObject> for CollisionObject {
    fn from_iter<T: IntoIterator<Item = SimpleCollisionObject>>(iter: T) -> Self {
        let none_if_full_space = iter
            .into_iter()
            .filter(|object| !object.is_empty())
            .map(|object| {
                if object.is_full_space() {
                    None
                } else {
                    Some(object)
                }
            })
            .collect();
        match none_if_full_space {
            Some(objects) => Self::new(objects),
            None => Self::full_space(),
        }
    }
}

impl IntoIterator for CollisionObject {
    type Item = SimpleCollisionObject;
    type IntoIter = <Vec<SimpleCollisionObject> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.collision_objects.into_iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::Rect;
    use glamx::{DPose2, DVec2};
    use rstest::{fixture, rstest};
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

    #[fixture]
    fn circle() -> SimpleCollisionObject {
        SimpleCollisionObject::circle((0.0, 0.0), 1.0).unwrap()
    }

    #[fixture]
    fn rectangle() -> SimpleCollisionObject {
        SimpleCollisionObject::rectangle(Rect::new((-1.0, -1.0), (1.0, 1.0)), 0.0).unwrap()
    }

    #[fixture]
    fn empty() -> SimpleCollisionObject {
        SimpleCollisionObject::empty()
    }

    #[fixture]
    fn full_space() -> SimpleCollisionObject {
        SimpleCollisionObject::full_space()
    }

    #[rstest]
    fn from_simple_object_filters_empty_shape(
        circle: SimpleCollisionObject,
        empty: SimpleCollisionObject,
        full_space: SimpleCollisionObject,
    ) {
        let collision_object = CollisionObject::from(circle.clone());
        assert_eq!(collision_object.collision_objects(), &[circle]);

        let empty_collision_object = CollisionObject::from(empty);
        assert!(empty_collision_object.is_empty());

        let full_space_collision_object = CollisionObject::from(full_space);
        assert!(full_space_collision_object.is_full_space());
    }

    #[rstest]
    fn from_vec_filters_empty_and_prefers_full_space(
        circle: SimpleCollisionObject,
        rectangle: SimpleCollisionObject,
        empty: SimpleCollisionObject,
        full_space: SimpleCollisionObject,
    ) {
        // Normal case
        let collision_object = CollisionObject::from(vec![circle.clone(), rectangle.clone()]);
        assert_eq!(
            collision_object.collision_objects(),
            &[circle.clone(), rectangle.clone()]
        );

        // Filter out empty objects
        let collision_object = CollisionObject::from(vec![circle.clone(), empty]);
        assert_eq!(
            collision_object.collision_objects(),
            std::slice::from_ref(&circle)
        );

        // Full space takes precedence
        let collision_object = CollisionObject::from(vec![circle, full_space, rectangle]);
        assert!(collision_object.is_full_space());
    }

    #[rstest]
    fn from_iter_filters_empty_and_prefers_full_space(
        circle: SimpleCollisionObject,
        rectangle: SimpleCollisionObject,
        empty: SimpleCollisionObject,
        full_space: SimpleCollisionObject,
    ) {
        // Normal case
        let collision_object: CollisionObject = vec![circle.clone(), rectangle.clone()]
            .into_iter()
            .collect();
        assert_eq!(
            collision_object.collision_objects(),
            &[circle.clone(), rectangle.clone()]
        );

        // Filter out empty objects
        let collision_object: CollisionObject = vec![circle.clone(), empty].into_iter().collect();
        assert_eq!(
            collision_object.collision_objects(),
            std::slice::from_ref(&circle)
        );

        // Full space takes precedence
        let collision_object: CollisionObject =
            vec![circle, full_space, rectangle].into_iter().collect();
        assert!(collision_object.is_full_space());
    }

    #[rstest]
    fn swept_areas_cover_interpolated_shape_positions(
        #[values(
            CollisionObject::from(vec![
                SimpleCollisionObject::circle((5.0, 0.0), 1.0).unwrap(),
                SimpleCollisionObject::rectangle(Rect::new((-2.0, -1.0), (2.0, 1.0)), 0.0).unwrap(),
            ]),
            CollisionObject::from(vec![
                SimpleCollisionObject::polygon(Polygon::new(
                    vec![(0.0, 0.0), (2.0, 0.0), (1.0, 1.0)].into(),
                    vec![],
                )).unwrap(),
                SimpleCollisionObject::polygon(Polygon::new(
                    vec![
                        (0.0, 0.0),
                        (2.0, 0.0),
                        (2.0, 2.0),
                        (1.0, 1.0),
                        (0.0, 2.0),
                    ]
                    .into(),
                    vec![],
                )).unwrap(),
            ]),
        )]
        shape: CollisionObject,
        #[values(&[
            DPose2::IDENTITY,
            DPose2::new(DVec2::new(10.0, 20.0), FRAC_PI_4),
            DPose2::new(DVec2::new(20.0, 40.0), FRAC_PI_2),
        ])]
        positions: &[DPose2],
    ) {
        let swept_areas = shape.swept_areas(positions);
        assert_eq!(swept_areas.len(), positions.len().saturating_sub(1));
        for ((start_pos, end_pos), swept_area) in
            positions.iter().tuple_windows().zip(swept_areas.iter())
        {
            // Interpolate 5 points between start_pos and end_pos
            for i in 0..=5 {
                let t = i as f64 / 5.0;
                let interp_pos = DPose2::from_parts(
                    start_pos.translation.lerp(end_pos.translation, t),
                    start_pos.rotation.slerp(&end_pos.rotation, t),
                );
                // Check that the swept area collides with the shape at the interpolated position
                assert!(
                    crate::collision_checker::engine::collides(
                        swept_area,
                        DPose2::IDENTITY,
                        &shape,
                        interp_pos,
                        crate::collision_checker::engine::CollisionEngine::default()
                    )
                    .unwrap()
                );
            }
        }
    }
}
