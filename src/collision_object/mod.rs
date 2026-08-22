use crate::collision_checker::engine::{self, CollisionEngine};
use crate::collision_object::simple::SimpleCollisionObject;
use crate::collision_object::simple::SweptArea;
use crate::error::CrccResult;
use geo::{Polygon, Rect, Triangle};
use glamx::{DPose2, DVec2};

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
    ///
    /// # Errors
    ///
    /// Returns an error if the selected collision engine cannot process the
    /// supplied shapes or if the underlying collision query fails.
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
    ///
    /// # Errors
    ///
    /// Returns an error if the selected collision engine cannot process the
    /// supplied shapes or if the underlying continuous query fails.
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
    /// # Errors
    ///
    /// Returns an error when the selected engine does not support the supplied
    /// shape combination or when the underlying distance query fails.
    pub fn distance(
        &self,
        other: &Self,
        pos_self: DPose2,
        pos_other: DPose2,
        engine: CollisionEngine,
    ) -> CrccResult<f64> {
        engine::distance(self, pos_self, other, pos_other, engine)
    }

    const fn new(collision_objects: Vec<SimpleCollisionObject>) -> Self {
        Self { collision_objects }
    }

    /// Creates an object that never collides.
    #[must_use]
    pub const fn empty() -> Self {
        Self::new(Vec::new())
    }

    /// Creates an object occupying the entire plane.
    #[must_use]
    pub fn full_space() -> Self {
        Self::new(vec![SimpleCollisionObject::full_space()])
    }

    /// Creates the half-space `normal · point <= offset`.
    ///
    /// # Errors
    ///
    /// Returns an error if the normal or offset does not define a valid
    /// half-space.
    pub fn half_space(outward_normal: impl Into<DVec2>, offset: f64) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::half_space(outward_normal, offset)?.into())
    }

    /// Creates the half-space to the right of the directed line `p1 -> p2`.
    ///
    /// # Errors
    ///
    /// Returns an error if either point is invalid or the two points do not
    /// define a valid directed line.
    pub fn half_space_from_points(p1: (f64, f64), p2: (f64, f64)) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::half_space_from_points(p1, p2)?.into())
    }

    /// Creates the half-space `a*x + b*y <= c`.
    ///
    /// # Errors
    ///
    /// Returns an error if the coefficients are non-finite or do not define a
    /// valid half-space.
    pub fn half_space_from_coeffs(a: f64, b: f64, c: f64) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::half_space_from_coeffs(a, b, c)?.into())
    }

    /// Creates a circle with a finite center and positive radius.
    ///
    /// # Errors
    ///
    /// Returns an error if the center is non-finite or the radius is not
    /// positive and finite.
    pub fn circle(center: (f64, f64), radius: f64) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::circle(center, radius)?.into())
    }

    /// Creates an oriented rectangle from an axis-aligned base rectangle.
    ///
    /// # Errors
    ///
    /// Returns an error if the rectangle or orientation does not define valid
    /// finite geometry.
    pub fn rectangle(rect: impl Into<Rect>, orientation: f64) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::rectangle(rect, orientation)?.into())
    }

    /// Creates a non-degenerate triangle.
    ///
    /// # Errors
    ///
    /// Returns an error if the triangle contains invalid coordinates or is
    /// degenerate.
    pub fn triangle(triangle: impl Into<Triangle>) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::triangle(triangle)?.into())
    }

    /// Creates a polygon, including non-convex polygons and polygons with holes.
    ///
    /// # Errors
    ///
    /// Returns an error if the polygon is invalid, degenerate, or cannot be
    /// decomposed into supported collision geometry.
    pub fn polygon(polygon: impl Into<Polygon>) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::polygon(polygon)?.into())
    }

    /// Returns the simple objects forming this compound object.
    #[must_use]
    pub fn collision_objects(&self) -> &[SimpleCollisionObject] {
        &self.collision_objects
    }

    /// Consumes this object and returns its simple collision objects.
    #[must_use]
    pub fn into_collision_objects(self) -> Vec<SimpleCollisionObject> {
        self.collision_objects
    }

    /// Returns whether this object contains no geometry.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.collision_objects.is_empty()
    }

    /// Returns whether this object occupies the full plane.
    #[must_use]
    pub const fn is_full_space(&self) -> bool {
        matches!(
            self.collision_objects.as_slice(),
            [object] if object.is_full_space()
        )
    }

    /// Returns the union of this object and `other`.
    #[must_use]
    pub fn merge(self, other: Self) -> Self {
        Self::merge_all([self, other])
    }

    /// Returns the union of all supplied objects.
    #[must_use]
    pub fn merge_all(objects: impl IntoIterator<Item = Self>) -> Self {
        objects.into_iter().flatten().collect()
    }

    /// Computes one swept area for each consecutive pair of poses.
    #[must_use]
    pub fn swept_areas(&self, positions: &[DPose2]) -> Vec<Self> {
        let step_count = positions.len().saturating_sub(1);
        let object_count = self.collision_objects.len();

        let mut areas_by_step = (0..step_count)
            .map(|_| Vec::with_capacity(object_count))
            .collect::<Vec<_>>();

        for object_areas in self
            .collision_objects
            .iter()
            .map(|object| object.swept_areas(positions))
        {
            debug_assert_eq!(object_areas.len(), step_count);

            for (step_areas, area) in areas_by_step.iter_mut().zip(object_areas) {
                step_areas.push(area);
            }
        }

        areas_by_step.into_iter().map(Self::from).collect()
    }

    /// Computes the swept area between two poses.
    ///
    /// Returns `None` if no swept-area interval is produced.
    #[must_use]
    pub fn swept_area(&self, start_pos: DPose2, end_pos: DPose2) -> Option<Self> {
        self.swept_areas(&[start_pos, end_pos]).pop()
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
    fn from_iter<T>(iter: T) -> Self
    where
        T: IntoIterator<Item = SimpleCollisionObject>,
    {
        iter.into_iter()
            .filter(|object| !object.is_empty())
            .map(|object| (!object.is_full_space()).then_some(object))
            .collect::<Option<Vec<_>>>()
            .map_or_else(Self::full_space, Self::new)
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
#[allow(clippy::panic_in_result_fn)]
mod tests {
    use super::*;
    use crate::error::CrccError;
    use geo::{BoundingRect, Rect};
    use glamx::DPose2;
    use rstest::{fixture, rstest};

    #[fixture]
    fn circle() -> CrccResult<SimpleCollisionObject> {
        SimpleCollisionObject::circle((0.0, 0.0), 1.0)
    }

    #[fixture]
    fn rectangle() -> CrccResult<SimpleCollisionObject> {
        SimpleCollisionObject::rectangle(Rect::new((-1.0, -1.0), (1.0, 1.0)), 0.0)
    }

    #[fixture]
    fn empty() -> SimpleCollisionObject {
        SimpleCollisionObject::empty()
    }

    #[fixture]
    fn full_space() -> SimpleCollisionObject {
        SimpleCollisionObject::full_space()
    }

    fn primitive_compound() -> CrccResult<CollisionObject> {
        Ok(CollisionObject::from(vec![
            SimpleCollisionObject::circle((5.0, 0.0), 1.0)?,
            SimpleCollisionObject::rectangle(Rect::new((-2.0, -1.0), (2.0, 1.0)), 0.0)?,
        ]))
    }

    #[rstest]
    fn from_simple_object_filters_empty_shape(
        circle: CrccResult<SimpleCollisionObject>,
        empty: SimpleCollisionObject,
        full_space: SimpleCollisionObject,
    ) -> CrccResult<()> {
        let circle = circle?;

        let collision_object = CollisionObject::from(circle.clone());

        assert_eq!(
            collision_object.collision_objects(),
            std::slice::from_ref(&circle),
        );

        let empty_collision_object = CollisionObject::from(empty);

        assert!(empty_collision_object.is_empty());

        let full_space_collision_object = CollisionObject::from(full_space);

        assert!(full_space_collision_object.is_full_space());

        Ok(())
    }

    #[rstest]
    fn from_vec_filters_empty_and_prefers_full_space(
        circle: CrccResult<SimpleCollisionObject>,
        rectangle: CrccResult<SimpleCollisionObject>,
        empty: SimpleCollisionObject,
        full_space: SimpleCollisionObject,
    ) -> CrccResult<()> {
        let circle = circle?;
        let rectangle = rectangle?;

        let collision_object = CollisionObject::from(vec![circle.clone(), rectangle.clone()]);

        assert_eq!(
            collision_object.collision_objects(),
            &[circle.clone(), rectangle.clone()],
        );

        let collision_object = CollisionObject::from(vec![circle.clone(), empty]);

        assert_eq!(
            collision_object.collision_objects(),
            std::slice::from_ref(&circle),
        );

        let collision_object = CollisionObject::from(vec![circle, full_space, rectangle]);

        assert!(collision_object.is_full_space());

        Ok(())
    }

    #[rstest]
    fn from_iter_filters_empty_and_prefers_full_space(
        circle: CrccResult<SimpleCollisionObject>,
        rectangle: CrccResult<SimpleCollisionObject>,
        empty: SimpleCollisionObject,
        full_space: SimpleCollisionObject,
    ) -> CrccResult<()> {
        let circle = circle?;
        let rectangle = rectangle?;

        let collision_object = vec![circle.clone(), rectangle.clone()]
            .into_iter()
            .collect::<CollisionObject>();

        assert_eq!(
            collision_object.collision_objects(),
            &[circle.clone(), rectangle.clone()],
        );

        let collision_object = vec![circle.clone(), empty]
            .into_iter()
            .collect::<CollisionObject>();

        assert_eq!(
            collision_object.collision_objects(),
            std::slice::from_ref(&circle),
        );

        let collision_object = vec![circle, full_space, rectangle]
            .into_iter()
            .collect::<CollisionObject>();

        assert!(collision_object.is_full_space());

        Ok(())
    }

    #[test]
    fn swept_area_preserves_compound_extrema() -> CrccResult<()> {
        let shape = primitive_compound()?;
        let swept_area = shape
            .swept_area(DPose2::IDENTITY, DPose2::translation(2.0, 3.0))
            .ok_or(CrccError::InvalidGeometry("test expected one swept area"))?;
        let [
            SimpleCollisionObject::Rectangle(circle_bound),
            SimpleCollisionObject::ConvexPolygon(rectangle_bound),
        ] = swept_area.collision_objects()
        else {
            return Err(CrccError::InvalidGeometry(
                "test expected circle and rectangle swept bounds",
            ));
        };

        assert_eq!(circle_bound.rect(), &Rect::new((4.0, -1.0), (8.0, 4.0)));
        assert_eq!(
            rectangle_bound.bounding_rect(),
            Some(Rect::new((-2.0, -1.0), (4.0, 4.0))),
        );
        Ok(())
    }
}
