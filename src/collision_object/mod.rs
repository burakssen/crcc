use crate::collision_object::simple::SimpleCollisionObject;
use crate::collision_object::simple::SweptArea;
use crate::error::CrccResult;
use delegate::delegate;
use geo::{Polygon, Rect, Triangle};
use glamx::{DPose2, DVec2};
use itertools::Itertools;

pub mod simple;

#[derive(Debug, Clone)]
pub struct CollisionObject {
    collision_objects: Vec<SimpleCollisionObject>,
}

impl CollisionObject {
    fn new(collision_objects: Vec<SimpleCollisionObject>) -> Self {
        Self {
            collision_objects,
        }
    }

    pub fn empty() -> Self {
        Self::new(Vec::new())
    }

    pub fn full_space() -> Self {
        Self::new(vec![SimpleCollisionObject::full_space()])
    }

    delegate! {
        #[into]
        to SimpleCollisionObject {
            pub fn half_space(outward_normal: impl Into<DVec2>, offset: f64) -> Self;
            pub fn half_space_from_points(p1: (f64, f64), p2: (f64, f64)) -> Self;
            pub fn half_space_from_coeffs(a: f64, b: f64, c: f64) -> Self;
        }
    }

    pub fn circle(center: (f64, f64), radius: f64) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::circle(center, radius)?.into())
    }

    pub fn rectangle(rect: impl Into<Rect>, orientation: f64) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::rectangle(rect, orientation)?.into())
    }

    pub fn triangle(triangle: impl Into<Triangle>) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::triangle(triangle)?.into())
    }

    pub fn polygon(polygon: impl Into<Polygon>) -> CrccResult<Self> {
        Ok(SimpleCollisionObject::polygon(polygon)?.into())
    }

    pub fn collision_objects(&self) -> &[SimpleCollisionObject] {
        &self.collision_objects
    }

    pub fn into_collision_objects(self) -> Vec<SimpleCollisionObject> {
        self.collision_objects
    }

    pub fn is_empty(&self) -> bool {
        self.collision_objects.is_empty()
    }

    pub fn is_full_space(&self) -> bool {
        self.collision_objects.len() == 1 && self.collision_objects[0].is_full_space()
    }

    pub fn merge(self, other: Self) -> Self {
        Self::merge_all([self, other])
    }

    pub fn merge_all(objects: impl IntoIterator<Item = Self>) -> Self {
        objects.into_iter().flatten().collect()
    }

    pub fn swept_areas(&self, positions: &[DPose2]) -> Vec<CollisionObject> {
        let mut swept_areas_simple = self
            .collision_objects
            .iter()
            .map(|obj| obj.swept_areas(positions))
            .collect_vec();
        let mut result = Vec::with_capacity(positions.len().saturating_sub(1));
        for _ in 0..positions.len().saturating_sub(1) {
            let swept_areas_at_i = swept_areas_simple
                .iter_mut()
                .map(|areas| {
                    areas
                        .pop()
                        .expect("There should be exactly positions.len() - 1 swept areas.")
                })
                .collect_vec();
            result.push(CollisionObject::from(swept_areas_at_i));
        }
        assert!(swept_areas_simple.iter().all(|vec| vec.is_empty()));
        // Reverse because we popped from the end, i.e., we have the result for the last position first.
        result.reverse();
        result
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
    fn from(mut value: Vec<SimpleCollisionObject>) -> Self {
        if value.iter().any(|obj| obj.is_full_space()) {
            Self::full_space()
        } else {
            value.retain(|obj| !obj.is_empty());
            Self::new(value)
        }
    }
}

impl FromIterator<SimpleCollisionObject> for CollisionObject {
    fn from_iter<T: IntoIterator<Item = SimpleCollisionObject>>(iter: T) -> Self {
        let none_if_full_space = iter
            .into_iter()
            .filter(|obj| !obj.is_empty())
            .map(|obj| if obj.is_full_space() { None } else { Some(obj) })
            .collect();
        match none_if_full_space {
            Some(objs) => Self::new(objs),
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
    fn test_from_simple(
        circle: SimpleCollisionObject,
        empty: SimpleCollisionObject,
        full_space: SimpleCollisionObject,
    ) {
        let co = CollisionObject::from(circle.clone());
        assert_eq!(co.collision_objects(), &[circle]);

        let co_empty = CollisionObject::from(empty);
        assert!(co_empty.is_empty());

        let co_full = CollisionObject::from(full_space);
        assert!(co_full.is_full_space());
    }

    #[rstest]
    fn test_from_vec(
        circle: SimpleCollisionObject,
        rectangle: SimpleCollisionObject,
        empty: SimpleCollisionObject,
        full_space: SimpleCollisionObject,
    ) {
        // Normal case
        let co = CollisionObject::from(vec![circle.clone(), rectangle.clone()]);
        assert_eq!(co.collision_objects(), &[circle.clone(), rectangle.clone()]);

        // Filter out empty objects
        let co = CollisionObject::from(vec![circle.clone(), empty]);
        assert_eq!(co.collision_objects(), std::slice::from_ref(&circle));

        // Full space takes precedence
        let co = CollisionObject::from(vec![circle, full_space, rectangle]);
        assert!(co.is_full_space());
    }

    #[rstest]
    fn test_from_iter(
        circle: SimpleCollisionObject,
        rectangle: SimpleCollisionObject,
        empty: SimpleCollisionObject,
        full_space: SimpleCollisionObject,
    ) {
        // Normal case
        let co: CollisionObject = vec![circle.clone(), rectangle.clone()]
            .into_iter()
            .collect();
        assert_eq!(co.collision_objects(), &[circle.clone(), rectangle.clone()]);

        // Filter out empty objects
        let co: CollisionObject = vec![circle.clone(), empty].into_iter().collect();
        assert_eq!(co.collision_objects(), std::slice::from_ref(&circle));

        // Full space takes precedence
        let co: CollisionObject = vec![circle, full_space, rectangle].into_iter().collect();
        assert!(co.is_full_space());
    }

    #[cfg(feature = "parry")]
    #[rstest]
    fn test_swept_areas(
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
                        &swept_area,
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
