use crate::collision_object::simple::SimpleCollisionObject;
use crate::collision_object::simple::SimpleCollisionObjectOps;
use glamx::DPose2;
use itertools::Itertools;

pub mod simple;

#[derive(Clone, Debug)]
pub struct CollisionObject(Vec<SimpleCollisionObject>);

impl CollisionObject {
    pub fn empty() -> Self {
        Self(vec![])
    }

    pub fn full_space() -> Self {
        Self(vec![SimpleCollisionObject::full_space()])
    }

    pub fn collision_objects(&self) -> &[SimpleCollisionObject] {
        &self.0
    }

    pub fn into_collision_objects(self) -> Vec<SimpleCollisionObject> {
        self.0
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn is_full_space(&self) -> bool {
        self.0.len() == 1 && self.0[0].is_full_space()
    }

    pub fn merge(self, other: Self) -> Self {
        Self::merge_all([self, other])
    }

    pub fn merge_all(objects: impl IntoIterator<Item = Self>) -> Self {
        objects.into_iter().flatten().collect()
    }

    pub fn swept_areas(&self, positions: &[DPose2]) -> Vec<CollisionObject> {
        let mut swept_areas_simple = self
            .0
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

    pub fn swept_area(&self, start_pos: &DPose2, end_pos: &DPose2) -> CollisionObject {
        self.swept_areas(&[*start_pos, *end_pos])
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
            _ => Self(vec![value]),
        }
    }
}

impl From<Vec<SimpleCollisionObject>> for CollisionObject {
    fn from(mut value: Vec<SimpleCollisionObject>) -> Self {
        if value.iter().any(|obj| obj.is_full_space()) {
            Self::full_space()
        } else {
            value.retain(|obj| !obj.is_empty());
            Self(value)
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
            Some(objs) => Self(objs),
            None => Self::full_space(),
        }
    }
}

impl IntoIterator for CollisionObject {
    type Item = SimpleCollisionObject;
    type IntoIter = <Vec<SimpleCollisionObject> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}
