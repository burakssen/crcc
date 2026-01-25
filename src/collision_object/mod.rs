use crate::collision_object::simple::SimpleCollisionObject;
use crate::collision_object::simple::SimpleCollisionObjectOps;
use cfg_if::cfg_if;
use glamx::DPose2;
use itertools::Itertools;

cfg_if!(
    if #[cfg(feature = "default-engine")] {
        use crate::collision_checker::engine::default::DefaultEngineCollisionObject;
        use crate::collision_checker::CollisionCheckerError;
        use crate::collision_checker::engine::EngineCollisionObject;
        use std::sync::OnceLock;
    }
);

pub mod simple;

#[derive(Debug, Clone)]
pub struct CollisionObject {
    collision_objects: Vec<SimpleCollisionObject>,
    #[cfg(feature = "default-engine")]
    engine_collision_object: OnceLock<DefaultEngineCollisionObject>,
}

impl CollisionObject {
    fn new(collision_objects: Vec<SimpleCollisionObject>) -> Self {
        Self {
            collision_objects,
            #[cfg(feature = "default-engine")]
            engine_collision_object: OnceLock::new(),
        }
    }

    pub fn empty() -> Self {
        Self::new(Vec::new())
    }

    pub fn full_space() -> Self {
        Self::new(vec![SimpleCollisionObject::full_space()])
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

#[cfg(feature = "default-engine")]
impl CollisionObject {
    fn engine_collision_object(&self) -> &DefaultEngineCollisionObject {
        self.engine_collision_object
            .get_or_init(|| DefaultEngineCollisionObject::from(self.clone()))
    }
}

#[cfg(feature = "default-engine")]
impl EngineCollisionObject for CollisionObject {
    fn collides_at(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<bool, CollisionCheckerError> {
        self.engine_collision_object().collides_at(
            pos_self,
            other.engine_collision_object(),
            pos_other,
        )
    }

    fn collides_continuous(
        &self,
        start_pos_self: DPose2,
        end_pos_self: DPose2,
        other: &Self,
        start_pos_other: DPose2,
        end_pos_other: DPose2,
    ) -> Result<bool, CollisionCheckerError> {
        self.engine_collision_object().collides_continuous(
            start_pos_self,
            end_pos_self,
            other.engine_collision_object(),
            start_pos_other,
            end_pos_other,
        )
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
