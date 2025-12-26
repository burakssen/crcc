use crate::collision_object::simple::SimpleCollisionObject;

pub mod simple;

#[derive(Clone, Debug)]
pub struct StaticCollisionObject(pub Vec<SimpleCollisionObject>);

impl StaticCollisionObject {
    pub fn empty() -> Self {
        Self(vec![])
    }

    pub fn merge(self, other: Self) -> Self {
        Self::merge_all([self, other])
    }

    pub fn merge_all(objects: impl IntoIterator<Item = Self>) -> Self {
        objects.into_iter().flatten().collect()
    }
}

impl Default for StaticCollisionObject {
    fn default() -> Self {
        Self::empty()
    }
}

impl From<SimpleCollisionObject> for StaticCollisionObject {
    fn from(value: SimpleCollisionObject) -> Self {
        match value {
            SimpleCollisionObject::Empty => Self::empty(),
            _ => Self(vec![value]),
        }
    }
}

impl From<Vec<SimpleCollisionObject>> for StaticCollisionObject {
    fn from(mut value: Vec<SimpleCollisionObject>) -> Self {
        value.retain(|obj| !matches!(obj, SimpleCollisionObject::Empty));
        Self(value)
    }
}

impl FromIterator<SimpleCollisionObject> for StaticCollisionObject {
    fn from_iter<T: IntoIterator<Item = SimpleCollisionObject>>(iter: T) -> Self {
        Self(
            iter.into_iter()
                .filter(|obj| !matches!(obj, SimpleCollisionObject::Empty))
                .collect(),
        )
    }
}

impl IntoIterator for StaticCollisionObject {
    type Item = SimpleCollisionObject;
    type IntoIter = <Vec<SimpleCollisionObject> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}
