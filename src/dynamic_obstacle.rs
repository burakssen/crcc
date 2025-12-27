use crate::collision_object::CollisionObject;
use crate::time::TimeStep;
use nalgebra::Isometry2;

pub type DynamicObstacle = GenericDynamicObstacle<CollisionObject>;

#[derive(Clone, Debug)]
pub struct GenericDynamicObstacle<C> {
    shape: C,
    positions: Vec<Isometry2<f64>>,
    time_offset: TimeStep,
}

impl<C> GenericDynamicObstacle<C> {
    pub fn new(shape: C, positions: Vec<Isometry2<f64>>, time_offset: TimeStep) -> Self {
        Self {
            shape,
            positions,
            time_offset,
        }
    }

    pub fn shape(&self) -> &C {
        &self.shape
    }

    pub fn position_at(&self, time_step: TimeStep) -> Option<&Isometry2<f64>> {
        let with_offset = time_step - self.time_offset;
        self.positions.get(with_offset.0 as usize)
    }

    pub fn convert_repr<D>(self) -> GenericDynamicObstacle<D>
    where
        C: Into<D>,
    {
        GenericDynamicObstacle {
            shape: self.shape.into(),
            positions: self.positions,
            time_offset: self.time_offset,
        }
    }
}
