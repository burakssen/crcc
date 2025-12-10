use nalgebra::Isometry2;
use parry2d_f64::{
    query::{Unsupported, intersection_test},
    shape::{Compound, Shape},
};

mod builder;
pub use builder::CollisionCheckerBuilder;

pub struct CollisionChecker {
    static_obstacles: Option<Compound>,
    static_obstacles_from_trimesh: Option<Compound>,
}

impl CollisionChecker {
    pub fn collides_static(
        &self,
        shape: &dyn Shape,
        position: &Isometry2<f64>,
    ) -> Result<bool, Unsupported> {
        Self::collides_multi(
            [&self.static_obstacles, &self.static_obstacles_from_trimesh],
            shape,
            position,
        )
    }

    fn collides_multi<'a>(
        obstacles: impl IntoIterator<Item = &'a Option<Compound>>,
        shape: &dyn Shape,
        position: &Isometry2<f64>,
    ) -> Result<bool, Unsupported> {
        let mut unsupported = false;
        for obs in obstacles.into_iter().flatten() {
            let collides = intersection_test(&Isometry2::identity(), obs, position, shape);
            unsupported |= matches!(collides, Err(Unsupported));
            if let Ok(true) = collides {
                return Ok(true);
            }
        }
        if unsupported {
            Err(Unsupported)
        } else {
            Ok(false)
        }
    }
}
