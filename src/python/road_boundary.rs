use geo::{LineString, Polygon};
use itertools::Itertools;
use nalgebra::{Isometry2, Vector2};
use parry2d_f64::{
    query::{DefaultQueryDispatcher, details::intersection_test_composite_shape_shape},
    shape::{Compound, Cuboid},
};
use pyo3::prelude::*;

use crate::road_boundary::create_road_boundary_obstacle;

#[pyclass]
pub struct RoadBoundaryChecker {
    road_boundary_obstacle: Vec<Compound>,
}

#[pymethods]
impl RoadBoundaryChecker {
    #[new]
    pub fn new(lanelets: Vec<Vec<(f64, f64)>>) -> Self {
        let lanelets = lanelets
            .into_iter()
            .map(|boundary| {
                Polygon::new(
                    LineString::new(boundary.into_iter().map_into().collect()),
                    vec![],
                )
            })
            .collect::<Vec<_>>();
        let road_boundary_obstacle = create_road_boundary_obstacle(&lanelets);
        RoadBoundaryChecker {
            road_boundary_obstacle,
        }
    }

    pub fn collides(&self, center: (f64, f64), orientation: f64) -> bool {
        let c = Cuboid::new(Vector2::new(2.0, 1.0));
        let iso = Isometry2::translation(center.0, center.1) * Isometry2::rotation(orientation);
        self.road_boundary_obstacle.iter().any(|obstacle| {
            intersection_test_composite_shape_shape(&DefaultQueryDispatcher, &iso, obstacle, &c)
        })
    }
}

#[pymodule]
pub(super) mod road_boundary {
    #[pymodule_export]
    use super::RoadBoundaryChecker;
}
