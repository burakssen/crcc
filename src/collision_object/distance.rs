use crate::collision_object::CollisionObject;
use crate::collision_object::simple::{SimpleCollisionObject, pose_to_affine};
use crate::error::{CrccError, CrccResult};
use geo::{AffineOps, Distance, Euclidean, Point, Polygon, Rotate};
use glamx::{DPose2, DVec2};
use std::ops::{Add, Mul, Neg, Sub};

#[derive(Debug)]
pub(crate) enum GeoRepresentation {
    Circle { center: Point<f64>, radius: f64 },
    Polygon(Polygon<f64>),
    HalfSpace { outward_normal: DVec2, offset: f64 },
    Empty,
    FullSpace,
}

/// Converts a collision object into its global geometric representation.
#[must_use]
pub(crate) fn to_geo(object: &SimpleCollisionObject, pose: DPose2) -> GeoRepresentation {
    let affine = pose_to_affine(pose);

    match object {
        SimpleCollisionObject::Empty(..) => GeoRepresentation::Empty,
        SimpleCollisionObject::FullSpace(..) => GeoRepresentation::FullSpace,

        SimpleCollisionObject::HalfSpace(half_space) => {
            let outward_normal = pose.rotation.mul(half_space.outward_normal);

            let offset = half_space.offset.add(outward_normal.dot(pose.translation));

            GeoRepresentation::HalfSpace {
                outward_normal,
                offset,
            }
        }

        SimpleCollisionObject::Circle(circle) => {
            let center = pose.mul(DVec2::from(circle.center()));

            GeoRepresentation::Circle {
                center: Point::new(center.x, center.y),
                radius: circle.radius(),
            }
        }

        SimpleCollisionObject::Rectangle(rectangle) => {
            let mut polygon = Polygon::from(*rectangle.rect());

            polygon.rotate_around_center_mut(rectangle.orientation().to_degrees());

            GeoRepresentation::Polygon(polygon.affine_transform(&affine))
        }

        SimpleCollisionObject::Triangle(triangle) => {
            let polygon = Polygon::from(**triangle);

            GeoRepresentation::Polygon(polygon.affine_transform(&affine))
        }

        SimpleCollisionObject::ConvexPolygon(polygon) => {
            GeoRepresentation::Polygon((**polygon).clone().affine_transform(&affine))
        }

        SimpleCollisionObject::NonConvexPolygon(polygon) => {
            GeoRepresentation::Polygon((**polygon).clone().affine_transform(&affine))
        }

        SimpleCollisionObject::PolygonWithHoles(polygon) => {
            GeoRepresentation::Polygon((**polygon).clone().affine_transform(&affine))
        }
    }
}

/// Calculates the distance between two geometric representations.
///
/// # Errors
///
/// Returns [`CrccError::Unsupported`] when either representation is empty.
pub(crate) fn distance_geo(left: &GeoRepresentation, right: &GeoRepresentation) -> CrccResult<f64> {
    match (left, right) {
        (GeoRepresentation::Empty, _) | (_, GeoRepresentation::Empty) => {
            Err(CrccError::Unsupported)
        }

        (GeoRepresentation::FullSpace, _) | (_, GeoRepresentation::FullSpace) => Ok(0.0),

        (
            GeoRepresentation::Circle {
                center: left_center,
                radius: left_radius,
            },
            GeoRepresentation::Circle {
                center: right_center,
                radius: right_radius,
            },
        ) => {
            let center_distance = Euclidean.distance(left_center, right_center);

            let distance = center_distance.sub(*left_radius).sub(*right_radius);

            Ok(distance.max(0.0))
        }

        (GeoRepresentation::Circle { center, radius }, GeoRepresentation::Polygon(polygon))
        | (GeoRepresentation::Polygon(polygon), GeoRepresentation::Circle { center, radius }) => {
            let center_distance = Euclidean.distance(center, polygon);

            Ok(center_distance.sub(*radius).max(0.0))
        }

        (GeoRepresentation::Polygon(left_polygon), GeoRepresentation::Polygon(right_polygon)) => {
            Ok(Euclidean.distance(left_polygon, right_polygon))
        }

        (
            GeoRepresentation::HalfSpace {
                outward_normal,
                offset,
            },
            GeoRepresentation::Circle { center, radius },
        )
        | (
            GeoRepresentation::Circle { center, radius },
            GeoRepresentation::HalfSpace {
                outward_normal,
                offset,
            },
        ) => {
            let center = DVec2::new(center.x(), center.y());
            let projection = outward_normal.dot(center);
            let boundary_distance = projection.sub(*offset);

            Ok(boundary_distance.sub(*radius).max(0.0))
        }

        (
            GeoRepresentation::HalfSpace {
                outward_normal,
                offset,
            },
            GeoRepresentation::Polygon(polygon),
        )
        | (
            GeoRepresentation::Polygon(polygon),
            GeoRepresentation::HalfSpace {
                outward_normal,
                offset,
            },
        ) => {
            let minimum_distance = polygon
                .exterior()
                .points()
                .map(|point| {
                    let point = DVec2::new(point.x(), point.y());

                    outward_normal.dot(point).sub(*offset)
                })
                .fold(f64::INFINITY, f64::min);

            Ok(minimum_distance.max(0.0))
        }

        (
            GeoRepresentation::HalfSpace {
                outward_normal: left_normal,
                offset: left_offset,
            },
            GeoRepresentation::HalfSpace {
                outward_normal: right_normal,
                offset: right_offset,
            },
        ) => {
            // Only exact parallelism permits a positive set distance.
            let normals_are_opposite = left_normal.perp_dot(*right_normal).abs() <= 0.0
                && left_normal.dot(*right_normal) < 0.0;

            if normals_are_opposite {
                let gap = (*right_offset).neg().sub(*left_offset);

                Ok(gap.max(0.0))
            } else {
                Ok(0.0)
            }
        }
    }
}

impl CollisionObject {
    /// Calculates the distance to another object at the supplied poses.
    ///
    /// # Errors
    ///
    /// Returns [`CrccError::Unsupported`] when no supported geometry pair
    /// can be evaluated.
    pub(crate) fn distance_at(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> CrccResult<f64> {
        let left = self
            .collision_objects
            .iter()
            .map(|object| to_geo(object, pos_self))
            .collect::<Vec<_>>();

        let right = other
            .collision_objects
            .iter()
            .map(|object| to_geo(object, pos_other))
            .collect::<Vec<_>>();

        let mut minimum_distance = f64::INFINITY;

        for left_geometry in &left {
            for right_geometry in &right {
                minimum_distance =
                    minimum_distance.min(distance_geo(left_geometry, right_geometry)?);

                // All supported distances are clamped to be non-negative.
                // Using `<=` avoids Clippy's floating-point equality lint.
                if minimum_distance <= 0.0 {
                    return Ok(0.0);
                }
            }
        }

        minimum_distance
            .is_finite()
            .then_some(minimum_distance)
            .ok_or(CrccError::Unsupported)
    }
}
