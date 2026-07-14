use crate::collision_object::CollisionObject;
use crate::collision_object::simple::{SimpleCollisionObject, pose_to_affine};
use crate::error::CrccError;
use geo::{AffineOps, Distance, Euclidean, Point, Polygon, Rotate};
use glamx::{DPose2, DVec2};

#[derive(Debug)]
pub(crate) enum GeoRepresentation {
    Circle { center: Point<f64>, radius: f64 },
    Polygon(Polygon<f64>),
    HalfSpace { outward_normal: DVec2, offset: f64 },
    Empty,
    FullSpace,
}

pub(crate) fn to_geo(obj: &SimpleCollisionObject, pose: DPose2) -> GeoRepresentation {
    let affine = pose_to_affine(pose);
    match obj {
        SimpleCollisionObject::Empty(..) => GeoRepresentation::Empty,
        SimpleCollisionObject::FullSpace(..) => GeoRepresentation::FullSpace,
        SimpleCollisionObject::HalfSpace(hs) => {
            let normal_new = pose.rotation * hs.outward_normal;
            let offset_new = hs.offset + normal_new.dot(pose.translation);
            GeoRepresentation::HalfSpace {
                outward_normal: normal_new,
                offset: offset_new,
            }
        }
        SimpleCollisionObject::Circle(c) => {
            let global_center = pose * DVec2::from(c.center());
            GeoRepresentation::Circle {
                center: Point::new(global_center.x, global_center.y),
                radius: c.radius(),
            }
        }
        SimpleCollisionObject::Rectangle(r) => {
            let mut poly = Polygon::from(*r.rect());
            poly.rotate_around_center_mut(r.orientation().to_degrees());
            GeoRepresentation::Polygon(poly.affine_transform(&affine))
        }
        SimpleCollisionObject::Triangle(t) => {
            let poly = Polygon::from(**t);
            GeoRepresentation::Polygon(poly.affine_transform(&affine))
        }
        SimpleCollisionObject::ConvexPolygon(cp) => {
            GeoRepresentation::Polygon((**cp).clone().affine_transform(&affine))
        }
        SimpleCollisionObject::NonConvexPolygon(ncp) => {
            GeoRepresentation::Polygon((**ncp).clone().affine_transform(&affine))
        }
        SimpleCollisionObject::PolygonWithHoles(pwh) => {
            GeoRepresentation::Polygon((**pwh).clone().affine_transform(&affine))
        }
    }
}

pub(crate) fn distance_geo(
    g1: &GeoRepresentation,
    g2: &GeoRepresentation,
) -> Result<f64, CrccError> {
    match (g1, g2) {
        (GeoRepresentation::Empty, _) | (_, GeoRepresentation::Empty) => {
            Err(CrccError::Unsupported)
        }
        (GeoRepresentation::FullSpace, _) | (_, GeoRepresentation::FullSpace) => Ok(0.0),
        (
            GeoRepresentation::Circle {
                center: c1,
                radius: r1,
            },
            GeoRepresentation::Circle {
                center: c2,
                radius: r2,
            },
        ) => {
            let d = Euclidean.distance(c1, c2);
            Ok((d - r1 - r2).max(0.0))
        }
        (GeoRepresentation::Circle { center, radius }, GeoRepresentation::Polygon(poly))
        | (GeoRepresentation::Polygon(poly), GeoRepresentation::Circle { center, radius }) => {
            let d = Euclidean.distance(center, poly);
            Ok((d - radius).max(0.0))
        }
        (GeoRepresentation::Polygon(p1), GeoRepresentation::Polygon(p2)) => {
            Ok(Euclidean.distance(p1, p2))
        }
        (
            GeoRepresentation::HalfSpace {
                outward_normal: n,
                offset,
            },
            GeoRepresentation::Circle { center, radius },
        )
        | (
            GeoRepresentation::Circle { center, radius },
            GeoRepresentation::HalfSpace {
                outward_normal: n,
                offset,
            },
        ) => {
            let center_vec = DVec2::new(center.x(), center.y());
            let proj = n.dot(center_vec);
            let d = proj - offset;
            Ok((d - radius).max(0.0))
        }
        (
            GeoRepresentation::HalfSpace {
                outward_normal: n,
                offset,
            },
            GeoRepresentation::Polygon(poly),
        )
        | (
            GeoRepresentation::Polygon(poly),
            GeoRepresentation::HalfSpace {
                outward_normal: n,
                offset,
            },
        ) => {
            let mut min_d = f64::INFINITY;
            for p in poly.exterior().points() {
                let p_vec = DVec2::new(p.x(), p.y());
                let proj = n.dot(p_vec);
                let d = proj - offset;
                min_d = min_d.min(d);
            }
            Ok(min_d.max(0.0))
        }
        (
            GeoRepresentation::HalfSpace {
                outward_normal: n1,
                offset: o1,
            },
            GeoRepresentation::HalfSpace {
                outward_normal: n2,
                offset: o2,
            },
        ) => {
            let dot = n1.dot(*n2);
            if (dot + 1.0).abs() < 1e-9 {
                let gap = -o2 - o1;
                Ok(gap.max(0.0))
            } else {
                Ok(0.0)
            }
        }
    }
}

impl CollisionObject {
    pub(crate) fn distance_at(
        &self,
        pos_self: DPose2,
        other: &Self,
        pos_other: DPose2,
    ) -> Result<f64, CrccError> {
        let left: Vec<_> = self
            .collision_objects
            .iter()
            .map(|object| to_geo(object, pos_self))
            .collect();
        let right: Vec<_> = other
            .collision_objects
            .iter()
            .map(|object| to_geo(object, pos_other))
            .collect();
        let mut min_distance = f64::INFINITY;
        for geo_self in &left {
            for geo_other in &right {
                min_distance = min_distance.min(distance_geo(geo_self, geo_other)?);
                // ponytail: exact zero cannot improve, so skip the remaining pairs.
                if min_distance == 0.0 {
                    return Ok(0.0);
                }
            }
        }
        if min_distance.is_finite() {
            Ok(min_distance)
        } else {
            Err(CrccError::Unsupported)
        }
    }
}
