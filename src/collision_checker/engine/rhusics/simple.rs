use crate::collision_object::simple::{
    Circle, ConvexPolygon, HalfSpace, NonConvexPolygon, PolygonWithHoles, Rectangle,
    SimpleCollisionObject, Triangle,
};
use cgmath::{Basis2, Point2, Rad, Rotation2, Transform};
use collision::primitive::Primitive2;
use geo::{TriangulateEarcut, Winding};
use glamx::DVec2;
use rhusics_core::Pose;
use rhusics_core::collide2d::{BodyPose2, ConvexPolygon as RhusicsConvexPolygon};

pub enum RhusicsCoreSimpleCollisionObject {
    Empty,
    FullSpace,
    Component(RhusicsCoreCollisionComponent),
    Compound(Vec<RhusicsCoreCollisionComponent>),
}

/// Represents a single collision component in the Rhusics engine.
#[derive(Clone)]
pub enum RhusicsCoreCollisionComponent {
    /// A finite shape that can be handled by GJK/EPA.
    Finite(FiniteShape),
    /// An infinite half-space, handled analytically.
    HalfSpace(HalfSpaceComponent),
}

/// A finite shape with a primitive, a relative position, and support point logic.
#[derive(Clone)]
pub struct FiniteShape {
    pub primitive: Primitive2<f64>,
    pub position: BodyPose2<f64>,
    pub motion_radius: f64,
}

/// Analytic representation of a half-space: outward_normal * p <= offset.
#[derive(Debug, Clone, Copy)]
pub struct HalfSpaceComponent {
    pub outward_normal: DVec2,
    pub offset: f64,
}

impl RhusicsCoreSimpleCollisionObject {
    pub fn into_components(self) -> Vec<RhusicsCoreCollisionComponent> {
        match self {
            RhusicsCoreSimpleCollisionObject::Empty
            | RhusicsCoreSimpleCollisionObject::FullSpace => Vec::new(),
            RhusicsCoreSimpleCollisionObject::Component(component) => vec![component],
            RhusicsCoreSimpleCollisionObject::Compound(components) => components,
        }
    }
}

impl From<SimpleCollisionObject> for RhusicsCoreSimpleCollisionObject {
    fn from(collision_object: SimpleCollisionObject) -> Self {
        match collision_object {
            SimpleCollisionObject::Empty(..) => RhusicsCoreSimpleCollisionObject::Empty,
            SimpleCollisionObject::FullSpace(..) => RhusicsCoreSimpleCollisionObject::FullSpace,
            SimpleCollisionObject::HalfSpace(half_space) => convert_half_space(half_space),
            SimpleCollisionObject::Circle(circle) => convert_circle(circle),
            SimpleCollisionObject::Rectangle(rect) => convert_rectangle(rect),
            SimpleCollisionObject::Triangle(triangle) => convert_triangle(triangle),
            SimpleCollisionObject::ConvexPolygon(convex_polygon) => {
                convert_convex_polygon(convex_polygon)
            }
            SimpleCollisionObject::NonConvexPolygon(non_convex_polygon) => {
                convert_non_convex_polygon(non_convex_polygon)
            }
            SimpleCollisionObject::PolygonWithHoles(polygon_with_holes) => {
                convert_polygon_with_holes(polygon_with_holes)
            }
        }
    }
}

/// Converts a domain HalfSpace into an analytic Rhusics component.
fn convert_half_space(half_space: HalfSpace) -> RhusicsCoreSimpleCollisionObject {
    RhusicsCoreSimpleCollisionObject::Component(RhusicsCoreCollisionComponent::HalfSpace(
        HalfSpaceComponent {
            outward_normal: half_space.outward_normal,
            offset: half_space.offset,
        },
    ))
}

/// Converts a domain Circle into a Rhusics finite primitive.
fn convert_circle(circle: Circle) -> RhusicsCoreSimpleCollisionObject {
    RhusicsCoreSimpleCollisionObject::Component(RhusicsCoreCollisionComponent::Finite(
        FiniteShape {
            primitive: rhusics_core::collide2d::Circle::new(circle.radius()).into(),
            position: make_pose(circle.center(), 0.0),
            motion_radius: DVec2::from(circle.center()).length() + circle.radius(),
        },
    ))
}

/// Converts a domain Rectangle into a Rhusics finite primitive.
fn convert_rectangle(rectangle: Rectangle) -> RhusicsCoreSimpleCollisionObject {
    let half_width = rectangle.width() / 2.0;
    let half_height = rectangle.height() / 2.0;
    RhusicsCoreSimpleCollisionObject::Component(RhusicsCoreCollisionComponent::Finite(
        FiniteShape {
            primitive: rhusics_core::collide2d::Rectangle::new(
                rectangle.width(),
                rectangle.height(),
            )
            .into(),
            position: make_pose(rectangle.center(), rectangle.orientation()),
            motion_radius: DVec2::from(rectangle.center()).length()
                + DVec2::new(half_width, half_height).length(),
        },
    ))
}

fn convert_triangle(triangle: Triangle) -> RhusicsCoreSimpleCollisionObject {
    let mut vertices = [
        DVec2::new(triangle.0.x, triangle.0.y),
        DVec2::new(triangle.1.x, triangle.1.y),
        DVec2::new(triangle.2.x, triangle.2.y),
    ];
    normalize_triangle_winding(&mut vertices);

    let primitive = RhusicsConvexPolygon::new(vertices.iter().copied().map(point).collect());

    RhusicsCoreSimpleCollisionObject::Component(RhusicsCoreCollisionComponent::Finite(
        FiniteShape {
            primitive: primitive.into(),
            position: BodyPose2::one(),
            motion_radius: vertices
                .iter()
                .map(|vertex| vertex.length())
                .fold(0.0, f64::max),
        },
    ))
}

fn convert_convex_polygon(convex_polygon: ConvexPolygon) -> RhusicsCoreSimpleCollisionObject {
    let vertices: Vec<DVec2> = convex_polygon
        .exterior()
        .points_ccw()
        .skip(1)
        .map(|p| DVec2::new(p.x(), p.y()))
        .collect();

    let primitive = RhusicsConvexPolygon::new(vertices.iter().copied().map(point).collect());

    RhusicsCoreSimpleCollisionObject::Component(RhusicsCoreCollisionComponent::Finite(
        FiniteShape {
            primitive: primitive.into(),
            position: BodyPose2::one(),
            motion_radius: vertices
                .iter()
                .map(|vertex| vertex.length())
                .fold(0.0, f64::max),
        },
    ))
}

fn convert_non_convex_polygon(
    non_convex_polygon: NonConvexPolygon,
) -> RhusicsCoreSimpleCollisionObject {
    RhusicsCoreSimpleCollisionObject::Compound(
        non_convex_polygon
            .earcut_triangles()
            .into_iter()
            .map(triangle_to_component)
            .collect(),
    )
}

fn convert_polygon_with_holes(
    polygon_with_holes: PolygonWithHoles,
) -> RhusicsCoreSimpleCollisionObject {
    RhusicsCoreSimpleCollisionObject::Compound(
        polygon_with_holes
            .earcut_triangles()
            .into_iter()
            .map(triangle_to_component)
            .collect(),
    )
}

fn triangle_to_component(triangle: geo::Triangle) -> RhusicsCoreCollisionComponent {
    let mut vertices = [
        DVec2::new(triangle.0.x, triangle.0.y),
        DVec2::new(triangle.1.x, triangle.1.y),
        DVec2::new(triangle.2.x, triangle.2.y),
    ];
    normalize_triangle_winding(&mut vertices);
    let primitive = RhusicsConvexPolygon::new(vertices.iter().copied().map(point).collect());
    RhusicsCoreCollisionComponent::Finite(FiniteShape {
        primitive: primitive.into(),
        position: BodyPose2::one(),
        motion_radius: vertices
            .iter()
            .map(|vertex| vertex.length())
            .fold(0.0, f64::max),
    })
}

fn normalize_triangle_winding(vertices: &mut [DVec2; 3]) {
    if signed_area(vertices) < 0.0 {
        vertices.swap(1, 2);
    }
}

fn signed_area(vertices: &[DVec2; 3]) -> f64 {
    (vertices[1] - vertices[0]).perp_dot(vertices[2] - vertices[0]) / 2.0
}

fn make_pose(translation: impl Into<DVec2>, rotation: f64) -> BodyPose2<f64> {
    let translation = translation.into();
    BodyPose2::new(
        Point2::new(translation.x, translation.y),
        Basis2::from_angle(Rad(rotation)),
    )
}

fn point(v: DVec2) -> Point2<f64> {
    Point2::new(v.x, v.y)
}
