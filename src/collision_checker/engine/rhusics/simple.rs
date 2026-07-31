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
use std::ops::{Add, Div, Sub};

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

    /// An infinite half-space handled analytically.
    HalfSpace(HalfSpaceComponent),
}

/// A finite shape with a primitive, relative position, and support-point logic.
#[derive(Clone)]
pub struct FiniteShape {
    pub primitive: Primitive2<f64>,
    pub position: BodyPose2<f64>,
    pub motion_radius: f64,
}

/// Analytic representation of the half-space
/// `outward_normal * point <= offset`.
#[derive(Debug, Clone, Copy)]
pub struct HalfSpaceComponent {
    pub outward_normal: DVec2,
    pub offset: f64,
}

impl RhusicsCoreSimpleCollisionObject {
    #[must_use]
    pub fn into_components(self) -> Vec<RhusicsCoreCollisionComponent> {
        match self {
            Self::Empty | Self::FullSpace => Vec::new(),
            Self::Component(component) => vec![component],
            Self::Compound(components) => components,
        }
    }
}

impl From<SimpleCollisionObject> for RhusicsCoreSimpleCollisionObject {
    fn from(collision_object: SimpleCollisionObject) -> Self {
        match &collision_object {
            SimpleCollisionObject::Empty(..) => Self::Empty,
            SimpleCollisionObject::FullSpace(..) => Self::FullSpace,

            SimpleCollisionObject::HalfSpace(half_space) => convert_half_space(half_space),

            SimpleCollisionObject::Circle(circle) => convert_circle(circle),

            SimpleCollisionObject::Rectangle(rectangle) => convert_rectangle(rectangle),

            SimpleCollisionObject::Triangle(triangle) => convert_triangle(triangle),

            SimpleCollisionObject::ConvexPolygon(polygon) => convert_convex_polygon(polygon),

            SimpleCollisionObject::NonConvexPolygon(polygon) => convert_non_convex_polygon(polygon),

            SimpleCollisionObject::PolygonWithHoles(polygon) => convert_polygon_with_holes(polygon),
        }
    }
}

/// Converts a domain [`HalfSpace`] into an analytic Rhusics component.
const fn convert_half_space(half_space: &HalfSpace) -> RhusicsCoreSimpleCollisionObject {
    RhusicsCoreSimpleCollisionObject::Component(RhusicsCoreCollisionComponent::HalfSpace(
        HalfSpaceComponent {
            outward_normal: half_space.outward_normal,
            offset: half_space.offset,
        },
    ))
}

/// Converts a domain [`Circle`] into a finite Rhusics primitive.
fn convert_circle(circle: &Circle) -> RhusicsCoreSimpleCollisionObject {
    let center = circle.center();
    let radius = circle.radius();

    RhusicsCoreSimpleCollisionObject::Component(RhusicsCoreCollisionComponent::Finite(
        FiniteShape {
            primitive: rhusics_core::collide2d::Circle::new(radius).into(),
            position: make_pose(center, 0.0),
            motion_radius: DVec2::from(center).length().add(radius),
        },
    ))
}

/// Converts a domain [`Rectangle`] into a finite Rhusics primitive.
fn convert_rectangle(rectangle: &Rectangle) -> RhusicsCoreSimpleCollisionObject {
    let width = rectangle.width();
    let height = rectangle.height();
    let center = rectangle.center();

    let half_width = width.div(2.0);
    let half_height = height.div(2.0);

    RhusicsCoreSimpleCollisionObject::Component(RhusicsCoreCollisionComponent::Finite(
        FiniteShape {
            primitive: rhusics_core::collide2d::Rectangle::new(width, height).into(),
            position: make_pose(center, rectangle.orientation()),
            motion_radius: DVec2::from(center)
                .length()
                .add(DVec2::new(half_width, half_height).length()),
        },
    ))
}

/// Converts a domain [`Triangle`] into a finite Rhusics primitive.
fn convert_triangle(triangle: &Triangle) -> RhusicsCoreSimpleCollisionObject {
    let mut vertices = [
        DVec2::new(triangle.0.x, triangle.0.y),
        DVec2::new(triangle.1.x, triangle.1.y),
        DVec2::new(triangle.2.x, triangle.2.y),
    ];

    normalize_triangle_winding(&mut vertices);

    finite_polygon(vertices)
}

/// Converts a [`ConvexPolygon`] into a finite Rhusics primitive.
fn convert_convex_polygon(convex_polygon: &ConvexPolygon) -> RhusicsCoreSimpleCollisionObject {
    let vertices = convex_polygon
        .exterior()
        .points_ccw()
        .skip(1)
        .map(|point| DVec2::new(point.x(), point.y()))
        .collect::<Vec<_>>();

    finite_polygon(vertices)
}

/// Decomposes a [`NonConvexPolygon`] into finite Rhusics components.
fn convert_non_convex_polygon(
    non_convex_polygon: &NonConvexPolygon,
) -> RhusicsCoreSimpleCollisionObject {
    RhusicsCoreSimpleCollisionObject::Compound(
        non_convex_polygon
            .earcut_triangles()
            .into_iter()
            .map(|triangle| triangle_to_component(&triangle))
            .collect(),
    )
}

/// Decomposes a [`PolygonWithHoles`] into finite Rhusics components.
fn convert_polygon_with_holes(
    polygon_with_holes: &PolygonWithHoles,
) -> RhusicsCoreSimpleCollisionObject {
    RhusicsCoreSimpleCollisionObject::Compound(
        polygon_with_holes
            .earcut_triangles()
            .into_iter()
            .map(|triangle| triangle_to_component(&triangle))
            .collect(),
    )
}

fn finite_polygon(vertices: impl IntoIterator<Item = DVec2>) -> RhusicsCoreSimpleCollisionObject {
    let vertices = vertices.into_iter().collect::<Vec<_>>();

    let primitive = RhusicsConvexPolygon::new(vertices.iter().copied().map(point).collect());

    RhusicsCoreSimpleCollisionObject::Component(RhusicsCoreCollisionComponent::Finite(
        FiniteShape {
            primitive: primitive.into(),
            position: BodyPose2::one(),
            motion_radius: motion_radius(&vertices),
        },
    ))
}

fn triangle_to_component(triangle: &geo::Triangle) -> RhusicsCoreCollisionComponent {
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
        motion_radius: motion_radius(&vertices),
    })
}

fn motion_radius(vertices: &[DVec2]) -> f64 {
    vertices
        .iter()
        .map(|vertex| vertex.length())
        .fold(0.0, f64::max)
}

fn normalize_triangle_winding(vertices: &mut [DVec2; 3]) {
    if signed_area(vertices) < 0.0 {
        vertices.swap(1, 2);
    }
}

fn signed_area(vertices: &[DVec2; 3]) -> f64 {
    vertices[1]
        .sub(vertices[0])
        .perp_dot(vertices[2].sub(vertices[0]))
        .div(2.0)
}

fn make_pose(translation: impl Into<DVec2>, rotation: f64) -> BodyPose2<f64> {
    let translation = translation.into();

    BodyPose2::new(
        Point2::new(translation.x, translation.y),
        Basis2::from_angle(Rad(rotation)),
    )
}

fn point(vector: DVec2) -> Point2<f64> {
    Point2::new(vector.x, vector.y)
}
