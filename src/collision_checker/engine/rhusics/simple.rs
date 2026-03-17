use crate::collision_object::simple::{
    Circle, ConvexPolygon, HalfSpace, NonConvexPolygon, PolygonWithHoles, Rectangle,
    SimpleCollisionObject, Triangle,
};
use cgmath::Point2;
use collision::primitive::Primitive2;
use geo::TriangulateEarcut;
use glamx::{DPose2, DVec2};
use rhusics_core::collide2d::ConvexPolygon as RhusicsConvexPolygon;

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
    pub position: DPose2,
    pub support: FiniteShapeSupport,
}

/// Support point implementation data for finite shapes.
#[derive(Clone)]
pub enum FiniteShapeSupport {
    Circle { radius: f64 },
    Vertices(Vec<DVec2>),
}

/// Analytic representation of a half-space: outward_normal * p <= offset.
#[derive(Debug, Clone, Copy)]
pub struct HalfSpaceComponent {
    pub outward_normal: DVec2,
    pub offset: f64,
}

// --- Conversion Logic ---

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
            support: FiniteShapeSupport::Circle {
                radius: circle.radius(),
            },
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
            support: FiniteShapeSupport::Vertices(vec![
                DVec2::new(-half_width, -half_height),
                DVec2::new(half_width, -half_height),
                DVec2::new(half_width, half_height),
                DVec2::new(-half_width, half_height),
            ]),
        },
    ))
}

fn convert_triangle(triangle: Triangle) -> RhusicsCoreSimpleCollisionObject {
    let vertices = [
        DVec2::new(triangle.0.x, triangle.0.y),
        DVec2::new(triangle.1.x, triangle.1.y),
        DVec2::new(triangle.2.x, triangle.2.y),
    ];

    let primitive = RhusicsConvexPolygon::new(vertices.iter().copied().map(point).collect());

    RhusicsCoreSimpleCollisionObject::Component(RhusicsCoreCollisionComponent::Finite(
        FiniteShape {
            primitive: primitive.into(),
            position: DPose2::IDENTITY,
            support: FiniteShapeSupport::Vertices(vertices.into()),
        },
    ))
}

fn convert_convex_polygon(convex_polygon: ConvexPolygon) -> RhusicsCoreSimpleCollisionObject {
    let mut vertices: Vec<DVec2> = convex_polygon
        .exterior()
        .coords()
        .map(|c| DVec2::new(c.x, c.y))
        .collect();

    if vertices.len() > 1 && vertices.first() == vertices.last() {
        vertices.pop();
    }

    let primitive = RhusicsConvexPolygon::new(vertices.iter().copied().map(point).collect());

    RhusicsCoreSimpleCollisionObject::Component(RhusicsCoreCollisionComponent::Finite(
        FiniteShape {
            primitive: primitive.into(),
            position: DPose2::IDENTITY,
            support: FiniteShapeSupport::Vertices(vertices),
        },
    ))
}

fn convert_non_convex_polygon(
    non_convex_polygon: NonConvexPolygon,
) -> RhusicsCoreSimpleCollisionObject {
    let triangles = non_convex_polygon
        .exterior()
        .triangles()
        .collect::<Vec<_>>();

    let mut shapes = Vec::new();
    for triangle in triangles {
        let vertices = [
            DVec2::new(triangle.0.x, triangle.0.y),
            DVec2::new(triangle.1.x, triangle.1.y),
            DVec2::new(triangle.2.x, triangle.2.y),
        ];
        let primitive = RhusicsConvexPolygon::new(vertices.iter().copied().map(point).collect());
        shapes.push(RhusicsCoreCollisionComponent::Finite(FiniteShape {
            primitive: primitive.into(),
            position: DPose2::IDENTITY,
            support: FiniteShapeSupport::Vertices(vertices.into()),
        }));
    }

    RhusicsCoreSimpleCollisionObject::Compound(shapes)
}

fn convert_polygon_with_holes(
    polygon_with_holes: PolygonWithHoles,
) -> RhusicsCoreSimpleCollisionObject {
    let triangles = polygon_with_holes.earcut_triangles();

    let mut shapes = Vec::new();
    for triangle in triangles {
        let vertices = [
            DVec2::new(triangle.0.x, triangle.0.y),
            DVec2::new(triangle.1.x, triangle.1.y),
            DVec2::new(triangle.2.x, triangle.2.y),
        ];
        let primitive = RhusicsConvexPolygon::new(vertices.iter().copied().map(point).collect());
        shapes.push(RhusicsCoreCollisionComponent::Finite(FiniteShape {
            primitive: primitive.into(),
            position: DPose2::IDENTITY,
            support: FiniteShapeSupport::Vertices(vertices.into()),
        }));
    }

    RhusicsCoreSimpleCollisionObject::Compound(shapes)
}

fn make_pose(translation: impl Into<DVec2>, rotation: f64) -> DPose2 {
    DPose2::new(translation.into(), rotation)
}

fn point(v: DVec2) -> Point2<f64> {
    Point2::new(v.x, v.y)
}
