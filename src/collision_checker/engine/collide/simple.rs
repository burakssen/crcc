use crate::collision_object::simple::{
    Circle, ConvexPolygon, HalfSpace, NonConvexPolygon, PolygonWithHoles, Rectangle,
    SimpleCollisionObject, Triangle,
};
use collide::Bounded;
use collide_convex::Convex as CollideConvex;
use collide_sphere::Sphere as CollideSphere;
use geo::{TriangulateEarcut, Winding};
use glamx::{DPose2, DVec2};
use simple_vectors::Vector;

pub type CollideVec2 = Vector<f64, 2>;

pub enum CollideSimpleCollisionObject {
    Empty,
    FullSpace,
    Component(CollideCollisionComponent),
    Compound(Vec<CollideCollisionComponent>),
}

#[derive(Clone)]
pub enum CollideCollisionComponent {
    Finite(FiniteShape),
    HalfSpace(HalfSpaceComponent),
}

#[derive(Clone)]
pub struct FiniteShape {
    pub collider: CollideConvex<CollideVec2>,
    pub bounding_sphere: CollideSphere<CollideVec2>,
    pub position: DPose2,
    pub support: FiniteShapeSupport,
}

#[derive(Clone)]
pub enum FiniteShapeSupport {
    Circle { radius: f64 },
    Vertices(Vec<DVec2>),
}

#[derive(Debug, Clone, Copy)]
pub struct HalfSpaceComponent {
    pub outward_normal: DVec2,
    pub offset: f64,
}

impl CollideSimpleCollisionObject {
    pub fn into_components(self) -> Vec<CollideCollisionComponent> {
        match self {
            CollideSimpleCollisionObject::Empty | CollideSimpleCollisionObject::FullSpace => {
                Vec::new()
            }
            CollideSimpleCollisionObject::Component(component) => vec![component],
            CollideSimpleCollisionObject::Compound(components) => components,
        }
    }
}

impl From<SimpleCollisionObject> for CollideSimpleCollisionObject {
    fn from(collision_object: SimpleCollisionObject) -> Self {
        match collision_object {
            SimpleCollisionObject::Empty(..) => CollideSimpleCollisionObject::Empty,
            SimpleCollisionObject::FullSpace(..) => CollideSimpleCollisionObject::FullSpace,
            SimpleCollisionObject::HalfSpace(half_space) => convert_half_space(half_space),
            SimpleCollisionObject::Circle(circle) => convert_circle(circle),
            SimpleCollisionObject::Rectangle(rectangle) => convert_rectangle(rectangle),
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

fn convert_half_space(half_space: HalfSpace) -> CollideSimpleCollisionObject {
    CollideSimpleCollisionObject::Component(CollideCollisionComponent::HalfSpace(
        HalfSpaceComponent {
            outward_normal: half_space.outward_normal,
            offset: half_space.offset,
        },
    ))
}

fn convert_circle(circle: Circle) -> CollideSimpleCollisionObject {
    let collider = CollideConvex::sphere(vec2(DVec2::ZERO), circle.radius());
    CollideSimpleCollisionObject::Component(CollideCollisionComponent::Finite(FiniteShape {
        collider,
        bounding_sphere: CollideSphere::new(vec2(DVec2::ZERO), circle.radius()),
        position: make_pose(circle.center(), 0.0),
        support: FiniteShapeSupport::Circle {
            radius: circle.radius(),
        },
    }))
}

fn convert_rectangle(rectangle: Rectangle) -> CollideSimpleCollisionObject {
    let half_width = rectangle.width() / 2.0;
    let half_height = rectangle.height() / 2.0;
    let vertices = vec![
        DVec2::new(-half_width, -half_height),
        DVec2::new(half_width, -half_height),
        DVec2::new(half_width, half_height),
        DVec2::new(-half_width, half_height),
    ];
    let collider = collide_convex_from_vertices(&vertices);
    CollideSimpleCollisionObject::Component(CollideCollisionComponent::Finite(FiniteShape {
        bounding_sphere: collider.bounding_volume(),
        collider,
        position: make_pose(rectangle.center(), rectangle.orientation()),
        support: FiniteShapeSupport::Vertices(vertices),
    }))
}

fn convert_triangle(triangle: Triangle) -> CollideSimpleCollisionObject {
    let mut vertices = [
        DVec2::new(triangle.0.x, triangle.0.y),
        DVec2::new(triangle.1.x, triangle.1.y),
        DVec2::new(triangle.2.x, triangle.2.y),
    ];
    normalize_triangle_winding(&mut vertices);
    finite_polygon_component(vertices.into())
}

fn convert_convex_polygon(convex_polygon: ConvexPolygon) -> CollideSimpleCollisionObject {
    let vertices = convex_polygon
        .exterior()
        .points_ccw()
        .skip(1)
        .map(|p| DVec2::new(p.x(), p.y()))
        .collect();
    finite_polygon_component(vertices)
}

fn convert_non_convex_polygon(
    non_convex_polygon: NonConvexPolygon,
) -> CollideSimpleCollisionObject {
    CollideSimpleCollisionObject::Compound(
        non_convex_polygon
            .earcut_triangles()
            .into_iter()
            .map(triangle_to_component)
            .collect(),
    )
}

fn convert_polygon_with_holes(
    polygon_with_holes: PolygonWithHoles,
) -> CollideSimpleCollisionObject {
    CollideSimpleCollisionObject::Compound(
        polygon_with_holes
            .earcut_triangles()
            .into_iter()
            .map(triangle_to_component)
            .collect(),
    )
}

fn finite_polygon_component(vertices: Vec<DVec2>) -> CollideSimpleCollisionObject {
    CollideSimpleCollisionObject::Component(triangle_or_polygon_to_component(vertices))
}

fn triangle_to_component(triangle: geo::Triangle) -> CollideCollisionComponent {
    let mut vertices = [
        DVec2::new(triangle.0.x, triangle.0.y),
        DVec2::new(triangle.1.x, triangle.1.y),
        DVec2::new(triangle.2.x, triangle.2.y),
    ];
    normalize_triangle_winding(&mut vertices);
    triangle_or_polygon_to_component(vertices.into())
}

fn triangle_or_polygon_to_component(vertices: Vec<DVec2>) -> CollideCollisionComponent {
    let collider = collide_convex_from_vertices(&vertices);
    CollideCollisionComponent::Finite(FiniteShape {
        bounding_sphere: collider.bounding_volume(),
        collider,
        position: DPose2::IDENTITY,
        support: FiniteShapeSupport::Vertices(vertices),
    })
}

fn collide_convex_from_vertices(vertices: &[DVec2]) -> CollideConvex<CollideVec2> {
    CollideConvex::new(0.0, vertices.iter().copied().map(vec2).collect())
}

fn normalize_triangle_winding(vertices: &mut [DVec2; 3]) {
    if signed_area(vertices) < 0.0 {
        vertices.swap(1, 2);
    }
}

fn signed_area(vertices: &[DVec2; 3]) -> f64 {
    (vertices[1] - vertices[0]).perp_dot(vertices[2] - vertices[0]) / 2.0
}

fn make_pose(translation: impl Into<DVec2>, rotation: f64) -> DPose2 {
    DPose2::new(translation.into(), rotation)
}

pub fn vec2(v: impl Into<DVec2>) -> CollideVec2 {
    let v = v.into();
    Vector::new([v.x, v.y])
}
