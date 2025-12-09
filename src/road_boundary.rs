use nalgebra::{Isometry2, Point2};
use parry2d_f64::shape::{Compound, ConvexPolygon, HalfSpace, SharedShape, TriMesh};

pub fn create_road_boundary_shape(
    convex_hull: &[(f64, f64)],
    convex_holes: &[&[(f64, f64)]],
    non_convex_holes: &[&[(f64, f64)]],
) -> Vec<Compound> {
    // Construct outer half-spaces from convex hull
    let convex_hull_polygon = ConvexPolygon::from_convex_polyline(
        convex_hull
            .iter()
            .map(|(x, y)| Point2::new(*x, *y))
            .collect(),
    )
    .unwrap();
    let outer_halfspaces = convex_hull_polygon
        .normals()
        .iter()
        .zip(convex_hull_polygon.points())
        .map(|(n, p)| (HalfSpace::new(-*n), Isometry2::translation(p.x, p.y)))
        .collect::<Vec<_>>();

    // Construct convex polygons from convex holes
    let convex_hole_polygons = convex_holes
        .iter()
        .map(|h| {
            ConvexPolygon::from_convex_polyline(
                h.iter().map(|(x, y)| Point2::new(*x, *y)).collect(),
            )
            .unwrap()
        })
        .collect::<Vec<_>>();

    // Construct compounds from non-convex holes based on trimeshes
    // let non_convex_hole_compounds = non_convex_holes
    //     .iter()
    //     .map(|h| {
    //         Compound::decompose_trimesh(
    //             &TriMesh::from_polygon(h.iter().map(|(x, y)| Point2::new(*x, *y)).collect())
    //                 .unwrap(),
    //         )
    //         .unwrap()
    //     })
    //     .collect::<Vec<_>>();
    let non_convex_hole_mesh = non_convex_holes
        .iter()
        .map(|h| {
            TriMesh::from_polygon(h.iter().map(|(x, y)| Point2::new(*x, *y)).collect()).unwrap()
        })
        .reduce(|mut acc, mesh| {
            acc.append(&mesh);
            acc
        })
        .map(|mesh| Compound::decompose_trimesh(&mesh).unwrap());

    let shared_shapes = outer_halfspaces
        .into_iter()
        .map(|(hs, iso)| (iso, SharedShape::new(hs)))
        // .chain(
        //     non_convex_hole_mesh
        //         .into_iter()
        //         .map(|mesh| (Isometry2::identity(), SharedShape::new(mesh))),
        // )
        .chain(
            convex_hole_polygons
                .into_iter()
                .map(|poly| (Isometry2::identity(), SharedShape::new(poly))),
        )
        .collect::<Vec<_>>();

    if let Some(non_convex_hole_mesh) = non_convex_hole_mesh {
        vec![Compound::new(shared_shapes), non_convex_hole_mesh]
    } else {
        vec![Compound::new(shared_shapes)]
    }
}
