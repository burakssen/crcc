use geo::Polygon;
use nalgebra::Isometry2;
use parry2d_f64::shape::{Compound, SharedShape, TriMesh, TriMeshBuilderError};

use crate::road_boundary::create_road_boundary_obstacle;

#[derive(Clone)]
pub struct CollisionCheckerBuilder {
    static_obstacles: Vec<(Isometry2<f64>, SharedShape)>,
    static_obstacles_trimeshes: Vec<TriMesh>,
}

impl CollisionCheckerBuilder {
    pub fn new() -> Self {
        CollisionCheckerBuilder {
            static_obstacles: Vec::new(),
            static_obstacles_trimeshes: Vec::new(),
        }
    }

    pub fn with_static_obstacle(
        &mut self,
        shape: SharedShape,
        position: Isometry2<f64>,
    ) -> &mut Self {
        self.static_obstacles.push((position, shape));
        self
    }

    pub fn with_static_obstacle_trimesh(&mut self, mesh: TriMesh) -> &mut Self {
        self.static_obstacles_trimeshes.push(mesh);
        self
    }

    pub fn with_road_boundary_obstacle(&mut self, lanelets: &[Polygon]) -> &mut Self {
        let (shapes, meshes) = create_road_boundary_obstacle(lanelets);
        self.static_obstacles.extend(shapes);
        self.static_obstacles_trimeshes.extend(meshes);
        self
    }

    pub fn build(self) -> super::CollisionChecker {
        let static_obstacles = if self.static_obstacles.is_empty() {
            None
        } else {
            Some(Compound::new(self.static_obstacles))
        };
        let static_obstacles_from_trimesh = if self.static_obstacles_trimeshes.is_empty() {
            None
        } else {
            let merged_trimesh = merge_trimeshes(self.static_obstacles_trimeshes)
                .expect("Merging trimeshes should succeed");
            Some(
                Compound::decompose_trimesh(&merged_trimesh)
                    .expect("Decomposing trimesh should succeed"),
            )
        };
        super::CollisionChecker {
            static_obstacles,
            static_obstacles_from_trimesh,
        }
    }
}

impl Default for CollisionCheckerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

fn merge_trimeshes(
    meshes: impl IntoIterator<Item = TriMesh>,
) -> Result<TriMesh, TriMeshBuilderError> {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut offset = 0;
    for mesh in meshes {
        let mesh_vertices = mesh.vertices();
        let mesh_indices = mesh.indices();
        vertices.extend_from_slice(mesh_vertices);
        indices.extend(
            mesh_indices
                .iter()
                .map(|[i0, i1, i2]| [i0 + offset, i1 + offset, i2 + offset]),
        );
        offset += mesh_vertices.len() as u32;
    }
    TriMesh::new(vertices, indices)
}
