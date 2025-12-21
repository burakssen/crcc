use crate::collision_checker::engine::parry::collision_object::{
    ParryCollisionObject, ParryCollisionObjectInner,
};
use crate::collision_object::CollisionObject;
use nalgebra::Isometry2;
use parry2d_f64::shape::{Compound, SharedShape, TriMesh, TriMeshBuilderError};

#[derive(Default)]
pub struct ParryEngineBuilder {
    tri_meshes: Vec<TriMesh>,
    generic_objects: Vec<(Isometry2<f64>, SharedShape)>,
}

impl ParryEngineBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_collision_object(mut self, collision_object: CollisionObject) -> Self {
        let inner = ParryCollisionObject::from(collision_object).0;
        match inner {
            ParryCollisionObjectInner::Empty => { /* intentionally left blank */ }
            ParryCollisionObjectInner::TriMesh(mesh) => {
                self.tri_meshes.push(*mesh);
            }
            ParryCollisionObjectInner::Generic { .. } => {
                self.generic_objects.push(
                    inner
                        .into_shared_shape()
                        .expect("Should not be an empty shape."),
                );
            }
        }
        self
    }

    pub fn build(self) -> super::ParryEngine {
        let tri_mesh_objects = if self.tri_meshes.is_empty() {
            None
        } else {
            let merged_trimesh =
                merge_trimeshes(self.tri_meshes).expect("Merging trimeshes should succeed");
            Some(
                Compound::decompose_trimesh(&merged_trimesh)
                    .expect("Decomposing trimesh should succeed"),
            )
        };
        let generic_objects = if self.generic_objects.is_empty() {
            None
        } else {
            Some(Compound::new(self.generic_objects))
        };
        super::ParryEngine {
            tri_mesh_objects,
            generic_objects,
        }
    }
}

fn merge_trimeshes(
    meshes: impl IntoIterator<Item = TriMesh>,
) -> Result<TriMesh, TriMeshBuilderError> {
    // parry can only merge two meshes at a time
    // calling this function repeatedly to merge all meshes can be inefficient
    // so we provide a custom implementation here
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
