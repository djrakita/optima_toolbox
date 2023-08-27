use stl_io::IndexedMesh;
use crate::{OTriMesh, ToTriMesh};

impl ToTriMesh for IndexedMesh {
    fn to_trimesh(&self) -> OTriMesh {
        let mut triangles = vec![];

        let mut all_vertices = vec![];
        self.vertices.iter().for_each(|x| {
            all_vertices.push( [x[0] as f64, x[1] as f64, x[2] as f64] );
        });

        self.faces.iter().for_each(|x| {
            triangles.push( [ all_vertices[x.vertices[0]], all_vertices[x.vertices[1]], all_vertices[x.vertices[2]] ] );
        });

        OTriMesh { triangles }
    }
}