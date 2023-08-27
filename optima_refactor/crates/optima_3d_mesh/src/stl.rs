use stl_io::IndexedMesh;
use crate::{OTriMesh, ToTriMesh};

impl ToTriMesh for IndexedMesh {
    fn to_trimesh(&self) -> OTriMesh {
        let mut points = vec![];

        self.vertices.iter().for_each(|x| {
            points.push( [x[0] as f64, x[1] as f64, x[2] as f64] );
        });

        let mut indices = vec![];

        self.faces.iter().for_each(|x| {
           indices.push(x.vertices.clone());
        });

        OTriMesh { points, indices }
    }
}