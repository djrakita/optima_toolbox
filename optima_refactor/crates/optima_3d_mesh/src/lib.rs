pub mod collada;
pub mod stl;

use optima_3d_spatial::optima_3d_vec::O3DVec;
use optima_file::path::OStemCellPath;

pub trait ToTriMesh {
    fn to_trimesh(&self) -> OTriMesh;
}

#[derive(Clone, Debug)]
pub struct OTriMesh {
    pub (crate) triangles: Vec<[[f64; 3]; 3]>
}
impl OTriMesh {
    pub fn triangles(&self) -> &Vec<[[f64; 3]; 3]> {
        &self.triangles
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait SaveToSTL {
    fn save_to_stl(&self, path: &OStemCellPath);
}

impl<M: ToTriMesh> SaveToSTL for M {
    fn save_to_stl(&self, path: &OStemCellPath) {
        let ext = path.extension().expect("must have extension");
        assert!(ext.as_str() == "stl" || ext.as_str() == "STL");

        let trimesh = self.to_trimesh();

        let mut mesh = vec![];
        trimesh.triangles.iter().for_each(|t| {
            let v1 = t[0];
            let v2 = t[1];
            let v3 = t[2];

            let a = v2.sub(&v1);
            let b = v3.sub(&v2);
            let n = a.cross(&b);

            let normal = stl_io::Normal::new([n[0] as f32, n[1] as f32, n[2] as f32]);

            let v1 = stl_io::Vertex::new( [v1[0] as f32, v1[1] as f32, v1[2] as f32]  );
            let v2 = stl_io::Vertex::new( [v2[0] as f32, v2[1] as f32, v2[2] as f32]  );
            let v3 = stl_io::Vertex::new( [v3[0] as f32, v3[1] as f32, v3[2] as f32]  );

            let triangle = stl_io::Triangle{ normal, vertices: [v1, v2, v3] };
            mesh.push(triangle);
        });

        let mut f = path.get_file_for_writing();
        stl_io::write_stl(&mut f, mesh.iter()).expect("could not write stl");
    }
}