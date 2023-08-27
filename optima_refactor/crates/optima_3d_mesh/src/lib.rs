pub mod collada;
pub mod stl;

use nalgebra::{Point, Point3};
use parry3d_f64::transformation::convex_hull;
use parry3d_f64::transformation::vhacd::{VHACD, VHACDParameters};
use optima_3d_spatial::optima_3d_vec::O3DVec;
use optima_file::path::OStemCellPath;

pub trait ToTriMesh {
    fn to_trimesh(&self) -> OTriMesh;
}

/*
pub trait ToTriMesh {
    fn to_trimesh(&self) -> OTriMesh;
}
*/

/*
#[derive(Clone, Debug)]
pub struct OTriMesh {
    pub (crate) triangles: Vec<[[f64; 3]; 3]>
}
impl OTriMesh {
    pub fn triangles(&self) -> &Vec<[[f64; 3]; 3]> {
        &self.triangles
    }
}
*/

#[derive(Clone, Debug)]
pub struct OTriMesh {
    pub (crate) points: Vec<[f64;3]>,
    pub (crate) indices: Vec<[usize;3]>
}
impl OTriMesh {
    pub fn new_empty() -> Self {
        Self { points: vec![], indices: vec![] }
    }
    pub fn extend(&mut self, trimesh: &Self) {
        self.extend_from_points_and_indices(&trimesh.points, &trimesh.indices);
    }
    pub (crate) fn extend_from_points_and_indices(&mut self, new_points: &Vec<[f64; 3]>, new_indices: &Vec<[usize;3]>) {
        let points_len = self.points.len();
        let new_indices: Vec<[usize; 3]> = new_indices.iter().map(|x| [x[0] + points_len, x[1] + points_len, x[2] + points_len]).collect();
        self.points.extend(new_points);
        self.indices.extend(new_indices);
    }
    pub fn save_to_stl(&self, path: &OStemCellPath) {
        path.verify_extension(&vec!["stl", "STL"]);

        let mut mesh = vec![];

        let triangles = self.to_triangles();
        triangles.iter().for_each(|t| {
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
    pub fn to_triangles(&self) -> Vec<[[f64; 3]; 3]> {
        let mut triangles = vec![];

        self.indices.iter().for_each(|idxs| {
            let triangle = [ self.points[idxs[0]], self.points[idxs[1]], self.points[idxs[2]] ];
            triangles.push(triangle);
        });

        triangles
    }
    pub fn to_convex_hull(&self) -> OTriMesh {
        let points: Vec<Point3<f64>> = self.points.iter().map(|x| Point::from_slice(x)).collect();

        let (ch_points, ch_indices) = convex_hull(&points);

        let points: Vec<[f64; 3]> = ch_points.iter().map(|x| [x[0], x[1], x[2]] ).collect();
        let indices: Vec<[usize; 3]> = ch_indices.iter().map(|x| [ x[0] as usize, x[1] as usize, x[2] as usize]).collect();

        OTriMesh {
            points,
            indices,
        }
    }
    pub fn to_convex_decomposition(&self) -> Vec<OTriMesh> {
        let mut out = vec![];

        let points: Vec<Point3<f64>> = self.points.iter().map(|x| Point::from_slice(x)).collect();
        let indices: Vec<[u32; 3]> = self.indices.iter().map(|x| [ x[0] as u32, x[1] as u32, x[2] as u32 ]).collect();

        let params = VHACDParameters {
            max_convex_hulls: 5,
            ..Default::default()
        };
        let res = VHACD::decompose(&params, &points, &indices, true);
        let convex_hulls = res.compute_convex_hulls(5);

        convex_hulls.iter().for_each(|(ch_points, ch_indices)| {
            let points: Vec<[f64; 3]> = ch_points.iter().map(|x| [x[0], x[1], x[2]] ).collect();
            let indices: Vec<[usize; 3]> = ch_indices.iter().map(|x| [ x[0] as usize, x[1] as usize, x[2] as usize]).collect();

            out.push(OTriMesh {points, indices} );
        });

        out
    }
    #[inline(always)]
    pub fn points(&self) -> &Vec<[f64; 3]> {
        &self.points
    }
    #[inline(always)]
    pub fn indices(&self) -> &Vec<[usize; 3]> {
        &self.indices
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait SaveToSTL {
    fn save_to_stl(&self, path: &OStemCellPath);
}

/*
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
*/

impl<M: ToTriMesh> SaveToSTL for M {
    fn save_to_stl(&self, path: &OStemCellPath) {
        self.to_trimesh().save_to_stl(path);
    }
}