use optima_3d_mesh::{SaveToSTL};
use optima_file::path::{OAssetLocation, OStemCellPath};

fn main() {
    let mut p = OStemCellPath::new_asset_path();
    p.append_file_location(&OAssetLocation::ChainOriginalMeshes { chain_name: "atlas" });
    p.append("r_larm.dae");
    let d = p.load_dae();

    // let trimesh = d.to_trimesh();
    // trimesh.triangles().iter().for_each(|x| println!("{:?}", x));

    let mut destination_path = OStemCellPath::new_asset_path();
    destination_path.append("test.stl");
    d.save_to_stl(&destination_path);
}
