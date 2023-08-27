use optima_file::path::{OAssetLocation, OStemCellPath};

pub fn get_urdf_path_from_chain_name(chain_name: &str) -> OStemCellPath {
    let mut p = OStemCellPath::new_asset_path();
    // p.append_file_location(&OAssetLocation::Robot { robot_name: robot_name.to_string() });
    p.append_file_location(&OAssetLocation::Chain { chain_name });
    let item_paths = p.get_all_items_in_directory_as_paths(false, false);
    for item_path in &item_paths {
        let extension = item_path.extension();
        if let Some(extension) = extension {
            if extension == "urdf" { return item_path.clone(); }
        }
    }
    panic!("urdf file not found for chain name {}", chain_name);
}