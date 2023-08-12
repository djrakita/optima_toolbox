use optima_file::path::{OptimaAssetLocation, OptimaStemCellPath};

pub fn get_urdf_path_from_robot_name(robot_name: &str) -> OptimaStemCellPath {
    let mut p = OptimaStemCellPath::new_asset_path();
    p.append_file_location(&OptimaAssetLocation::Robot { robot_name: robot_name.to_string() });
    let item_paths = p.get_all_items_in_directory_as_paths(false, false);
    for item_path in &item_paths {
        let extension = item_path.extension();
        if let Some(extension) = extension {
            if extension == "urdf" { return item_path.clone(); }
        }
    }
    panic!("urdf file not found for robot name {}", robot_name);
}