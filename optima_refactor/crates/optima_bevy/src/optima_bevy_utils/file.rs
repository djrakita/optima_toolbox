use std::path::PathBuf;
use optima_file::path::OStemCellPath;

pub fn get_asset_path_str_from_ostemcellpath(p: &OStemCellPath) -> String {
    let mut path_buf_back_to_optima_assets = PathBuf::new();
    path_buf_back_to_optima_assets.push("../");
    path_buf_back_to_optima_assets.push("../");
    // path_buf_back_to_optima_assets.push("../");
    // path_buf_back_to_optima_assets.push("../");
    let string_components = p.split_path_into_string_components_back_to_given_dir("optima_toolbox");

    let mut path_buf_from_optima_toolbox = PathBuf::new();
    for c in &string_components { path_buf_from_optima_toolbox.push(c); }

    let combined_path_buf = path_buf_back_to_optima_assets.join(path_buf_from_optima_toolbox);

    return combined_path_buf.to_str().expect("error").to_string();
}