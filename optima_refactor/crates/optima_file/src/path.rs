use std::fs;
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::path::PathBuf;
use std::str::FromStr;
use dae_parser::Document;
use vfs::*;
#[cfg(not(feature = "do_not_embed_assets"))]
use rust_embed::RustEmbed;
use serde::{Serialize, Deserialize, Serializer, Deserializer};
use serde::de::DeserializeOwned;
use stl_io::IndexedMesh;
use walkdir::WalkDir;
use crate::traits::{ToJsonString};
use optima_console::output::{oprint_full, PrintColor, PrintMode};
use urdf_rs::Robot;

/// excludes have higher priority than includes.  Includes work based on union of sets, so if you use
/// even one include, you must then include everything else you want too.
#[cfg(not(feature = "do_not_embed_assets"))]
#[derive(RustEmbed, Debug)]
#[folder = "../../../optima_assets"]
#[exclude="*.DS_Store"]
#[cfg_attr(feature = "exclude_all_robot_assets", exclude = "optima_robots/*")]
#[cfg_attr(feature = "exclude_all_robot_meshes", exclude = "*/meshes/*")]
#[cfg_attr(feature = "exclude_all_robot_meshes", exclude = "*/input_meshes/*")]
#[cfg_attr(feature = "exclude_all_robot_meshes", exclude = "*/glb_meshes/*")]
#[cfg_attr(feature = "exclude_optima_scenes", exclude = "optima_scenes/*")]
#[cfg_attr(feature = "exclude_file_io", exclude = "file_io/*")]
#[cfg_attr(feature = "exclude_optima_robot_sets", exclude = "optima_robot_sets/*")]
#[cfg_attr(feature = "include_ur5", include = "*/ur5/*")]
#[cfg_attr(feature = "include_sawyer", include = "*/sawyer/*")]
#[cfg_attr(feature = "include_fetch", include = "*/fetch/*")]
#[cfg_attr(feature = "include_hubo", include = "*/hubo/*")]
#[cfg_attr(feature = "include_optima_scenes", include = "optima_scenes/*")]
#[cfg_attr(feature = "include_file_io", include = "file_io/*")]
#[cfg_attr(feature = "include_optima_robot_sets", include = "optima_robot_sets/*")]
struct AssetEmbed;

/// An `OptimaStemCellPath` has the same functionality as an `OptimaPath`, but it
/// will try to automatically select whether it should use a physical or virtual file path based on
/// your target (rust executable, web-assembly, python module, etc).  When in doubt, use this over
/// an `OptimaPath`.
///
/// # Example
///```
/// use optima_file::path::{OAssetLocation, OStemCellPath};
///
/// let mut p = OStemCellPath::new_asset_path().expect("error");
/// p.append_file_location(&OAssetLocation::RobotMeshes {robot_name: "ur5".to_string()});
/// p.append("0.dae");
///
///```
#[derive(Clone, Debug)]
pub struct OStemCellPath {
    optima_file_paths: Vec<OPath>
}
impl OStemCellPath {
    pub fn new_asset_path() -> Self {
        let mut optima_file_paths = vec![];

        let mut error_strings = vec![];

        if cfg!(target_arch = "wasm32") || cfg!(feature = "only_use_embedded_assets") {
            #[cfg(not(feature = "do_not_embed_assets"))]
            let p_res = OPath::new_asset_virtual_path();
            #[cfg(not(feature = "do_not_embed_assets"))]
            if let Ok(p) = &p_res { optima_file_paths.push(p.clone()); }
            #[cfg(not(feature = "do_not_embed_assets"))]
            if let Err(p) = &p_res { error_strings.push(p.clone()); }
        } else if cfg!(feature = "do_not_embed_assets") {
            let p_res = OPath::new_asset_physical_path_from_json_file();
            if let Ok(p) = &p_res { optima_file_paths.push(p.clone()); }
            if let Err(p) = &p_res { error_strings.push(p.clone()); }
        } else {
            let p_res1 = OPath::new_asset_physical_path_from_json_file();
            if let Ok(p) = &p_res1 { optima_file_paths.push(p.clone()); }
            if let Err(p) = &p_res1 { error_strings.push(p.clone()) }
            #[cfg(not(feature = "do_not_embed_assets"))]
            let p_res2 = OPath::new_asset_virtual_path();
            #[cfg(not(feature = "do_not_embed_assets"))]
            if let Ok(p) = &p_res2 { optima_file_paths.push(p.clone()); }
            #[cfg(not(feature = "do_not_embed_assets"))]
            if let Err(p) = &p_res2 { error_strings.push(p.clone()); }
        }

        if optima_file_paths.len() == 0 {
            panic!("OptimaStemCellPath has zero valid paths for the following reasons: \n{:?}", error_strings);
        }

        Self {
            optima_file_paths
        }
    }
    pub fn new_asset_path_from_string_components(components: &Vec<String>) -> Self {
        let mut out_path = Self::new_asset_path();
        for s in components { out_path.append(s); }
        out_path
    }
    pub fn append(&mut self, s: &str) {
        for p in &mut self.optima_file_paths { p.append(s); }
    }
    pub fn append_vec(&mut self, v: &Vec<String>) {
        for p in &mut self.optima_file_paths {
            p.append_vec(v);
        }
    }
    pub fn append_file_location(&mut self, location: &OAssetLocation) {
        for p in &mut self.optima_file_paths {
            p.append_file_location(location);
        }
    }
    pub fn read_file_contents_to_string(&self) -> String {
        self.try_function_on_all_optima_file_paths(OPath::read_file_contents_to_string, "read_file_contents_to_string")
    }
    pub fn write_string_to_file(&self, s: &String) {
        self.try_function_on_all_optima_file_paths_with_one_param(OPath::write_string_to_file, s, "write_string_to_file")
    }
    pub fn exists(&self) -> bool {
        return self.optima_file_paths[0].exists();
    }
    pub fn get_file_for_writing(&self) -> File {
        self.try_function_on_all_optima_file_paths(OPath::get_file_for_writing, "get_file_for_writing")
    }
    pub fn to_string(&self) -> String {
        return self.optima_file_paths[0].to_string();
    }
    pub fn filename(&self) -> Option<String> {
        return self.optima_file_paths[0].filename();
    }
    pub fn filename_without_extension(&self) -> Option<String> {
        return self.optima_file_paths[0].filename_without_extension();
    }
    pub fn extension(&self) -> Option<String> {
        return self.optima_file_paths[0].extension();
    }
    pub fn set_extension(&mut self, extension: &str) {
        for p in &mut self.optima_file_paths {
            p.set_extension(extension);
        }
    }
    pub fn split_path_into_string_components(&self) -> Vec<String> {
        return self.optima_file_paths[0].split_path_into_string_components();
    }
    pub fn split_path_into_string_components_back_to_assets_dir(&self) -> Vec<String> {
        return self.optima_file_paths[0].split_path_into_string_components_back_to_asset_dir();
    }
    pub fn split_path_into_string_components_back_to_given_dir(&self, dir: &str) -> Vec<String> {
        return self.optima_file_paths[0].split_path_into_string_components_back_to_given_dir(dir);
    }
    pub fn delete_file(&self) {
        self.try_function_on_all_optima_file_paths(OPath::delete_file, "delete_file")
    }
    pub fn delete_all_items_in_directory(&self) {
        self.try_function_on_all_optima_file_paths(OPath::delete_all_items_in_directory, "delete_all_items_in_directory")
    }
    pub fn copy_file_to_destination(&self, destination: &OPath) {
        self.try_function_on_all_optima_file_paths_with_one_param(OPath::copy_file_to_destination, destination, "copy_file_to_destination")
    }
    pub fn verify_extension(&self, extensions: &Vec<&str>) {
        self.try_function_on_all_optima_file_paths_with_one_param(OPath::verify_extension, extensions, "verify_extension")
    }
    pub fn get_all_items_in_directory(&self, include_directories: bool, include_hidden_files: bool) -> Vec<String> {
        for p in &self.optima_file_paths {
            let items = p.get_all_items_in_directory(include_directories, include_hidden_files);
            if items.len() > 0 { return items; }
        }
        return vec![];
    }
    pub fn get_all_directories_in_directory(&self) -> Vec<String> {
        for p in &self.optima_file_paths {
            let items = p.get_all_directories_in_directory();
            if items.len() > 0 { return items; }
        }
        return vec![];
    }
    pub fn get_all_items_in_directory_as_paths(&self, include_directories: bool, include_hidden_files: bool) -> Vec<OStemCellPath> {
        let mut out = vec![];

        let mut tmp_paths = vec![];

        for optima_file_path in &self.optima_file_paths {
            let paths = optima_file_path.get_all_items_in_directory_as_paths(include_directories, include_hidden_files);
            tmp_paths.push(paths);
        }

        let num_items = tmp_paths[0].len();
        for item_idx in 0..num_items {
            let mut paths = vec![];
            for path_type_idx in 0..tmp_paths.len() {
                paths.push( tmp_paths[path_type_idx][item_idx].clone() )
            }
            out.push(OStemCellPath { optima_file_paths: paths })
        }
        
        out
    }
    pub fn get_all_directories_in_directory_as_paths(&self) -> Vec<OStemCellPath> {
        let mut out = vec![];

        let mut tmp_paths = vec![];

        for optima_file_path in &self.optima_file_paths {
            let paths = optima_file_path.get_all_directories_in_directory_as_paths();
            tmp_paths.push(paths);
        }

        let num_items = tmp_paths[0].len();
        for item_idx in 0..num_items {
            let mut paths = vec![];
            for path_type_idx in 0..tmp_paths.len() {
                paths.push( tmp_paths[path_type_idx][item_idx].clone() )
            }
            out.push(OStemCellPath { optima_file_paths: paths })
        }

        out
    }
    pub fn save_object_to_file_as_json<T: Serialize + DeserializeOwned>(&self, object: &T) {
        self.try_function_on_all_optima_file_paths_with_one_param(OPath::save_object_to_file_as_json, object, "save_object_to_file_as_json")
    }
    pub fn load_object_from_json_file<T: DeserializeOwned>(&self) -> T {
        self.try_function_on_all_optima_file_paths(OPath::load_object_from_json_file, "load_object_from_json_file")
    }
    pub fn walk_directory_and_match(&self, pattern: OPathMatchingPattern, stop_condition: OPathMatchingStopCondition) -> Vec<OPath> {
        for p in &self.optima_file_paths {
            let res = p.walk_directory_and_match(pattern.clone(), stop_condition.clone());
            if res.len() > 0 { return res; }
        }
        return vec![];
    }
    pub fn try_function_on_all_optima_file_paths<T>(&self, f: fn(&OPath) -> Result<T, String>, function_name: &str) -> T {
        let mut error_strings = vec![];
        for p in &self.optima_file_paths {
            let res = f(p);
            match res {
                Ok(a) => { return a }
                Err(s) => { error_strings.push(s) }
            }
        }
        panic!("No valid optima_path in function {:?} with error strings {:?}", function_name, error_strings);
    }
    pub fn try_function_on_all_optima_file_paths_with_one_param<T, P>(&self, f: fn(&OPath, &P) -> Result<T, String>, param: &P, function_name: &str) -> T {
        let mut error_strings = vec![];
        for p in &self.optima_file_paths {
            let res = f(p, param);
            match res {
                Ok(a) => { return a }
                Err(s) => { error_strings.push(s) }
            }
        }
        panic!("No valid optima_path in function {:?} with error strings {:?}", function_name, error_strings);
    }
    pub fn optima_file_paths(&self) -> &Vec<OPath> {
        &self.optima_file_paths
    }
    pub fn as_physical_path(&self) -> &OPath {
        for x in self.optima_file_paths() {
            match x {
                OPath::Path(_) => { return x; }
                OPath::VfsPath(_) => {}
            }
        }
        panic!("physical path not found");
    }
    pub fn as_virtual_path(&self) -> &OPath {
        for x in self.optima_file_paths() {
            match x {
                OPath::Path(_) => { }
                OPath::VfsPath(_) => { return x; }
            }
        }
        panic!("virtual path not found");
    }
}
impl OStemCellPath {
    pub fn load_urdf(&self) -> Robot {
        return self.try_function_on_all_optima_file_paths(OPath::load_urdf, "load_urdf");
    }
    /*
    pub fn load_dae(&self) -> Scene {
        return self.try_function_on_all_optima_file_paths(OPath::load_dae, "load_dae");
    }
    */
    pub fn load_dae(&self) -> Document {
        return self.try_function_on_all_optima_file_paths(OPath::load_dae, "load_dae");
    }
    pub fn load_stl(&self) -> IndexedMesh {
        return self.try_function_on_all_optima_file_paths(OPath::load_stl, "load_stl");
    }
}

impl Serialize for OStemCellPath {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let paths_as_strings: Vec<String> = self.split_path_into_string_components();
        paths_as_strings.serialize(serializer)
    }
}
impl<'de> Deserialize<'de> for OStemCellPath {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let paths_as_strings: Vec<String> = Vec::deserialize(deserializer)?;
        Ok(OStemCellPath::new_asset_path_from_string_components(&paths_as_strings))
    }
}

/// An `OptimaPath` is an object used to load files and write to files in the Optima library.
/// The object is designed to be cross-platform and flexible such that it can either write to or
/// read from the physical harddrive on the user's computer or read from a virtual file system that
/// is embedded into the library binary itself.  When in doubt, use the `OptimaStemCellPath` instead of
/// this object as it has the same underlying functionality, but it will try to automatically select
/// whether it should use a physical or virtual file path based on your target (rust executable, web-assembly,
/// python module, etc).
///
/// # Example
/// ```text
/// use optima::utils::utils_files::optima_path::OptimaAssetLocation;
/// use optima::utils::utils_files::optima_path::OptimaPath;
///
/// // Initializes an OptimaPath linking to the embedded virtual path.
/// let mut p = OptimaPath::new_asset_virtual_path().expect("error");
/// p.append_file_location(&OptimaAssetLocation::RobotInputMeshes {robot_name: "ur5".to_string()});
/// p.append("0.dae");
/// let d = p.load_dae();
/// println!("{:?}", d);
/// ```
/// Note that the virtual file system (VfsPath) variant does not support any
/// writing operations; if tried, an `UnsupportedOperationError` will be returned.  Also, in order
/// for the VFS options to work, the --no-default-features flag must be used at compile time (because
/// the do_not_embed_assets feature is on by default).
#[derive(Clone, Debug)]
pub enum OPath {
    Path(PathBuf),
    VfsPath(VfsPath)
}
impl OPath {
    pub fn new_home_path() -> Self {
        if cfg!(target_arch = "wasm32") { panic!("Not supported by wasm32.") }

        Self::Path(dirs::home_dir().unwrap().to_path_buf())
    }
    pub fn new_asset_physical_path_from_json_file() -> Result<Self, String> {
        if cfg!(target_arch = "wasm32") { return Err("Not supported by wasm32.".to_string()) }

        let mut check_path = Self::new_home_path();
        check_path.append(".optima_asset_path.JSON");
        if check_path.exists() {
            let path_to_assets_dir_res = check_path.load_object_from_json_file::<PathToAssetsDir>();
            return match path_to_assets_dir_res {
                Ok(path_to_asset_dir) => {
                    Ok(Self::Path(path_to_asset_dir.path_to_assets_dir))
                }
                Err(_) => {
                    let found = Self::auto_create_optima_asset_path_json_file();
                    if !found { Err("optima_asset folder not found on computer.".to_string()) } else { Self::new_asset_physical_path_from_json_file() }
                }
            }
        } else {
            let mut check_path = Self::new_home_path();
            check_path.append(".optima_asset_path.lock");
            if check_path.exists() {
                // panic!("optima_assets folder not found on computer.  This was indicated by the .optima_asset_path.lock file.");
                return Err("optima_assets folder not found on computer.  This was indicated by the .optima_asset_path.lock file.".to_string())
            }
            let found = Self::auto_create_optima_asset_path_json_file();
            return if !found {
                // panic!("optima_asset folder not found on computer.")
                Err("optima_asset folder not found on computer.".to_string())
            } else { Self::new_asset_physical_path_from_json_file() }
        }
    }
    #[cfg(not(feature = "do_not_embed_assets"))]
    pub fn new_asset_virtual_path() -> Result<Self, String> {
        let e: EmbeddedFS<AssetEmbed> = EmbeddedFS::new();
        let root_path = VfsPath::new(e);
        return Ok(Self::VfsPath(root_path));
    }
    pub fn new_asset_physical_path_from_string_components(components: &Vec<String>) -> Self {
        if cfg!(target_arch = "wasm32") { panic!("Not supported by wasm32.") }

        let mut path = Self::new_asset_physical_path_from_json_file().expect("error");
        for s in components { path.append(s); }

        return path;
    }
    #[cfg(not(feature = "do_not_embed_assets"))]
    pub fn new_asset_virtual_path_from_string_components(components: &Vec<String>) -> Self {
        let mut path = Self::new_asset_virtual_path().expect("error");
        for s in components { path.append(s); }

        return path;
    }
    pub fn append(&mut self, s: &str) {
        if s == "" { return; }
        match self {
            OPath::Path(p) => { p.push(s); }
            OPath::VfsPath(p) => { *p = p.join(s).expect("error"); }
        }
    }
    pub fn append_vec(&mut self, v: &Vec<String>) {
        for s in v {
            self.append(s);
        }
    }
    pub fn append_file_location(&mut self, location: &OAssetLocation) {
        let v = location.get_path_wrt_asset_folder();
        match self {
            OPath::Path(p) => {
                for s in v { p.push(s); }
            }
            OPath::VfsPath(p) => {
                for s in v { *p = p.join(s).expect("error"); }
            }
        }
    }
    pub fn read_file_contents_to_string(&self) -> Result<String, String> {
        return match self {
            OPath::Path(p) => {
                let mut file_res = File::open(p);
                return match &mut file_res {
                    Ok(f) => {
                        let mut contents = String::new();
                        let res = f.read_to_string(&mut contents);
                        if res.is_err() {
                            // panic!("Could not read file contents to string for path {:?}", self);
                            return Err(format!("Could not read file contents to string for path {:?}", self));
                        }
                        Ok(contents)
                    }
                    Err(e) => {
                        // panic!("{:?}", e);
                        Err(e.to_string())
                    }
                }
            }
            OPath::VfsPath(p) => {
                let mut content = String::new();

                let mut seek_and_read_res = p.open_file();
                match &mut seek_and_read_res {
                    Ok(seek_and_read) => {
                        seek_and_read.read_to_string(&mut content).expect("error");
                        Ok(content)
                    }
                    Err(e) => {
                        // panic!("{:?}", e);
                        Err(e.to_string())
                    }
                }
            }
        }
    }
    pub fn write_string_to_file(&self, s: &String) -> Result<(), String> {
        match self {
            OPath::Path(p) => {
                let parent_option = p.parent();
                match parent_option {
                    None => {
                        panic!("Could not get parent of path in save_object_to_file_as_json.");
                    }
                    Some(parent) => {
                        fs::create_dir_all(parent).expect("error");
                    }
                }

                if p.exists() { fs::remove_file(p).expect("error"); }

                let mut file_res = OpenOptions::new()
                    .write(true)
                    .create(true)
                    .open(p);

                match &mut file_res {
                    Ok(f) => {
                        f.write(s.as_bytes()).expect("error");
                        Ok(())
                    }
                    Err(e) => {
                        // panic!("{:?}", e);
                        Err(e.to_string())
                    }
                }
            }
            OPath::VfsPath(_) => {
                // panic!("Writing is not supported by VfsPath.  Try using a Path variant instead.");
                Err("Writing is not supported by VfsPath.  Try using a Path variant instead.".to_string())
            }
        }
    }
    pub fn exists(&self) -> bool {
        return match self {
            OPath::Path(p) => { p.exists() }
            OPath::VfsPath(p) => { p.exists().expect("error") }
        }
    }
    pub fn get_file_for_writing(&self) -> Result<File, String> {
        return match self {
            OPath::Path(p) => {
                let prefix = p.parent().unwrap();
                fs::create_dir_all(prefix).unwrap();
                if p.exists() { fs::remove_file(p).expect("error"); }
                let file = OpenOptions::new().write(true).create_new(true).open(p).expect("error");
                Ok(file)
            }
            OPath::VfsPath(_) => {
                // panic!("Cannot get file for writing from VfsPath.");
                Err("Cannot get file for writing from VfsPath.".to_string())
            }
        }
    }
    pub fn to_string(&self) -> String {
        return match self {
            OPath::Path(p) => {
                p.to_str().unwrap().to_string()
            }
            OPath::VfsPath(p) => {
                p.as_str().to_string()
            }
        }
    }
    pub fn filename(&self) -> Option<String> {
        return match self {
            OPath::Path(p) => {
                let f = p.file_name();
                match f {
                    None => { None }
                    Some(ff) => { Some(ff.to_str().unwrap().to_string()) }
                }
            }
            OPath::VfsPath(p) => {
                let f = p.filename();
                Some(f)
            }
        }
    }
    pub fn filename_without_extension(&self) -> Option<String> {
        let f = self.filename();
        return match f {
            None => { None }
            Some(ff) => {
                let split: Vec<&str> = ff.split(".").collect();
                Some(split[0].to_string())
            }
        }
    }
    pub fn extension(&self) -> Option<String> {
        return match self {
            OPath::Path(p) => {
                let ext_option = p.extension();
                match ext_option {
                    None => { None }
                    Some(ext) => { Some(ext.to_str().unwrap().to_string()) }
                }
            }
            OPath::VfsPath(p) => {
                let filename = p.filename();
                let split: Vec<&str> = filename.split(".").collect();
                if split.len() <= 1 { None } else { Some(split[split.len() - 1].to_string()) }
            }
        }
    }
    pub fn set_extension(&mut self, extension: &str) {
        match self {
            OPath::Path(p) => {
                p.set_extension(extension);
            }
            OPath::VfsPath(p) => {
                let parent_path = p.parent();
                let filename = parent_path.filename();
                let split: Vec<&str> = filename.split(".").collect();
                let mut new_filename = split[0].to_string();
                if extension != "" {
                    new_filename += format!(".{}", extension).as_str();
                }
                *p = parent_path.join(new_filename.as_str()).expect("error");
            }
        }
    }
    pub fn save_object_to_file_as_json<T: Serialize + DeserializeOwned>(&self, object: &T) -> Result<(), String> {
        return match self {
            OPath::Path(_) => {
                let s = object.to_json_string();
                self.write_string_to_file(&s).expect("error");
                Ok(())
            }
            OPath::VfsPath(_) => {
                // panic!("Writing is not supported by VfsPath.  Try using a Path variant instead.");
                Err("Writing is not supported by VfsPath.  Try using a Path variant instead.".to_string())
            }
        }
    }
    pub fn load_object_from_json_file<T: DeserializeOwned>(&self) -> Result<T, String> {
        let contents = self.read_file_contents_to_string()?;
        return load_object_from_json_string::<T>(&contents);
    }
    pub fn walk_directory_and_match(&self, pattern: OPathMatchingPattern, stop_condition: OPathMatchingStopCondition) -> Vec<OPath> {
        let mut out_vec = vec![];

        match self {
            OPath::Path(p) => {
                for entry_res in WalkDir::new(p) {
                    if let Ok(entry) = entry_res {
                        let entry_path = entry.path().to_path_buf();
                        let mut optima_path = Self::Path(entry_path);
                        let matched = Self::directory_walk_standard_entry(&mut optima_path, &mut out_vec, &pattern);
                        if matched {
                            match stop_condition {
                                OPathMatchingStopCondition::First => {
                                    return out_vec;
                                }
                                OPathMatchingStopCondition::All => {}
                            }
                        }
                    }
                }
            }
            OPath::VfsPath(p) => {
                let it_res = p.walk_dir();
                if let Ok(it) = it_res {
                    for entry_res in it {
                        if let Ok(entry) = entry_res {
                            let mut optima_path = Self::VfsPath(entry.clone());
                            let matched = Self::directory_walk_standard_entry(&mut optima_path, &mut out_vec, &pattern);
                            if matched {
                                match stop_condition {
                                    OPathMatchingStopCondition::First => {
                                        return out_vec;
                                    }
                                    OPathMatchingStopCondition::All => {}
                                }
                            }
                        }
                    }
                }
            }
        }

        out_vec
    }
    pub fn split_path_into_string_components(&self) -> Vec<String> {
        let mut out_vec = vec![];

        match self {
            OPath::Path(p) => {
                let mut par = p.clone();
                loop {
                    let filename_option = par.file_name();
                    if let Some(filename) = filename_option {
                        out_vec.insert(0, filename.to_str().unwrap().to_string());
                    }
                    let par_option = par.parent();
                    if let Some(par_some) = par_option {
                        par = par_some.to_path_buf();
                    } else {
                        return out_vec;
                    }
                }
            }
            OPath::VfsPath(p) => {
                let mut par = p.clone();
                loop {
                    let filename = par.filename();
                    out_vec.insert(0, filename);
                    par = par.parent();
                }
            }
        };
    }
    pub fn split_path_into_string_components_back_to_asset_dir(&self) -> Vec<String> {
        return match self {
            OPath::Path(_) => {
                let string_components = self.split_path_into_string_components();
                let mut optima_assets_idx: Option<usize> = None;
                for (i, s) in string_components.iter().enumerate() {
                    if s == "optima_assets" {
                        optima_assets_idx = Some(i);
                        break;
                    }
                }

                if optima_assets_idx.is_none() {
                    // return Err(OptimaError::new_generic_error_str(&format!("optima_assets was not found on the given path {:?}", self), file!(), line!()));
                    panic!("optima_assets was not found on the given path {:?}", self);
                }

                let optima_assets_idx = optima_assets_idx.unwrap();
                let num_string_components = string_components.len();

                if optima_assets_idx == num_string_components - 1 { return vec![]; }

                let mut out_vec = vec![];
                for i in optima_assets_idx + 1..num_string_components {
                    out_vec.push(string_components[i].clone());
                }

                out_vec
            }
            OPath::VfsPath(_) => {
                self.split_path_into_string_components()
            }
        }
    }
    pub fn split_path_into_string_components_back_to_given_dir(&self, dir: &str) -> Vec<String> {
        return match self {
            OPath::Path(_) => {
                let string_components = self.split_path_into_string_components();
                let mut optima_assets_idx: Option<usize> = None;
                for (i, s) in string_components.iter().enumerate() {
                    if s == dir {
                        optima_assets_idx = Some(i);
                        break;
                    }
                }

                if optima_assets_idx.is_none() {
                    // return Err(OptimaError::new_generic_error_str(&format!("dir {:?} was not found on the given path {:?}",dir, self), file!(), line!()));
                    panic!("dir {:?} was not found on the given path {:?}",dir, self);
                }

                let optima_assets_idx = optima_assets_idx.unwrap();
                let num_string_components = string_components.len();

                if optima_assets_idx == num_string_components - 1 { return vec![]; }

                let mut out_vec = vec![];
                for i in optima_assets_idx + 1..num_string_components {
                    out_vec.push(string_components[i].clone());
                }

                out_vec
            }
            OPath::VfsPath(_) => {
                self.split_path_into_string_components()
            }
        }
    }
    pub fn delete_file(&self) -> Result<(), String> {
        match self {
            OPath::Path(p) => {
                fs::remove_file(p).expect("error");
                Ok(())
            }
            OPath::VfsPath(_) => {
                // panic!("VfsPath does not support deleting files.");
                Err("VfsPath does not support deleting files.".to_string())
            }
        }
    }
    pub fn delete_all_items_in_directory(&self) -> Result<(), String> {
        match self {
            OPath::Path(p) => {
                fs::remove_dir_all(p).expect("error");
                fs::create_dir(p).expect("error");
                Ok(())
            }
            OPath::VfsPath(_) => {
                // panic!("VfsPath does not support deleting files.");
                Err("VfsPath does not support deleting files.".to_string())
            }
        }
    }
    pub fn copy_file_to_destination(&self, destination: &OPath) -> Result<(), String> {
        if !self.exists() {
            // panic!("Tried to copy file {:?} but it does not exist!", self);
            return Err(format!("Tried to copy file {:?} but it does not exist!", self));
        }
        match self {
            OPath::Path(p) => {
                match destination {
                    OPath::Path(p2) => {
                        if !p2.exists() {
                            let par = p2.parent().unwrap();
                            fs::create_dir_all(par).expect("error");
                        }
                        fs::copy(p, p2).expect("error");
                        Ok(())
                    }
                    OPath::VfsPath(_) => {
                        // panic!("VfsPath does not support copying files.");
                        Err("VfsPath does not support copying files.".to_string())
                    }
                }
            }
            OPath::VfsPath(_) => {
                // panic!("VfsPath does not support copying files.")
                Err("VfsPath does not support copying files.".to_string())
            }
        }
    }
    pub fn verify_extension(&self, extensions: &Vec<&str>) -> Result<(), String> {
        let ext_option = self.extension();
        match ext_option {
            None => {
                // panic!("Path {:?} does not have one of the following extensions: {:?} ", self, extensions);
                return Err(format!("Path {:?} does not have one of the following extensions: {:?}", self, extensions))
            }
            Some(ext) => {
                for e in extensions {
                    if e == &ext {
                        return Ok(());
                    }
                }
            }
        }
        // panic!("Path {:?} does not have one of the following extensions: {:?}", self, extensions);
        Err(format!("Path {:?} does not have one of the following extensions: {:?}", self, extensions))
    }
    pub fn get_all_items_in_directory(&self, include_directories: bool, include_hidden_files: bool) -> Vec<String> {
        let mut out_vec = vec![];

        match self {
            OPath::Path(p) => {
                let res = p.read_dir();
                if let Ok(read_dir) = res {
                    for dir_entry_res in read_dir {
                        if let Ok(dir_entry) = dir_entry_res {
                            let filename = dir_entry.file_name();
                            if include_directories || dir_entry.path().is_file() {
                                let f = filename.to_str().unwrap().to_string();
                                if !(f.chars().nth(0).unwrap().to_string() == ".") || include_hidden_files {
                                    out_vec.push(filename.to_str().unwrap().to_string());
                                }
                            }
                        }
                    }
                }
            }
            OPath::VfsPath(p) => {
                let res = p.read_dir();
                if let Ok(read_dir) = res {
                    for i in read_dir {
                        if include_directories || i.is_file().unwrap() {
                            let f = i.filename();
                            if !(f.chars().nth(0).unwrap().to_string() == ".") || include_hidden_files {
                                out_vec.push(i.filename());
                            }
                        }
                    }
                }
            }
        }

        out_vec
    }
    pub fn get_all_directories_in_directory(&self) -> Vec<String> {
        let mut out_vec = vec![];

        match self {
            OPath::Path(p) => {
                let res = p.read_dir();
                if let Ok(read_dir) = res {
                    for dir_entry_res in read_dir {
                        if let Ok(dir_entry) = dir_entry_res {
                            if dir_entry.path().is_dir() {
                                let filename = dir_entry.file_name();
                                let filename_string = filename.to_str().unwrap().to_string();
                                out_vec.push(filename_string);
                            }
                        }
                    }
                }
            }
            OPath::VfsPath(p) => {
                let res = p.read_dir();
                if let Ok(read_dir) = res {
                    for i in read_dir {
                        if i.is_dir().unwrap() {
                            out_vec.push(i.filename());
                        }
                    }
                }
            }
        }

        out_vec
    }
    pub fn get_all_items_in_directory_as_paths(&self, include_directories: bool, include_hidden_files: bool) -> Vec<OPath> {
        let mut out = vec![];

        let items = self.get_all_items_in_directory(include_directories, include_hidden_files);
        for item in &items {
            let mut s = self.clone();
            s.append(item);
            out.push(s);
        }

        out
    }
    pub fn get_all_directories_in_directory_as_paths(&self) -> Vec<OPath> {
        let mut out = vec![];

        let items = self.get_all_directories_in_directory();
        for item in &items {
            let mut s = self.clone();
            s.append(item);
            out.push(s);
        }

        out
    }
    fn directory_walk_standard_entry(optima_path: &mut OPath,
                                     out_vec: &mut Vec<OPath>,
                                     pattern: &OPathMatchingPattern) -> bool {
        let mut matched = false;
        match pattern {
            OPathMatchingPattern::FileOrDirName(s) => {
                let filename_option = optima_path.filename();
                if let Some(filename) = filename_option {
                    if &filename == s {
                        out_vec.push(optima_path.clone());
                        matched = true;
                    }
                }
            }
            OPathMatchingPattern::Extension(s) => {
                let extension_option = optima_path.extension();
                if let Some(extension) = extension_option {
                    if &extension == s {
                        out_vec.push(optima_path.clone());
                        matched = true;
                    }
                }
            }
            OPathMatchingPattern::FilenameWithoutExtension(s) => {
                let filename_option = optima_path.filename_without_extension();
                if let Some(filename) = filename_option {
                    if &filename == s {
                        out_vec.push(optima_path.clone());
                        matched = true;
                    }
                }
            }
            OPathMatchingPattern::PathComponents(v) => {
                let split = optima_path.split_path_into_string_components();
                let v_len = v.len();
                let split_len = split.len();
                if split_len < v_len { return false; }

                let mut local_match = true;
                for i in 0..v_len {
                    if &v[i] != &split[split_len - v_len + i] {
                        local_match = false;
                        break;
                    }
                }

                if local_match {
                    out_vec.push(optima_path.clone());
                    matched = true;
                }
            }
            OPathMatchingPattern::PathComponentsWithoutExtension(v) => {
                let mut optima_path_copy = optima_path.clone();
                optima_path_copy.set_extension("");
                let split = optima_path_copy.split_path_into_string_components();
                let v_len = v.len();
                let split_len = split.len();
                if split_len < v_len { return false; }

                let mut local_match = true;
                for i in 0..v_len {
                    if &v[i] != &split[split_len - v_len + i] {
                        local_match = false;
                        break;
                    }
                }

                if local_match {
                    out_vec.push(optima_path.clone());
                    matched = true;
                }
            }
        }
        return matched;
    }
    fn auto_create_optima_asset_path_json_file() -> bool {
        oprint_full("Searching for Optima assets folder...", PrintMode::Println, PrintColor::Cyan, true, 0, None, vec![]);
        let mut home_dir = Self::new_home_path();
        let walk_vec = home_dir.walk_directory_and_match(OPathMatchingPattern::FileOrDirName("optima_assets".to_string()), OPathMatchingStopCondition::All);
        return if walk_vec.is_empty() {
            oprint_full("WARNING: optima_assets folder not found on your computer.", PrintMode::Println, PrintColor::Yellow, true, 0, None, vec![]);
            let mut lock_path = Self::new_home_path();
            lock_path.append(".optima_asset_path.lock");
            lock_path.write_string_to_file(&"".to_string()).expect("error");
            oprint_full(&format!("Adding a lock file here: {:?}", lock_path), PrintMode::Println, PrintColor::Yellow, true, 0, None, vec![]);
            oprint_full(&format!("If you would like to use a local optima_assets directory on your computer, please delete the lock file once the assets directory is on your computer"), PrintMode::Println, PrintColor::Yellow, true, 0, None, vec![]);
            false
        } else {
            let mut found_path = walk_vec[0].clone();
            let mut split_vec = found_path.split_path_into_string_components();
            for p in &walk_vec {
                let sv = p.split_path_into_string_components();
                if sv.len() < split_vec.len() || split_vec.contains(&".git".to_string()) {
                    found_path = p.clone();
                    split_vec = sv.clone();
                }
            }

            match &found_path {
                OPath::Path(p) => {
                    oprint_full(&format!("Optima assets folder found at {:?}", p), PrintMode::Println, PrintColor::Green, true, 0, None, vec![]);
                    home_dir.append(".optima_asset_path.JSON");
                    oprint_full(&format!("Saved found path at {:?}", home_dir), PrintMode::Println, PrintColor::Green, true, 0, None, vec![]);
                    let path_to_assets_dir = PathToAssetsDir { path_to_assets_dir: p.clone() };
                    home_dir.save_object_to_file_as_json(&path_to_assets_dir).expect("error");
                    true
                }
                _ => { false }
            }
        }
    }
}
impl OPath {
    pub fn load_urdf(&self) -> Result<Robot, String> {
        self.verify_extension(&vec!["urdf"])?;
        let s = self.read_file_contents_to_string()?;
        let robot = urdf_rs::read_from_string(&s);
        return match robot {
            Ok(robot) => { Ok(robot) }
            Err(e) => { Err(format!("{:?}", e)) }
        }
    }
    /*
    pub fn load_dae(&self) -> Result<Scene, String> {
        let contents = self.read_file_contents_to_string()?;
        Ok( mesh_loader::collada::from_str(&contents).expect( &format!("there was an error loading dae. {:?}", self)) )
    }
    */
    pub fn load_dae(&self) -> Result<Document, String> {
        self.verify_extension(&vec!["dae", "DAE"])?;
        let contents = self.read_file_contents_to_string()?;
        Ok( Document::from_str(&contents).expect(&format!("there was an error loading dae. {:?}", self)) )
    }
    pub fn load_stl(&self) -> Result<IndexedMesh, String> {
        self.verify_extension(&vec!["stl", "STL"])?;
        return match self {
            OPath::Path(p) => {
                let mut file = File::open(p);
                match &mut file {
                    Ok(f) => {
                        let read_res = stl_io::read_stl(f);
                        match read_res {
                            Ok(read) => { Ok(read) }
                            Err(e) => { Err(e.to_string()) }
                        }
                    }
                    Err(e) => { Err(e.to_string()) }
                }
            }
            OPath::VfsPath(p) => {
                let mut file = p.open_file();
                match &mut file {
                    Ok(f) => {
                        let read_res = stl_io::read_stl(f);
                        match read_res {
                            Ok(read) => { Ok(read) }
                            Err(e) => { Err(e.to_string()) }
                        }
                    }
                    Err(e) => { Err(e.to_string()) }
                }
            }
        }
    }
}

/// Loads an object that implements the `Deserialize` trait from a deserialized json string.
pub fn load_object_from_json_string<T: DeserializeOwned>(json_str: &str) -> Result<T, String> {
    let o_res = serde_json::from_str(json_str);
    return match o_res {
        Ok(o) => {
            Ok(o)
        }
        Err(e) => {
            Err(e.to_string())
        }
    }
}

/// Loads an object that implements the `Deserialize` trait from a deserialized ron string.
pub fn load_object_from_ron_string<T: DeserializeOwned>(ron_str: &str) -> Result<T, String> {
    let o_res = ron::from_str::<T>(ron_str);
    match o_res {
        Ok(o) => {
            Ok(o)
        }
        Err(e) => {
            Err(e.to_string())
        }
    }
}

pub fn path_buf_from_string_components(components: &Vec<String>) -> PathBuf {
    let mut out = PathBuf::new();
    for c in components { out.push(c); }
    out
}

/// Convenience class that will be used for path_to_assets_dir.JSON file.
#[derive(Clone, Debug, Serialize, Deserialize)]
struct PathToAssetsDir {
    pub path_to_assets_dir: PathBuf
}

/// Asset folder location.  Will be used to easily access paths to these locations with respect to
/// the asset folder.
#[derive(Clone, Debug)]
pub enum OAssetLocation<'a> {
    RobotSets,
    RobotSet { set_name: String },
    Robots,
    Robot { robot_name: String },
    RobotConfigurations { robot_name: String },
    RobotInputMeshes { robot_name: String },
    RobotMeshes { robot_name: String  },
    RobotGLBMeshes { robot_name: String  },
    RobotPreprocessedData { robot_name: String },
    RobotModuleJsons { robot_name: String },
    RobotModuleJson { robot_name: String, t: RobotModuleJsonType },
    RobotConvexShapes { robot_name: String },
    RobotConvexSubcomponents { robot_name: String },
    Scenes,
    SceneMeshFiles,
    SceneMeshFile { name: String },
    SceneMeshFilePreprocessing { name: String },
    SceneMeshFileConvexShape { name: String },
    SceneMeshFileConvexShapeSubcomponents { name: String },
    FileIO,
    Chains,
    Chain { chain_name: &'a str },
    ChainOriginalMeshes { chain_name: &'a str },
    ChainSTLMeshes { chain_name: &'a str },
    ChainConvexHulls { chain_name: &'a str },
    ChainConvexDecomposition { chain_name: &'a str },
    LinkConvexDecomposition { chain_name: &'a str, link_mesh_name: &'a str }
}
impl<'a> OAssetLocation<'a> {
    pub fn get_path_wrt_asset_folder(&self) -> Vec<String> {
        return match self {
            OAssetLocation::RobotSets => {
                vec!["optima_robot_sets".to_string()]
            }
            OAssetLocation::RobotSet { set_name } => {
                let mut v = Self::RobotSets.get_path_wrt_asset_folder();
                v.push(set_name.clone());
                v
            }
            OAssetLocation::Robots => {
                vec!["optima_robots".to_string()]
            }
            OAssetLocation::Robot { robot_name } => {
                let mut v = Self::Robots.get_path_wrt_asset_folder();
                v.push(robot_name.clone());
                v
            }
            OAssetLocation::RobotConfigurations { robot_name } => {
                let mut v = Self::Robot { robot_name: robot_name.clone() }.get_path_wrt_asset_folder();
                v.push("configurations".to_string());
                v
            }
            OAssetLocation::RobotInputMeshes { robot_name } => {
                let mut v = Self::Robot { robot_name: robot_name.clone() }.get_path_wrt_asset_folder();
                v.push("input_meshes".to_string());
                v
            }
            OAssetLocation::RobotMeshes { robot_name } => {
                let mut v = Self::Robot { robot_name: robot_name.clone() }.get_path_wrt_asset_folder();
                v.push("meshes".to_string());
                v
            }
            OAssetLocation::RobotGLBMeshes { robot_name } => {
                let mut v = Self::Robot { robot_name: robot_name.clone() }.get_path_wrt_asset_folder();
                v.push("glb_meshes".to_string());
                v
            }
            OAssetLocation::RobotPreprocessedData { robot_name } => {
                let mut v = Self::Robot { robot_name: robot_name.clone() }.get_path_wrt_asset_folder();
                v.push("preprocessed_data".to_string());
                v
            }
            OAssetLocation::RobotModuleJsons { robot_name } => {
                let mut v = Self::RobotPreprocessedData { robot_name: robot_name.clone() }.get_path_wrt_asset_folder();
                v.push("robot_module_jsons".to_string());
                v
            }
            OAssetLocation::RobotModuleJson { robot_name, t } => {
                let mut v = Self::RobotModuleJsons { robot_name: robot_name.clone() }.get_path_wrt_asset_folder();
                v.push(t.filename().to_string());
                v
            }
            OAssetLocation::RobotConvexShapes { robot_name } => {
                let mut v = Self::RobotPreprocessedData { robot_name: robot_name.clone() }.get_path_wrt_asset_folder();
                v.push("convex_shapes".to_string());
                v
            }
            OAssetLocation::RobotConvexSubcomponents { robot_name } => {
                let mut v = Self::RobotPreprocessedData { robot_name: robot_name.clone() }.get_path_wrt_asset_folder();
                v.push("convex_shape_subcomponents".to_string());
                v
            }
            OAssetLocation::Scenes => {
                vec!["optima_scenes".to_string()]
            }
            OAssetLocation::SceneMeshFiles => {
                let mut v = Self::Scenes.get_path_wrt_asset_folder();
                v.push("mesh_files".to_string());
                v
            }
            OAssetLocation::SceneMeshFile { name } => {
                let mut v = Self::SceneMeshFiles.get_path_wrt_asset_folder();
                v.push(name.clone());
                v
            }
            OAssetLocation::SceneMeshFilePreprocessing { name } => {
                let mut v = Self::SceneMeshFile { name: name.clone() }.get_path_wrt_asset_folder();
                v.push("preprocessing".to_string());
                v
            }
            OAssetLocation::SceneMeshFileConvexShape { name } => {
                let mut v = Self::SceneMeshFilePreprocessing { name: name.clone() }.get_path_wrt_asset_folder();
                v.push("convex_shape".to_string());
                v
            }
            OAssetLocation::SceneMeshFileConvexShapeSubcomponents { name } => {
                let mut v = Self::SceneMeshFilePreprocessing { name: name.clone() }.get_path_wrt_asset_folder();
                v.push("convex_shape_subcomponents".to_string());
                v
            }
            OAssetLocation::FileIO => {
                vec!["fileIO".to_string()]
            }
            OAssetLocation::Chains => {
                vec!["chains".to_string()]
            }
            OAssetLocation::Chain { chain_name } => {
                let mut v = Self::Chains.get_path_wrt_asset_folder();
                v.push(chain_name.to_string());
                v
            }
            OAssetLocation::ChainOriginalMeshes { chain_name } => {
                let mut v = Self::Chain { chain_name }.get_path_wrt_asset_folder();
                v.push("original_meshes".to_string());
                v
            }
            OAssetLocation::ChainSTLMeshes { chain_name } => {
                let mut v = Self::Chain { chain_name }.get_path_wrt_asset_folder();
                v.push("stl_meshes".to_string());
                v
            }
            OAssetLocation::ChainConvexHulls { chain_name } => {
                let mut v = Self::Chain { chain_name }.get_path_wrt_asset_folder();
                v.push("convex_hulls".to_string());
                v
            }
            OAssetLocation::ChainConvexDecomposition { chain_name } => {
                let mut v = Self::Chain { chain_name }.get_path_wrt_asset_folder();
                v.push("convex_decomposition".to_string());
                v
            }
            OAssetLocation::LinkConvexDecomposition { chain_name, link_mesh_name } => {
                let mut v = Self::ChainConvexDecomposition { chain_name }.get_path_wrt_asset_folder();
                v.push(link_mesh_name.to_string());
                v
            }
        }
    }
}

/// An Enum used to specify a particular patten that should be matched during a directory walk.
#[derive(Debug, Clone)]
pub enum OPathMatchingPattern {
    FileOrDirName(String),
    Extension(String),
    FilenameWithoutExtension(String),
    PathComponents(Vec<String>),
    PathComponentsWithoutExtension(Vec<String>)
}

/// An Enum used to specify when a particular directory walk should stop based on when a pattern is
/// matched.  For example, `First` will stop the directory walk when the given pattern is matched
/// for the first time, while `All` will continue recursively searching through the whole directory
/// until all potential matches are found.
#[derive(Debug, Clone)]
pub enum OPathMatchingStopCondition {
    First,
    All
}

/// Specifies a particular robot module json type.  This enum provides a unified and convenient way
/// to handle paths to particular module json files.
#[derive(Clone, Debug)]
pub enum RobotModuleJsonType {
    ModelModule,
    ShapeGeometryModule,
    ShapeGeometryModulePermanent
}
impl RobotModuleJsonType {
    pub fn filename(&self) -> &str {
        match self {
            RobotModuleJsonType::ModelModule => { "robot_model_module.JSON" }
            RobotModuleJsonType::ShapeGeometryModule => { "robot_shape_geometry_module.JSON" }
            RobotModuleJsonType::ShapeGeometryModulePermanent => { "robot_shape_geometry_module_permanent.JSON" }
        }
    }
}