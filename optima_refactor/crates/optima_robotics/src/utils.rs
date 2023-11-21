use ad_trait::AD;
use optima_file::path::{OAssetLocation, OStemCellPath};
use optima_linalg::OVec;
use crate::robotics_traits::{AsTrajectory, AsTrajectoryWaypoint};

pub fn get_urdf_path_from_chain_name(chain_name: &str) -> OStemCellPath {
    let mut p = OStemCellPath::new_asset_path();
    // p.append_file_location(&OAssetLocation::Robot { robot_name: robot_name.to_string() });
    p.append_file_location(&OAssetLocation::UrdfRobot { robot_name: chain_name });
    let item_paths = p.get_all_items_in_directory_as_paths(false, false);
    for item_path in &item_paths {
        let extension = item_path.extension();
        if let Some(extension) = extension {
            if extension == "urdf" { return item_path.clone(); }
        }
    }
    panic!("urdf file not found for chain name {}", chain_name);
}

pub struct RobotTrajectoryWaypoint<T: AD, V: OVec<T>> {
    time: T,
    waypoint: V
}
impl<T: AD, V: OVec<T>> RobotTrajectoryWaypoint<T, V> {
    pub fn new(time: T, waypoint: V) -> Self {
        Self { time, waypoint }
    }
}
impl<T: AD, V: OVec<T>> AsTrajectoryWaypoint<T> for RobotTrajectoryWaypoint<T, V> {
    #[inline(always)]
    fn get_waypoint_time(&self) -> T {
        self.time
    }

    #[inline(always)]
    fn waypoint_as_slice(&self) -> &[T] {
        self.waypoint.ovec_as_slice()
    }
}

pub struct RobotTrajectory<T: AD, V: OVec<T>> {
    waypoints: Vec<RobotTrajectoryWaypoint<T, V>>
}
impl<T: AD, V: OVec<T>> RobotTrajectory<T, V> {
    pub fn new_empty() -> Self {
        Self {
            waypoints: vec![],
        }
    }

    pub fn new(waypoints: Vec<RobotTrajectoryWaypoint<T, V>>) -> Self {
        Self { waypoints }
    }

    pub fn add_waypoint(&mut self, waypoint: RobotTrajectoryWaypoint<T, V>) {
        self.waypoints.push(waypoint);
    }
}
impl<T: AD, V: OVec<T>> AsTrajectory<T, RobotTrajectoryWaypoint<T, V>> for RobotTrajectory<T, V> {
    #[inline(always)]
    fn get_waypoints(&self) -> &Vec<RobotTrajectoryWaypoint<T, V>> {
        &self.waypoints
    }
}


