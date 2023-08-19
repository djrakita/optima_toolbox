use optima_robotics::robot_model::ORobotModelDefault;

fn main() {
    let r = ORobotModelDefault::<f64>::from_urdf("ur5");
    println!("{:?}", r.link_kinematic_hierarchy());
    println!("{:?}", r.base_link_idx());

    r.links().iter().for_each(|x| {
       println!("{:?}", x.link_idx_paths_to_other_links());
    });
}