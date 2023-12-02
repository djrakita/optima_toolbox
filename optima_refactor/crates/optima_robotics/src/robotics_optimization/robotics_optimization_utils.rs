use ad_trait::differentiable_function::DerivativeMethodTrait;
use optima_3d_spatial::optima_3d_pose::{O3DPoseCategoryIsometry3, O3DPoseCategory};
use optima_linalg::{OLinalgCategoryNalgebra, OLinalgCategory};
use crate::robot::{ORobot};

pub struct RoboticsOptimizationUtils;
impl RoboticsOptimizationUtils {
    pub fn get_f64_and_ad_default_robots<E: DerivativeMethodTrait>(robot_name: &str) -> (ORobot<f64, O3DPoseCategoryIsometry3, OLinalgCategoryNalgebra>, ORobot<<E as DerivativeMethodTrait>::T, O3DPoseCategoryIsometry3, OLinalgCategoryNalgebra>) {
        Self::get_f64_and_ad_robots::<E, O3DPoseCategoryIsometry3, OLinalgCategoryNalgebra>(robot_name)
    }

    pub fn get_f64_and_ad_robots<E: DerivativeMethodTrait, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static>(robot_name: &str) -> (ORobot<f64, C, L>, ORobot<E::T, C, L>) {
        let robot1 = ORobot::<f64, C, L>::load_from_saved_robot(robot_name);
        let robot2 = robot1.to_new_ad_type::<E::T>();

        (robot1, robot2)
    }
}