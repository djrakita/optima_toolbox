use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryTrait};
use optima_3d_spatial::optima_3d_rotation::ScaledAxis;
use optima_linalg::{OLinalgCategoryTrait, OVec};
use crate::robot::ORobot;
use crate::robotics_components::{ODynamics, OJointLimit, OJointType, OMimic, OPose, OSafetyController};
use optima_misc::arr_storage::ImmutArrTraitRaw;

pub trait JointTrait<T: AD, C: O3DPoseCategoryTrait + 'static> {
    fn name(&self) -> &str;
    fn joint_idx(&self) -> usize;
    fn parent_link_idx(&self) -> usize;
    fn child_link_idx(&self) -> usize;
    fn joint_type(&self) -> &OJointType;
    fn axis(&self) -> &[T; 3];
    fn origin(&self) -> &OPose<T, C>;
    fn limit(&self) -> &OJointLimit<T>;
    fn mimic(&self) -> &Option<OMimic<T>>;
    fn dynamics(&self) -> &Option<ODynamics<T>>;
    fn safety_controller(&self) -> &Option<OSafetyController<T>>;
    fn get_num_dofs(&self) -> usize;
    fn fixed_values(&self) -> &Option<Vec<T>>;
    fn dof_idxs(&self) -> &Vec<usize>;
    fn dof_idxs_range(&self) -> &Option<(usize, usize)>;
    #[inline]
    fn get_variable_transform_from_joint_values_subslice(&self, joint_values_subslice: &[T]) -> C::P<T> {
        return match &self.joint_type() {
            OJointType::Revolute => {
                assert_eq!(joint_values_subslice.len(), 1);
                let axis = self.axis();
                let axis = axis.ovec_scalar_mul(&joint_values_subslice[0]);
                C::P::<T>::from_constructors(&[T::zero(), T::zero(), T::zero()], &ScaledAxis(axis))
            }
            OJointType::Continuous => {
                assert_eq!(joint_values_subslice.len(), 1);
                let axis = self.axis();
                let axis = axis.ovec_scalar_mul(&joint_values_subslice[0]);
                C::P::<T>::from_constructors(&[T::zero(), T::zero(), T::zero()], &ScaledAxis(axis))
            }
            OJointType::Prismatic => {
                assert_eq!(joint_values_subslice.len(), 1);
                let axis = self.axis();
                let axis = axis.ovec_scalar_mul(&joint_values_subslice[0]);
                C::P::<T>::from_constructors(&axis, &[T::zero(), T::zero(), T::zero()])
            }
            OJointType::Fixed => {
                C::P::<T>::identity()
            }
            OJointType::Floating => {
                assert_eq!(joint_values_subslice.len(), 6);
                C::P::<T>::from_constructors(&[joint_values_subslice[0], joint_values_subslice[1], joint_values_subslice[2]], &ScaledAxis([joint_values_subslice[3], joint_values_subslice[4], joint_values_subslice[5]]))
            }
            OJointType::Planar => {
                assert_eq!(joint_values_subslice.len(), 2);
                C::P::<T>::from_constructors(&[joint_values_subslice[0], joint_values_subslice[1], T::zero()], &[T::zero(), T::zero(), T::zero()])
            }
            OJointType::Spherical => {
                assert_eq!(joint_values_subslice.len(), 3);
                C::P::<T>::from_constructors(&[T::zero(), T::zero(), T::zero()], &ScaledAxis([joint_values_subslice[0], joint_values_subslice[1], joint_values_subslice[2]]))
            }
        }
    }
    #[inline]
    fn get_joint_variable_transform<V: OVec<T>>(&self, state: &V, all_joints: &Vec<Self>) -> C::P<T> where Self: Sized {
        let joint = self;
        if let Some(fixed_values) = &joint.fixed_values() {
            joint.get_variable_transform_from_joint_values_subslice(fixed_values)
        } else if let Some(mimic) = &joint.mimic() {
            let mimic_joint_idx = mimic.joint_idx();
            let mimic_joint = &all_joints[mimic_joint_idx];
            let range = mimic_joint.dof_idxs_range();
            match range {
                None => { panic!("mimicked joint must have exactly one dof.") }
                Some(range) => {
                    let axis = joint.axis();
                    let mut negative_axis = false;
                    axis.iter().for_each(|x| if x.is_negative() { negative_axis = true } );
                    let subslice = state.subslice(range.0, range.1);
                    assert_eq!(subslice.len(), 1, "mimicked joint must have exactly one dof.");
                    let mut value = match (mimic.offset(), mimic.multiplier()) {
                        (Some(offset), Some(multiplier)) => { (subslice[0] + *offset) * *multiplier }
                        (None, Some(multiplier)) => { subslice[0] * *multiplier }
                        (Some(offset), None) => { subslice[0] + *offset }
                        (None, None) => { subslice[0] }
                    };
                    if negative_axis { value = -value; }
                    return all_joints.get_element(mimic_joint_idx).get_variable_transform_from_joint_values_subslice(&[value]);
                }
            }
        } else {
            let range = joint.dof_idxs_range();
            return match range {
                None => { C::P::<T>::identity() }
                Some(range) => {
                    let subslice = state.subslice(range.0, range.1);
                    joint.get_variable_transform_from_joint_values_subslice(subslice)
                }
            }
        }
    }
    #[inline]
    fn get_joint_fixed_offset_transform(&self) -> &C::P<T> {
        self.origin().pose()
    }
    #[inline]
    fn get_joint_transform<V: OVec<T>>(&self, state: &V, all_joints: &Vec<Self>) -> C::P<T> where Self: Sized {
        // self.get_joint_fixed_offset_transform(joint_idx).mul(&self.get_joint_variable_transform(state, joint_idx))
        self.get_joint_fixed_offset_transform().mul(&self.get_joint_variable_transform(state, all_joints))
    }
}

pub trait AsRobotTrait<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> {
    fn as_robot(&self) -> &ORobot<T, C, L>;
}

pub trait AsTrajectoryWaypoint<T: AD> {
    fn get_waypoint_time(&self) -> T;
    fn waypoint_as_slice(&self) -> &[T];
}

pub trait AsTrajectory<T: AD, W: AsTrajectoryWaypoint<T>> {
    fn get_waypoints(&self) -> &Vec<W>;
}



/*
pub trait OForwardKinematicsTrait<T: AD, P: O3DPose<T>> {
    fn forward_kinematics<V: OVec<T>>(&self, state: &V, base_offset: Option<&P>) -> Vec<Vec<Option<P>>>;
    fn forward_kinematics_floating_chain<V: OVec<T>>(&self, state: &V, start_link_idx: usize, end_link_idx: usize, base_offset: Option<&P>) -> Vec<Vec<Option<P>>>;
}
*/

/*
pub trait OJacobianTrait<T: AD, P: O3DPose<T>, L: OLinalgCategoryTrait> : OForwardKinematicsTrait<T, P> {
    fn jacobian<V: OVec<T>>(&self, state: &V, start_link_idx: Option<usize>, end_link_idx: usize) -> L::MatType<T>;
}
*/

/*
#[derive(Clone, Debug)]
pub enum JacobianMode {
    Full, Translational, Rotational
}

#[derive(Clone, Debug)]
pub enum JacobianEndPoint<T: AD, H: O3DVec<T>> {
    Link,
    LocalOffset(H),
    GlobalOffset(H),
    InertialOrigin,
    _PhantomData(PhantomData<T>)
}
*/