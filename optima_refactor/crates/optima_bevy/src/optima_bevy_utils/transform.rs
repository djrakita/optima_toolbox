use ad_trait::AD;
use bevy::math::Quat;
use bevy::prelude::{Transform, Vec3};
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_3d_spatial::optima_3d_rotation::O3DRotation;
use optima_3d_spatial::optima_3d_vec::O3DVec;
use optima_linalg::OVec;

pub struct TransformUtils;
impl TransformUtils {
    #[inline(always)]
    pub fn util_convert_3d_pose_to_y_up_bevy_transform<T: AD, P: O3DPose<T>>(pose: &P) -> Transform {
        let pose_new = P::from_constructors(&[T::zero(),T::zero(),T::zero()], &[T::constant(-std::f64::consts::FRAC_PI_2), T::zero(), T::zero()]).mul(pose);
        let t = pose_new.translation();
        let r = pose_new.rotation().unit_quaternion_as_wxyz_slice();

        return Transform {
            translation: Vec3::new( t.x().to_constant() as f32, t.y().to_constant() as f32, t.z().to_constant() as f32 ),
            rotation: Quat::from_xyzw( r[1].to_constant() as f32, r[2].to_constant() as f32, r[3].to_constant() as f32, r[0].to_constant() as f32 ),
            ..Default::default()
        }
    }

    #[inline(always)]
    pub fn util_convert_z_up_vec3_to_y_up_bevy_vec3(vec: Vec3) -> Vec3 {
        return Vec3::new(vec.x, vec.z, -vec.y);
    }

    #[inline(always)]
    pub fn util_convert_bevy_y_up_vec3_to_z_up_vec3(vec: Vec3) -> Vec3 {
        return Vec3::new(vec.x, -vec.z, vec.y);
    }

    #[inline(always)]
    pub fn util_convert_z_up_ovec3_to_z_up_vec3<T: AD, V: O3DVec<T>>(v: &V) -> Vec3 {
        return Vec3::new( v.x().to_constant() as f32, v.y().to_constant() as f32, v.z().to_constant() as f32 )
    }

    #[inline(always)]
    pub fn util_convert_z_up_ovec3_to_y_up_bevy_vec3<T: AD, V: O3DVec<T>>(v: &V) -> Vec3 {
        let v = Self::util_convert_z_up_ovec3_to_z_up_vec3(v);
        Self::util_convert_z_up_vec3_to_y_up_bevy_vec3(v)
    }

    #[inline(always)]
    pub fn util_convert_z_up_ovec_to_z_up_vec3<T: AD, V: OVec<T>>(v: &V) -> Vec3 {
        return Vec3::new( v.ovec_get_element(0).to_constant() as f32, v.ovec_get_element(1).to_constant() as f32, v.ovec_get_element(2).to_constant() as f32 )
    }

    #[inline(always)]
    pub fn util_convert_z_up_ovec_to_y_up_bevy_vec3<T: AD, V: OVec<T>>(v: &V) -> Vec3 {
        let v = Self::util_convert_z_up_ovec_to_z_up_vec3(v);
        Self::util_convert_z_up_vec3_to_y_up_bevy_vec3(v)
    }
}