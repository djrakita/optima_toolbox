use std::fmt::Debug;
use ad_trait::{AD, NalgebraMatMulAD};
use nalgebra::{ArrayStorage, Const, Matrix3, Quaternion, Rotation3, UnitQuaternion, Vector3};
use crate::optima_3d_vec::Optima3DVec;

/// Point is the "native vector" type that serve as the native type that this rotation multiplies by
pub trait Optima3DRotation<'a, T: AD> :
    Debug
{
    type Native3DVecType: Optima3DVec<'a, T>;

    fn mul(&self, other: &Self) -> Self;
    fn mul_by_point_native(&self, point: &Self::Native3DVecType) -> Self::Native3DVecType;
    fn mul_by_point_generic<V: Optima3DVec<'a, T>>(&self, point: &V) -> V;
    fn scaled_axis_of_rotation(&self) -> [T; 3];
    fn from_scaled_axis_of_rotation<V: Optima3DVec<'a, T>>(axis: &V) -> Self;
    fn euler_angles(&self) -> [T; 3];
    fn from_euler_angles<V: Optima3DVec<'a, T>>(euler_angles: &V) -> Self;
    fn rotation_matrix_as_column_major_slice(&self) -> [T; 9];
    fn from_rotation_matrix_as_column_major_slice(slice: &[T]) -> Self;
    fn unit_quaternion_as_wxyz_slice(&self) -> [T; 4];
    fn from_unit_quaternion_as_wxyz_slice(slice: &[T]) -> Self;
    fn coordinate_frame_vectors(&self) -> [[T;3]; 3] {
        let r = self.rotation_matrix_as_column_major_slice();
        let vec1 = [r[0], r[1], r[2]];
        let vec2 = [r[3], r[4], r[5]];
        let vec3 = [r[6], r[7], r[8]];

        [ vec1, vec2, vec3 ]
    }
    fn inverse(&self) -> Self;
    fn angle(&self) -> T;
    fn displacement(&self, other: &Self) -> Self;
    fn dis(&self, other: &Self) -> T;
    fn interpolate(&self, to: &Self, t: T) -> Self;
}

impl<'a, T: AD + NalgebraMatMulAD<'a, Const<3>, Const<1>, ArrayStorage<T, 3, 1>>> Optima3DRotation<'a, T> for Rotation3<T> {
    type Native3DVecType = Vector3<T>;

    fn mul(&self, other: &Self) -> Self {
        self * other
    }

    fn mul_by_point_native(&self, point: &Vector3<T>) -> Vector3<T> {
        self * point
    }

    fn mul_by_point_generic<V: Optima3DVec<'a, T>>(&self, point: &V) -> V {
        let res = self * Vector3::from_column_slice(point.as_slice());
        V::from_slice(res.as_slice())
    }

    fn scaled_axis_of_rotation(&self) -> [T; 3] {
        let v = self.scaled_axis();
        [ v[0], v[1], v[2] ]
    }

    fn from_scaled_axis_of_rotation<V: Optima3DVec<'a, T>>(axis: &V) -> Self {
        let slice = axis.as_slice();
        Rotation3::from_scaled_axis(Vector3::from_column_slice(slice))
    }

    fn euler_angles(&self) -> [T; 3] {
        let e = self.euler_angles();

        [ e.0, e.1, e.2 ]
    }

    fn from_euler_angles<V: Optima3DVec<'a, T>>(euler_angles: &V) -> Self {
        Rotation3::from_euler_angles(euler_angles.x(), euler_angles.y(), euler_angles.z())
    }

    fn rotation_matrix_as_column_major_slice(&self) -> [T; 9] {
        <[T; 9]>::try_from(self.matrix().as_slice()).unwrap()
    }

    fn from_rotation_matrix_as_column_major_slice(slice: &[T]) -> Self {
        let m = Matrix3::from_column_slice(slice);
        Rotation3::from_matrix(&m)
    }

    fn unit_quaternion_as_wxyz_slice(&self) -> [T; 4] {
        let q = UnitQuaternion::from_scaled_axis(self.scaled_axis());

        [ q.w, q.i, q.j, q.k ]
    }

    fn from_unit_quaternion_as_wxyz_slice(slice: &[T]) -> Self {
        let q = UnitQuaternion::from_quaternion(Quaternion::new(slice[0], slice[1], slice[2], slice[3]));
        return Rotation3::from_scaled_axis(q.scaled_axis())
    }

    fn coordinate_frame_vectors(&self) -> [[T; 3]; 3] {
        let vec1 = [ self[(0,0)], self[(1,0)], self[(2,0)] ];
        let vec2 = [ self[(0,1)], self[(1,1)], self[(2,1)] ];
        let vec3 = [ self[(0,2)], self[(1,2)], self[(2,2)] ];

        [ vec1, vec2, vec3 ]
    }

    fn inverse(&self) -> Self {
        self.inverse()
    }

    fn angle(&self) -> T {
        self.angle()
    }

    fn displacement(&self, other: &Self) -> Self {
        self.inverse() * other
    }

    fn dis(&self, other: &Self) -> T {
        self.displacement(other).angle()
    }

    fn interpolate(&self, to: &Self, t: T) -> Self {
        self.slerp(to, t)
    }
}

impl<'a, T: AD + NalgebraMatMulAD<'a, Const<3>, Const<1>, ArrayStorage<T, 3, 1>>> Optima3DRotation<'a, T> for UnitQuaternion<T> {
    type Native3DVecType = Vector3<T>;

    fn mul(&self, other: &Self) -> Self {
        self * other
    }

    fn mul_by_point_native(&self, point: &Vector3<T>) -> Vector3<T> {
        self * point
    }

    fn mul_by_point_generic<V: Optima3DVec<'a, T>>(&self, point: &V) -> V {
        let res = self * Vector3::from_column_slice(point.as_slice());
        V::from_slice(res.as_slice())
    }

    fn scaled_axis_of_rotation(&self) -> [T; 3] {
        let v = self.scaled_axis();
        [ v[0], v[1], v[2] ]
    }

    fn from_scaled_axis_of_rotation<V: Optima3DVec<'a, T>>(axis: &V) -> Self {
        let slice = axis.as_slice();
        UnitQuaternion::from_scaled_axis(Vector3::from_column_slice(slice))
    }

    fn euler_angles(&self) -> [T; 3] {
        let e = self.euler_angles();

        [ e.0, e.1, e.2 ]
    }

    fn from_euler_angles<V: Optima3DVec<'a, T>>(euler_angles: &V) -> Self {
        UnitQuaternion::from_euler_angles(euler_angles.x(), euler_angles.y(), euler_angles.z())
    }

    fn rotation_matrix_as_column_major_slice(&self) -> [T; 9] {
        <[T; 9]>::try_from(self.to_rotation_matrix().matrix().as_slice()).unwrap()
    }

    fn from_rotation_matrix_as_column_major_slice(slice: &[T]) -> Self {
        let m = Matrix3::from_column_slice(slice);
        UnitQuaternion::from_matrix(&m)
    }

    fn unit_quaternion_as_wxyz_slice(&self) -> [T; 4] {
        [ self.w, self.i, self.j, self.k ]
    }

    fn from_unit_quaternion_as_wxyz_slice(slice: &[T]) -> Self {
        UnitQuaternion::from_quaternion(Quaternion::new(slice[0], slice[1], slice[2], slice[3]))
    }

    fn coordinate_frame_vectors(&self) -> [[T; 3]; 3] {
        let m = self.to_rotation_matrix();

        let vec1 = [ m[(0,0)], m[(1,0)], m[(2,0)] ];
        let vec2 = [ m[(0,1)], m[(1,1)], m[(2,1)] ];
        let vec3 = [ m[(0,2)], m[(1,2)], m[(2,2)] ];

        [ vec1, vec2, vec3 ]
    }

    fn inverse(&self) -> Self {
        self.inverse()
    }

    fn angle(&self) -> T {
        self.angle()
    }

    fn displacement(&self, other: &Self) -> Self {
        self.inverse() * other
    }

    fn dis(&self, other: &Self) -> T {
        self.displacement(other).angle()
    }

    fn interpolate(&self, to: &Self, t: T) -> Self {
        self.slerp(to, t)
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait Optima3DRotationConstructor<'a, T: AD, TargetRotationType: Optima3DRotation<'a, T>> {
    fn construct(&self) -> TargetRotationType;
}

/// This will be euler angles
impl<'a, T, TargetRotationType> Optima3DRotationConstructor<'a, T, TargetRotationType> for [T; 3]
    where T: AD,
          TargetRotationType: Optima3DRotation<'a, T>
{
    fn construct(&self) -> TargetRotationType {
        TargetRotationType::from_euler_angles(self)
    }
}

pub struct ScaledAxis<T: AD>(pub [T; 3]);
impl<'a, T, TargetRotationType> Optima3DRotationConstructor<'a, T, TargetRotationType> for ScaledAxis<T>
    where T: AD,
          TargetRotationType: Optima3DRotation<'a, T>
{
    fn construct(&self) -> TargetRotationType {
        TargetRotationType::from_scaled_axis_of_rotation(&self.0)
    }
}

impl<'a, T, TargetRotationType> Optima3DRotationConstructor<'a, T, TargetRotationType> for Rotation3<T>
    where T: AD,
          TargetRotationType: Optima3DRotation<'a, T>
{
    fn construct(&self) -> TargetRotationType {
        TargetRotationType::from_rotation_matrix_as_column_major_slice(self.matrix().as_slice())
    }
}

impl<'a, T, TargetRotationType> Optima3DRotationConstructor<'a, T, TargetRotationType> for UnitQuaternion<T>
    where T: AD,
          TargetRotationType: Optima3DRotation<'a, T>
{
    fn construct(&self) -> TargetRotationType {
        TargetRotationType::from_unit_quaternion_as_wxyz_slice(self.coords.as_slice())
    }
}
