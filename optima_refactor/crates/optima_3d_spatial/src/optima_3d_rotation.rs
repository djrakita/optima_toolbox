use std::borrow::Cow;
use std::fmt;
use std::fmt::Debug;
use std::marker::PhantomData;
use ad_trait::{AD};
use as_any::AsAny;
use nalgebra::{Matrix3, Quaternion, Rotation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde::de::{SeqAccess, Visitor};
use serde::ser::SerializeTuple;
use serde_with::{DeserializeAs, SerializeAs};
use optima_linalg::OVec;
use crate::optima_3d_vec::O3DVec;

#[derive(Clone, Debug, Copy, Eq, PartialEq)]
pub enum O3DRotationType {
    NalgebraRotation3, NalgebraUnitQuaternion
}

pub trait O3DRotationCategoryTrait :
    Clone + Debug + Serialize + for<'a> Deserialize<'a> + Send + Sync
{
    type R<T: AD> : O3DRotation<T>;
}

/// Point is the "native vector" type that serve as the native type that this rotation multiplies by
pub trait O3DRotation<T: AD> :
    Clone + Debug + Serialize + for<'a> Deserialize<'a> + Send + Sync + AsAny {
    type Category: O3DRotationCategoryTrait;
    type Native3DVecType: O3DVec<T>;

    fn type_identifier() -> O3DRotationType;
    fn mul(&self, other: &Self) -> Self;
    fn mul_by_point_native(&self, point: &Self::Native3DVecType) -> Self::Native3DVecType;
    fn mul_by_point_generic<V: O3DVec<T>>(&self, point: &V) -> V;
    fn scaled_axis_of_rotation(&self) -> [T; 3];
    fn from_scaled_axis_of_rotation<V: O3DVec<T>>(axis: &V) -> Self;
    fn euler_angles(&self) -> [T; 3];
    fn from_euler_angles<V: O3DVec<T>>(euler_angles: &V) -> Self;
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
    #[inline(always)]
    fn o3drot_to_constant_ads(&self) -> Self {
        let axis: Vec<T> = self.scaled_axis_of_rotation().iter().map(|x| T::constant(x.to_constant()) ).collect();
        Self::from_scaled_axis_of_rotation(&axis)
    }
    fn o3drot_to_other_generic_category<T2: AD, C: O3DRotationCategoryTrait>(&self) -> C::R<T2> {
        let scaled_axis = self.scaled_axis_of_rotation().o3dvec_to_other_ad_type::<T2>();
        C::R::from_scaled_axis_of_rotation(&scaled_axis)
    }
    fn o3drot_to_other_ad_type<T2: AD>(&self) -> <Self::Category as O3DRotationCategoryTrait>::R<T2> {
        self.o3drot_to_other_generic_category::<T2, Self::Category>()
    }
    #[inline(always)]
    fn o3drot_downcast_or_convert<R: O3DRotation<T>>(&self) -> Cow<R> {
        let downcast = self.as_any().downcast_ref::<R>();
        match downcast {
            Some(d) => { Cow::Borrowed(d) }
            None => {
                let out = R::from_scaled_axis_of_rotation(&self.scaled_axis_of_rotation());
                Cow::Owned(out)
            }
        }
    }

}

impl<T: AD> O3DRotation<T> for Rotation3<T> {
    type Category = O3DRotationCategoryRotation3;
    type Native3DVecType = Vector3<T>;

    #[inline(always)]
    fn type_identifier() -> O3DRotationType {
        O3DRotationType::NalgebraRotation3
    }

    #[inline]
    fn mul(&self, other: &Self) -> Self {
        self * other
    }

    #[inline]
    fn mul_by_point_native(&self, point: &Vector3<T>) -> Vector3<T> {
        self * point
    }

    #[inline]
    fn mul_by_point_generic<V: O3DVec<T>>(&self, point: &V) -> V {
        let res = self * Vector3::from_column_slice(point.o3dvec_as_slice());
        V::o3dvec_from_slice(res.as_slice())
    }

    #[inline]
    fn scaled_axis_of_rotation(&self) -> [T; 3] {
        let v = self.scaled_axis();
        [ v[0], v[1], v[2] ]
    }

    #[inline]
    fn from_scaled_axis_of_rotation<V: O3DVec<T>>(axis: &V) -> Self {
        let slice = axis.o3dvec_as_slice();
        Rotation3::from_scaled_axis(Vector3::from_column_slice(slice))
    }

    #[inline]
    fn euler_angles(&self) -> [T; 3] {
        let e = self.euler_angles();

        [ e.0, e.1, e.2 ]
    }

    #[inline]
    fn from_euler_angles<V: O3DVec<T>>(euler_angles: &V) -> Self {
        Rotation3::from_euler_angles(euler_angles.x(), euler_angles.y(), euler_angles.z())
    }

    #[inline]
    fn rotation_matrix_as_column_major_slice(&self) -> [T; 9] {
        <[T; 9]>::try_from(self.matrix().as_slice()).unwrap()
    }

    #[inline]
    fn from_rotation_matrix_as_column_major_slice(slice: &[T]) -> Self {
        let m = Matrix3::from_column_slice(slice);
        Rotation3::from_matrix(&m)
    }

    #[inline]
    fn unit_quaternion_as_wxyz_slice(&self) -> [T; 4] {
        let q = UnitQuaternion::from_scaled_axis(self.scaled_axis());

        [ q.w, q.i, q.j, q.k ]
    }

    #[inline]
    fn from_unit_quaternion_as_wxyz_slice(slice: &[T]) -> Self {
        let q = UnitQuaternion::from_quaternion(Quaternion::new(slice[0], slice[1], slice[2], slice[3]));
        return Rotation3::from_scaled_axis(q.scaled_axis())
    }

    #[inline]
    fn coordinate_frame_vectors(&self) -> [[T; 3]; 3] {
        let vec1 = [ self[(0,0)], self[(1,0)], self[(2,0)] ];
        let vec2 = [ self[(0,1)], self[(1,1)], self[(2,1)] ];
        let vec3 = [ self[(0,2)], self[(1,2)], self[(2,2)] ];

        [ vec1, vec2, vec3 ]
    }

    #[inline]
    fn inverse(&self) -> Self {
        self.inverse()
    }

    #[inline(always)]
    fn angle(&self) -> T {
        self.angle()
    }

    #[inline]
    fn displacement(&self, other: &Self) -> Self {
        self.inverse() * other
    }

    #[inline]
    fn dis(&self, other: &Self) -> T {
        self.displacement(other).angle()
    }

    #[inline]
    fn interpolate(&self, to: &Self, t: T) -> Self {
        self.slerp(to, t)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct O3DRotationCategoryRotation3;
impl O3DRotationCategoryTrait for O3DRotationCategoryRotation3 {
    type R<T: AD> = Rotation3<T>;
}

impl<T: AD> O3DRotation<T> for UnitQuaternion<T> {
    type Category = O3DRotationCategoryUnitQuaternion;
    type Native3DVecType = Vector3<T>;

    #[inline(always)]
    fn type_identifier() -> O3DRotationType {
        O3DRotationType::NalgebraRotation3
    }

    #[inline(always)]
    fn mul(&self, other: &Self) -> Self {
        self * other
    }

    #[inline(always)]
    fn mul_by_point_native(&self, point: &Vector3<T>) -> Vector3<T> {
        self * point
    }

    #[inline(always)]
    fn mul_by_point_generic<V: O3DVec<T>>(&self, point: &V) -> V {
        let res = self * Vector3::from_column_slice(point.o3dvec_as_slice());
        V::o3dvec_from_slice(res.as_slice())
    }

    #[inline(always)]
    fn scaled_axis_of_rotation(&self) -> [T; 3] {
        let v = self.scaled_axis();
        [ v[0], v[1], v[2] ]
    }

    #[inline(always)]
    fn from_scaled_axis_of_rotation<V: O3DVec<T>>(axis: &V) -> Self {
        let slice = axis.o3dvec_as_slice();
        UnitQuaternion::from_scaled_axis(Vector3::from_column_slice(slice))
    }

    #[inline(always)]
    fn euler_angles(&self) -> [T; 3] {
        let e = self.euler_angles();

        [ e.0, e.1, e.2 ]
    }

    #[inline(always)]
    fn from_euler_angles<V: O3DVec<T>>(euler_angles: &V) -> Self {
        UnitQuaternion::from_euler_angles(euler_angles.x(), euler_angles.y(), euler_angles.z())
    }

    #[inline(always)]
    fn rotation_matrix_as_column_major_slice(&self) -> [T; 9] {
        <[T; 9]>::try_from(self.to_rotation_matrix().matrix().as_slice()).unwrap()
    }

    #[inline(always)]
    fn from_rotation_matrix_as_column_major_slice(slice: &[T]) -> Self {
        let m = Matrix3::from_column_slice(slice);
        UnitQuaternion::from_matrix(&m)
    }

    #[inline(always)]
    fn unit_quaternion_as_wxyz_slice(&self) -> [T; 4] {
        [ self.w, self.i, self.j, self.k ]
    }

    #[inline(always)]
    fn from_unit_quaternion_as_wxyz_slice(slice: &[T]) -> Self {
        UnitQuaternion::from_quaternion(Quaternion::new(slice[0], slice[1], slice[2], slice[3]))
    }

    #[inline(always)]
    fn coordinate_frame_vectors(&self) -> [[T; 3]; 3] {
        let m = self.to_rotation_matrix();

        let vec1 = [ m[(0,0)], m[(1,0)], m[(2,0)] ];
        let vec2 = [ m[(0,1)], m[(1,1)], m[(2,1)] ];
        let vec3 = [ m[(0,2)], m[(1,2)], m[(2,2)] ];

        [ vec1, vec2, vec3 ]
    }

    #[inline(always)]
    fn inverse(&self) -> Self {
        self.inverse()
    }

    #[inline(always)]
    fn angle(&self) -> T {
        // self.w.acos() * T::constant(2.0)
        self.angle()
    }

    #[inline(always)]
    fn displacement(&self, other: &Self) -> Self {
        self.inverse() * other
    }

    #[inline(always)]
    fn dis(&self, other: &Self) -> T {
        let option1 = self.displacement(other).angle();
        let option2 = self.displacement(&other.conjugate()).angle();
        return option1.min(option2);
    }

    #[inline(always)]
    fn interpolate(&self, to: &Self, t: T) -> Self {
        self.slerp(to, t)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct O3DRotationCategoryUnitQuaternion;
impl O3DRotationCategoryTrait for O3DRotationCategoryUnitQuaternion {
    type R<T: AD> = UnitQuaternion<T>;
}

pub fn o3d_rotation_custom_serialize<S, T: AD, R: O3DRotation<T>>(value: &R, serializer: S) -> Result<S::Ok, S::Error> where S: serde::Serializer {
    let slice = value.scaled_axis_of_rotation();
    let slice_as_f64 = [ slice[0].to_constant(), slice[1].to_constant(), slice[2].to_constant() ];
    let mut tuple = serializer.serialize_tuple(3)?;
    for element in &slice_as_f64 {
        tuple.serialize_element(element)?;
    }
    tuple.end()
}

struct O3dRotationMyVisitor<T2: AD, R2: O3DRotation<T2>> {
    _phantom_data: PhantomData<(T2, R2)>
}

impl<'de, T2: AD, R2: O3DRotation<T2>> Visitor<'de> for O3dRotationMyVisitor<T2, R2> {
    type Value = R2;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a tuple of size 3")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let x: f64 = seq.next_element().expect("error").expect("error");
        let y: f64 = seq.next_element().expect("error").expect("error");
        let z: f64 = seq.next_element().expect("error").expect("error");
        let xad = T2::constant(x);
        let yad = T2::constant(y);
        let zad = T2::constant(z);
        Ok(R2::from_scaled_axis_of_rotation(&[xad, yad, zad]))
    }
}

pub fn o3d_rotation_custom_deserialize<'de, D, T: AD, R: O3DRotation<T>>(deserializer: D) -> Result<R, D::Error>
where
    D: Deserializer<'de>,
{
    deserializer.deserialize_tuple(3, O3dRotationMyVisitor::<T, R> { _phantom_data: PhantomData::default() })
}

pub struct SerdeO3DRotation<T: AD, R: O3DRotation<T>>(pub R, PhantomData<T>);

impl<T: AD, R: O3DRotation<T>> SerializeAs<R> for SerdeO3DRotation<T, R> {
    fn serialize_as<S>(source: &R, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
        o3d_rotation_custom_serialize(source, serializer)
    }
}
impl<'de, T: AD, R: O3DRotation<T>> DeserializeAs<'de, R> for SerdeO3DRotation<T, R> {
    fn deserialize_as<D>(deserializer: D) -> Result<R, D::Error> where D: Deserializer<'de> {
        o3d_rotation_custom_deserialize(deserializer)
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait O3DRotationConstructor<T: AD, TargetRotationType: O3DRotation<T>> {
    fn construct(&self) -> TargetRotationType;
}

/// This will be euler angles
impl<T, TargetRotationType> O3DRotationConstructor<T, TargetRotationType> for [T; 3]
    where T: AD,
          TargetRotationType: O3DRotation<T>
{
    fn construct(&self) -> TargetRotationType {
        TargetRotationType::from_euler_angles(self)
    }
}

/// This will be euler angles
impl<T, TargetRotationType> O3DRotationConstructor<T, TargetRotationType> for Vec<T>
    where T: AD,
          TargetRotationType: O3DRotation<T>
{
    fn construct(&self) -> TargetRotationType {
        assert_eq!(self.len(), 3);
        TargetRotationType::from_euler_angles(self)
    }
}

pub struct ScaledAxis<T: AD>(pub [T; 3]);
impl<T, TargetRotationType> O3DRotationConstructor<T, TargetRotationType> for ScaledAxis<T>
    where T: AD,
          TargetRotationType: O3DRotation<T>
{
    fn construct(&self) -> TargetRotationType {
        TargetRotationType::from_scaled_axis_of_rotation(&self.0)
    }
}

impl<T, TargetRotationType> O3DRotationConstructor<T, TargetRotationType> for Rotation3<T>
    where T: AD,
          TargetRotationType: O3DRotation<T>
{
    fn construct(&self) -> TargetRotationType {
        TargetRotationType::from_rotation_matrix_as_column_major_slice(self.matrix().as_slice())
    }
}

impl<T, TargetRotationType> O3DRotationConstructor<T, TargetRotationType> for UnitQuaternion<T>
    where T: AD,
          TargetRotationType: O3DRotation<T>
{
    fn construct(&self) -> TargetRotationType {
        TargetRotationType::from_unit_quaternion_as_wxyz_slice(self.coords.as_slice())
    }
}

pub struct QuatConstructor<T: AD>{pub w: T, pub x: T, pub y: T, pub z: T}
impl<T: AD> QuatConstructor<T> {
    pub fn new(w: T, x: T, y: T, z: T) -> Self {
        Self { w, x, y, z }
    }
    pub fn new_from_wxyz_ovec<V: OVec<T>>(v: &V) -> Self {
        assert_eq!(v.len(), 4);

        Self::new(*v.ovec_get_element(0), *v.ovec_get_element(1), *v.ovec_get_element(2), *v.ovec_get_element(3))
    }
}
impl<T, TargetRotationType> O3DRotationConstructor<T, TargetRotationType> for QuatConstructor<T>
    where T: AD,
          TargetRotationType: O3DRotation<T> {
    fn construct(&self) -> TargetRotationType {
        let quaternion = Quaternion::new(self.w, self.x, self.y, self.z);
        let unit_quaternion = UnitQuaternion::from_quaternion(quaternion);
        let axis = unit_quaternion.scaled_axis();
        TargetRotationType::from_scaled_axis_of_rotation(&axis)
    }
}

