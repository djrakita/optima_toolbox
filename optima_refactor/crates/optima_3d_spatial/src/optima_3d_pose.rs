use std::borrow::Cow;
use std::fmt;
use std::fmt::Debug;
use std::marker::PhantomData;
use ad_trait::AD;
use as_any::AsAny;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3, Vector6};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde::de::{SeqAccess, Visitor};
use serde::ser::SerializeTuple;
use serde_with::{DeserializeAs, SerializeAs};
use optima_linalg::OVec;
use crate::optima_3d_vec::{O3DVec, O3DVecCategoryArr};
use crate::optima_3d_rotation::{O3DRotation, O3DRotationConstructor, ScaledAxis};

#[derive(Clone, Debug, Copy, Eq, PartialEq)]
pub enum O3DPoseType {
    ImplicitDualQuaternion, NalgebraIsometry3
}

pub trait O3DPoseCategory:
    Clone + Debug + Serialize + for<'a> Deserialize<'a> + Send + Sync
{
    type P<T: AD> : O3DPose<T>;
}

pub trait AliasO3DPoseCategory: O3DPoseCategory + 'static { }
impl<A> AliasO3DPoseCategory for A where A : O3DPoseCategory + 'static { }

pub trait O3DPose<T: AD> :
    Clone + Debug + Serialize + for<'a> Deserialize<'a> + Send + Sync + AsAny
{
    type Category: O3DPoseCategory;
    type RotationType: O3DRotation<T>;
    type LieAlgebraType : OVec<T>;

    fn type_identifier() -> O3DPoseType;
    fn identity() -> Self;
    fn from_translation_and_rotation<V: O3DVec<T>, R: O3DRotation<T>>(translation: &V, rotation: &R) -> Self;
    fn from_constructors<V: O3DVec<T>, RC: O3DRotationConstructor<T, Self::RotationType>>(translation: &V, rotation_constructor: &RC) -> Self;
    fn translation(&self) -> &<Self::RotationType as O3DRotation<T>>::Native3DVecType;
    fn rotation(&self) -> &Self::RotationType;
    fn update_translation(&mut self, translation: &[T]);
    fn update_rotation_constructor<RC: O3DRotationConstructor<T, Self::RotationType>>(&mut self, rotation: &RC);
    fn update_rotation_native(&mut self, rotation: &Self::RotationType);
    fn update_rotation_direct<R: O3DRotation<T>>(&mut self, rotation: &R);
    fn mul(&self, other: &Self) -> Self;
    fn mul_by_point_native(&self, point: &<Self::RotationType as O3DRotation<T>>::Native3DVecType) -> <Self::RotationType as O3DRotation<T>>::Native3DVecType {
        self.rotation().mul_by_point_native(point).o3dvec_add(self.translation())
    }
    fn mul_by_point_generic<V: O3DVec<T>>(&self, point: &V) -> V {
        let tmp = <Self::RotationType as O3DRotation<T>>::Native3DVecType::o3dvec_from_slice(point.o3dvec_as_slice());
        let res = self.mul_by_point_native(&tmp);
        V::o3dvec_from_slice(res.o3dvec_as_slice())
    }
    fn inverse(&self) -> Self;
    fn displacement(&self, other: &Self) -> Self;
    fn magnitude(&self) -> T;
    fn dis(&self, other: &Self) -> T;
    fn interpolate(&self, to: &Self, t: T) -> Self;
    fn ln(&self) -> Self::LieAlgebraType;
    fn exp(lie: &Self::LieAlgebraType) -> Self;
    #[inline(always)]
    fn interpolate_with_separate_max_translation_and_rotation(&self, to: &Self, max_translation: T, max_rotation: T) -> Self {
        let t_disp = to.translation().o3dvec_sub(self.translation()).o3dvec_to_other_generic_category::<T, O3DVecCategoryArr>();
        let r_disp = self.rotation().displacement(to.rotation()).scaled_axis_of_rotation();

        let t_n = t_disp.norm().max(T::constant(0.0000001));
        let r_n = r_disp.norm().max(T::constant(0.0000001));

        let binding = t_disp.ovec_scalar_div(&t_n).ovec_scalar_mul(& t_n.min(max_translation) );
        let t_disp_new = binding.o3dvec_downcast_or_convert::< <Self::RotationType as O3DRotation<T>>::Native3DVecType  >();
        let r_disp_new = r_disp.ovec_scalar_div(&r_n).ovec_scalar_mul(& r_n.min(max_rotation) );

        let new_translation = self.translation().o3dvec_add(t_disp_new.as_ref());
        let new_rotation = self.rotation().mul( &Self::RotationType::from_scaled_axis_of_rotation(&r_disp_new) );

        Self::from_translation_and_rotation(&new_translation, &new_rotation)
    }
    #[inline(always)]
    fn interpolate_with_combined_max_translation_and_rotation(&self, to: &Self, max_translation_and_rotation: T) -> Self {
        let disp = to.displacement(self);
        let ln = disp.ln();
        let n = ln.ovec_p_norm(&T::constant(2.0)).max(T::constant(0.000001));
        let new_ln = ln.ovec_scalar_div(&n).ovec_scalar_mul(&n.min(max_translation_and_rotation));
        self.mul(&Self::exp(&new_ln))
    }
    #[inline(always)]
    fn o3dpose_to_constant_ads(&self) -> Self {
        let translation = self.translation().o3dvec_to_constant_ads();
        let rotation = self.rotation().o3drot_to_constant_ads();
        Self::from_translation_and_rotation(&translation, &rotation)
    }
    fn o3dpose_to_other_generic_category<T2: AD, C: O3DPoseCategory>(&self) -> C::P<T2> {
        let translation_slice = self.translation().o3dvec_as_slice();
        let binding = self.rotation().scaled_axis_of_rotation();
        let rotation_slice = binding.as_slice();
        let s = [
            translation_slice[0].to_constant(),
            translation_slice[1].to_constant(),
            translation_slice[2].to_constant(),
            rotation_slice[0].to_constant(),
            rotation_slice[1].to_constant(),
            rotation_slice[2].to_constant()
        ];
        C::P::from_constructors( &[T2::constant(s[0]), T2::constant(s[1]), T2::constant(s[2])], &ScaledAxis([T2::constant(s[3]), T2::constant(s[4]), T2::constant(s[5])]))
    }
    fn o3dpose_to_other_ad_type<T2: AD>(&self) -> <Self::Category as O3DPoseCategory>::P::<T2> {
        self.o3dpose_to_other_generic_category::<T2, Self::Category>()
    }
    #[inline(always)]
    fn o3dpose_downcast_or_convert<P: O3DPose<T>>(&self) -> Cow<P> {
        let downcast = self.as_any().downcast_ref::<P>();
        match downcast {
            Some(d) => { Cow::Borrowed(d) }
            None => {
                let out = P::from_translation_and_rotation(self.translation(), self.rotation());
                Cow::Owned(out)
            }
        }
    }
}

impl<T: AD> O3DPose<T> for ImplicitDualQuaternion<T>
{
    type Category = O3DPoseCategoryImplicitDualQuaternion;
    type RotationType = UnitQuaternion<T>;
    type LieAlgebraType = Vector6<T>;

    #[inline(always)]
    fn type_identifier() -> O3DPoseType {
        O3DPoseType::ImplicitDualQuaternion
    }

    #[inline(always)]
    fn identity() -> Self {
        Self::from_constructors(&[T::zero(), T::zero(), T::zero()], &[T::zero(), T::zero(), T::zero()])
    }

    #[inline(always)]
    fn from_translation_and_rotation<V: O3DVec<T>, R: O3DRotation<T>>(location: &V, orientation: &R) -> Self {
        let location = Vector3::from_column_slice(location.o3dvec_as_slice());
        let orientation = UnitQuaternion::from_scaled_axis(Vector3::from_column_slice(&orientation.scaled_axis_of_rotation()));
        Self {
            translation: location,
            rotation: orientation
        }
    }

    #[inline(always)]
    fn from_constructors<V: O3DVec<T>, RC: O3DRotationConstructor<T, Self::RotationType>>(translation: &V, rotation_constructor: &RC) -> Self {
        let translation = Vector3::from_column_slice(translation.o3dvec_as_slice());
        let rotation = rotation_constructor.construct();

        Self {
            translation,
            rotation,
        }
    }

    #[inline(always)]
    fn translation(&self) -> &Vector3<T> {
        &self.translation
    }

    #[inline(always)]
    fn rotation(&self) -> &UnitQuaternion<T> {
        &self.rotation
    }

    #[inline(always)]
    fn update_translation(&mut self, translation: &[T]) {
        self.translation = Vector3::from_column_slice(translation);
    }

    #[inline(always)]
    fn update_rotation_constructor<RC: O3DRotationConstructor<T, UnitQuaternion<T>>>(&mut self, orientation: &RC) {
        self.rotation = orientation.construct();
    }

    #[inline(always)]
    fn update_rotation_native(&mut self, orientation: &UnitQuaternion<T>) {
        self.rotation = orientation.clone();
    }

    #[inline(always)]
    fn update_rotation_direct<R: O3DRotation<T>>(&mut self, orientation: &R) {
        self.rotation = UnitQuaternion::from_scaled_axis(Vector3::from_column_slice(&orientation.scaled_axis_of_rotation()));
    }

    #[inline(always)]
    fn mul(&self, other: &Self) -> Self {
        let orientation = &self.rotation * &other.rotation;
        let location = &self.rotation * &other.translation + &self.translation;

        Self {
            translation: location,
            rotation: orientation,
        }
    }

    #[inline(always)]
    fn inverse(&self) -> Self {
        let orientation = self.rotation.inverse();
        let location = &orientation * -&self.translation;

        Self {
            translation: location,
            rotation: orientation,
        }
    }

    #[inline(always)]
    fn displacement(&self, other: &Self) -> Self {
        self.inverse().mul(other)
    }

    #[inline(always)]
    fn magnitude(&self) -> T {
        let ln = self.ln();
        ln.norm()
    }

    #[inline(always)]
    fn dis(&self, other: &Self) -> T {
        let l = self.displacement(other).ln();
        l.norm()
    }

    #[inline(always)]
    fn interpolate(&self, to: &Self, t: T) -> Self {
        let orientation = self.rotation.slerp(&to.rotation, t);
        let location = (T::one() - t).mul_by_nalgebra_matrix_ref(&self.translation) + t.mul_by_nalgebra_matrix_ref(&to.translation);

        Self {
            translation: location,
            rotation: orientation,
        }
    }

    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraType {
        generic_pose_ln(&self.translation, &self.rotation)
    }

    #[inline(always)]
    fn exp(lie: &Self::LieAlgebraType) -> Self {
        let (t, r) = generic_pose_exp(&lie);
        Self::from_translation_and_rotation(&t, &r)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct O3DPoseCategoryImplicitDualQuaternion;
impl O3DPoseCategory for O3DPoseCategoryImplicitDualQuaternion {
    type P<T: AD> = ImplicitDualQuaternion<T>;
}

impl<T: AD> O3DPose<T> for Isometry3<T> {
    type Category = O3DPoseCategoryIsometry3;
    type RotationType = UnitQuaternion<T>;
    type LieAlgebraType = Vector6<T>;

    fn type_identifier() -> O3DPoseType {
        O3DPoseType::NalgebraIsometry3
    }

    #[inline(always)]
    fn identity() -> Self {
        Self::from_constructors(&[T::zero(), T::zero(), T::zero()], &[T::zero(), T::zero(), T::zero()])
    }

    fn from_translation_and_rotation<V: O3DVec<T>, R: O3DRotation<T>>(translation: &V, rotation: &R) -> Self {
        Isometry3::from_parts(Translation3::new(translation.x(), translation.y(), translation.z()), UnitQuaternion::from_scaled_axis(Vector3::from_column_slice(&rotation.scaled_axis_of_rotation())))
    }

    fn from_constructors<V: O3DVec<T>, RC: O3DRotationConstructor<T, Self::RotationType>>(translation: &V, rotation_constructor: &RC) -> Self {
        let rotation = rotation_constructor.construct();
        Self::from_translation_and_rotation(translation, &rotation)
    }

    #[inline(always)]
    fn translation(&self) -> &<Self::RotationType as O3DRotation<T>>::Native3DVecType {
        &self.translation.vector
    }

    #[inline(always)]
    fn rotation(&self) -> &Self::RotationType {
        &self.rotation
    }

    fn update_translation(&mut self, translation: &[T]) {
        self.translation = Vector3::from_column_slice(translation).into();
    }

    fn update_rotation_constructor<RC: O3DRotationConstructor<T, Self::RotationType>>(&mut self, orientation: &RC) {
        self.rotation = orientation.construct();
    }

    fn update_rotation_native(&mut self, orientation: &Self::RotationType) {
        self.rotation = orientation.clone();
    }

    fn update_rotation_direct<R: O3DRotation<T>>(&mut self, orientation: &R) {
        self.rotation = UnitQuaternion::from_scaled_axis(Vector3::from_column_slice(&orientation.scaled_axis_of_rotation()));
    }

    #[inline(always)]
    fn mul(&self, other: &Self) -> Self {
        self * other
    }

    #[inline(always)]
    fn inverse(&self) -> Self {
        self.inverse()
    }

    #[inline(always)]
    fn displacement(&self, other: &Self) -> Self {
        self.inverse() * other
    }

    #[inline(always)]
    fn magnitude(&self) -> T {
        generic_pose_ln(&self.translation.vector, &self.rotation).norm()
    }

    #[inline(always)]
    fn dis(&self, other: &Self) -> T {
        let disp = self.displacement(other);
        generic_pose_ln(&disp.translation.vector, &disp.rotation).norm()
    }

    #[inline(always)]
    fn interpolate(&self, to: &Self, t: T) -> Self {
        self.lerp_slerp(to, t)
    }

    #[inline(always)]
    fn ln(&self) -> Self::LieAlgebraType {
        generic_pose_ln(&self.translation.vector, &self.rotation)
    }

    #[inline(always)]
    fn exp(lie: &Self::LieAlgebraType) -> Self {
        let (t, r) = generic_pose_exp(&lie);
        Self::from_translation_and_rotation(&t, &r)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct O3DPoseCategoryIsometry3;
impl O3DPoseCategory for O3DPoseCategoryIsometry3 {
    type P<T: AD> = Isometry3<T>;
}

pub trait O3DLieAlgebraPose<T: AD> : O3DPose<T> {
    type LnVecType;

    fn ln(&self) -> Self::LnVecType;
    fn exp(ln_vec: &Self::LnVecType) -> Self;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ImplicitDualQuaternion<T: AD> {
    #[serde(deserialize_with = "Vector3::<T>::deserialize")]
    translation: Vector3<T>,
    #[serde(deserialize_with = "UnitQuaternion::<T>::deserialize")]
    rotation: UnitQuaternion<T>
}

fn generic_pose_ln<T: AD>(translation: &Vector3<T>, rotation: &UnitQuaternion<T>) -> Vector6<T> {
    let h_v = Vector3::new(rotation.i, rotation.j, rotation.k);
    let s: T = h_v.norm();
    let c = rotation.w;
    let phi = s.atan2(c);
    let mut a = T::zero();
    if s > T::zero() { a = phi / s; }
    // let rot_vec_diff = a * h_v;
    let rot_vec_diff = a.mul_by_nalgebra_matrix_ref(&h_v);

    let mu_r;
    let mu_d;

    if s < T::constant(0.00000000000001) {
        mu_r = T::one() - (phi.powi(2) / T::constant(3.0)) - (phi.powi(4) / T::constant(45.0));
    } else {
        mu_r = (c * phi) / s;
    }

    if phi < T::constant(0.00000000000001) {
        mu_d = (T::one() / T::constant(3.0)) + (phi.powi(2) / T::constant(45.0)) + ((T::constant(2.0) * phi.powi(4)) / T::constant(945.0));
    } else {
        mu_d = (T::one() - mu_r) / (phi.powi(2));
    }

    let tmp = translation / T::constant(2.0);

    let translation_diff = (mu_d * tmp.dot(&rot_vec_diff)).mul_by_nalgebra_matrix_ref(&rot_vec_diff) + mu_r.mul_by_nalgebra_matrix_ref(&tmp) + tmp.cross(&rot_vec_diff);

    let out_vec = Vector6::new(rot_vec_diff[0], rot_vec_diff[1], rot_vec_diff[2], translation_diff[0], translation_diff[1], translation_diff[2]);

    out_vec
}

fn generic_pose_exp<T: AD>(ln_vec: &Vector6<T>) -> (Vector3<T>, UnitQuaternion<T>) {
    let w = Vector3::new(ln_vec[0], ln_vec[1], ln_vec[2]);
    let v = Vector3::new(ln_vec[3], ln_vec[4], ln_vec[5]);

    let phi = w.norm();
    let s = phi.sin();
    let c = phi.cos();
    let gamma = w.dot(&v);

    let mu_r;
    let mu_d;

    if phi < T::constant(0.00000001) {
        mu_r = T::constant(1.0) - phi.powi(2) / T::constant(6.0) + phi.powi(4) / T::constant(120.0);
        mu_d = T::constant(4.0 / 3.0) - T::constant(4.0) * phi.powi(2) / T::constant(15.0) + T::constant(8.0) * phi.powi(4) / T::constant(315.0);
    } else {
        mu_r = s / phi;
        mu_d = (T::constant(2.0) - c * (T::constant(2.0) * mu_r)) / phi.powi(2);
    }

    let h_v: Vector3<T> = mu_r.mul_by_nalgebra_matrix_ref(&w);
    let quat_ = Quaternion::new(c, h_v[0], h_v[1], h_v[2]);
    let rotation = UnitQuaternion::from_quaternion(quat_);

    let translation = T::constant(2.0).mul_by_nalgebra_matrix_ref(&mu_r.mul_by_nalgebra_matrix_ref(&h_v.cross(&v))) + (c * T::constant(2.0) * mu_r).mul_by_nalgebra_matrix_ref(&v) + (mu_d * gamma).mul_by_nalgebra_matrix_ref(&w);

    return (translation, rotation);
}

impl<T: AD> ImplicitDualQuaternion<T>
{
    pub fn ln(&self) -> Vector6<T> {
        generic_pose_ln(&self.translation, &self.rotation)
    }
    pub fn exp(ln_vec: &Vector6<T>) -> Self {
        let res = generic_pose_exp(ln_vec);
        Self {
            translation: res.0,
            rotation: res.1,
        }
    }
}

pub fn o3d_pose_custom_serialize<S, T: AD, P: O3DPose<T>>(value: &P, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
    let translation_slice = value.translation().o3dvec_as_slice();
    let binding = value.rotation().scaled_axis_of_rotation();
    let rotation_slice = binding.as_slice();
    let slice_as_f64 = [
        translation_slice[0].to_constant(),
        translation_slice[1].to_constant(),
        translation_slice[2].to_constant(),
        rotation_slice[0].to_constant(),
        rotation_slice[1].to_constant(),
        rotation_slice[2].to_constant()
    ];
    let mut tuple = serializer.serialize_tuple(6)?;
    for element in &slice_as_f64 {
        tuple.serialize_element(element)?;
    }
    tuple.end()
}

struct O3dPoseMyVisitor<T2: AD, P2: O3DPose<T2>> {
    _phantom_data: PhantomData<(T2, P2)>
}

impl<'de, T2: AD, P2: O3DPose<T2>> Visitor<'de> for O3dPoseMyVisitor<T2, P2> {
    type Value = P2;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a tuple of size 6")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let x: f64 = seq.next_element().expect("error").expect("error");
        let y: f64 = seq.next_element().expect("error").expect("error");
        let z: f64 = seq.next_element().expect("error").expect("error");
        let rx: f64 = seq.next_element().expect("error").expect("error");
        let ry: f64 = seq.next_element().expect("error").expect("error");
        let rz: f64 = seq.next_element().expect("error").expect("error");
        let xad = T2::constant(x);
        let yad = T2::constant(y);
        let zad = T2::constant(z);
        let rxad = T2::constant(rx);
        let ryad = T2::constant(ry);
        let rzad = T2::constant(rz);

        let translation = [xad, yad, zad];
        let rotation = P2::RotationType::from_scaled_axis_of_rotation(&[rxad, ryad, rzad]);

        Ok(P2::from_translation_and_rotation(&translation, &rotation))
    }
}

pub fn o3d_pose_custom_deserialize<'de, D, T: AD, P: O3DPose<T>>(deserializer: D) -> Result<P, D::Error>
where
    D: Deserializer<'de>,
{
    deserializer.deserialize_tuple(6, O3dPoseMyVisitor::<T, P> { _phantom_data: PhantomData::default() })
}

pub struct SerdeO3DPose<T: AD, P: O3DPose<T>>(pub P, PhantomData<T>);

impl<T: AD, P: O3DPose<T>> SerializeAs<P> for SerdeO3DPose<T, P> {
    fn serialize_as<S>(source: &P, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
        o3d_pose_custom_serialize(source, serializer)
    }
}
impl<'de, T: AD, P: O3DPose<T>> DeserializeAs<'de, P> for SerdeO3DPose<T, P> {
    fn deserialize_as<D>(deserializer: D) -> Result<P, D::Error> where D: Deserializer<'de> {
        o3d_pose_custom_deserialize(deserializer)
    }
}