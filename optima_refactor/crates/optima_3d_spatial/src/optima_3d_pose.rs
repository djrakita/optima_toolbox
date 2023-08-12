use std::fmt;
use std::fmt::Debug;
use std::marker::PhantomData;
use ad_trait::{AD};
use nalgebra::{UnitQuaternion, Vector3, Vector6};
use serde::{Deserialize, Deserializer, Serialize};
use serde::de::{SeqAccess, Visitor};
use serde::ser::SerializeTuple;
use crate::optima_3d_vec::O3DVec;
use crate::optima_3d_rotation::{O3DRotation, O3DRotationConstructor};

pub trait O3DPose<T: AD> :
    Debug + Serialize + for<'a> Deserialize<'a>
{
    type RotationType: O3DRotation<T>;

    fn from_translation_and_rotation<V: O3DVec<T>, R: O3DRotation<T>>(translation: &V, rotation: &R) -> Self;
    fn from_translation_and_rotation_constructor<V: O3DVec<T>, RC: O3DRotationConstructor<T, Self::RotationType>>(translation: &V, rotation_constructor: &RC) -> Self;
    fn translation(&self) -> &<Self::RotationType as O3DRotation<T>>::Native3DVecType;
    fn rotation(&self) -> &Self::RotationType;
    fn update_translation(&mut self, translation: &[T]);
    fn update_rotation<RC: O3DRotationConstructor<T, Self::RotationType>>(&mut self, orientation: &RC);
    fn update_rotation_native(&mut self, orientation: &Self::RotationType);
    fn update_rotation_direct<R: O3DRotation<T>>(&mut self, orientation: &R);
    fn mul(&self, other: &Self) -> Self;
    fn inverse(&self) -> Self;
    fn displacement(&self, other: &Self) -> Self;
    fn dis(&self, other: &Self) -> T;
    fn interpolate(&self, to: &Self, t: T) -> Self;
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ImplicitDualQuaternion<T: AD> {
    #[serde(deserialize_with = "Vector3::<T>::deserialize")]
    translation: Vector3<T>,
    #[serde(deserialize_with = "UnitQuaternion::<T>::deserialize")]
    rotation: UnitQuaternion<T>
}

impl<T: AD> ImplicitDualQuaternion<T>
{
    pub fn new<V, RC>(translation: &V, rotation_constructor: &RC) -> Self
        where V: O3DVec<T>,
              RC: O3DRotationConstructor<T, UnitQuaternion<T>>
    {
        let translation = Vector3::from_column_slice(translation.as_slice());
        let rotation = rotation_constructor.construct();

        Self {
            translation,
            rotation,
        }
    }

    pub fn ln(&self) -> Vector6<T> {
        let h_v = Vector3::new(self.rotation.i, self.rotation.j, self.rotation.k  );
        let s: T = h_v.norm();
        let c = self.rotation.w;
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

        let tmp = &self.translation / T::constant(2.0);

        let translation_diff = (mu_d * tmp.dot(&rot_vec_diff)).mul_by_nalgebra_matrix_ref(&rot_vec_diff)  + mu_r.mul_by_nalgebra_matrix_ref(&tmp) + tmp.cross(&rot_vec_diff);

        let out_vec = Vector6::new(rot_vec_diff[0], rot_vec_diff[1], rot_vec_diff[2], translation_diff[0], translation_diff[1], translation_diff[2]);

        out_vec
    }
}

impl<T: AD> O3DPose<T> for ImplicitDualQuaternion<T>
{
    type RotationType = UnitQuaternion<T>;

    fn from_translation_and_rotation<V: O3DVec<T>, R: O3DRotation<T>>(location: &V, orientation: &R) -> Self {
        let location = Vector3::from_column_slice(location.as_slice());
        let orientation = UnitQuaternion::from_scaled_axis(Vector3::from_column_slice(&orientation.scaled_axis_of_rotation()));
        Self {
            translation: location,
            rotation: orientation
        }
    }

    fn from_translation_and_rotation_constructor<V: O3DVec<T>, RC: O3DRotationConstructor<T, Self::RotationType>>(translation: &V, rotation_constructor: &RC) -> Self {
        Self::new(translation, rotation_constructor)
    }

    fn translation(&self) -> &Vector3<T> {
        &self.translation
    }

    fn rotation(&self) -> &UnitQuaternion<T> {
        &self.rotation
    }

    fn update_translation(&mut self, translation: &[T]) {
        self.translation = Vector3::from_column_slice(translation);
    }

    fn update_rotation<RC: O3DRotationConstructor<T, UnitQuaternion<T>>>(&mut self, orientation: &RC) {
        self.rotation = orientation.construct();
    }

    fn update_rotation_native(&mut self, orientation: &UnitQuaternion<T>) {
        self.rotation = orientation.clone();
    }

    fn update_rotation_direct<R: O3DRotation<T>>(&mut self, orientation: &R) {
        self.rotation = UnitQuaternion::from_scaled_axis(Vector3::from_column_slice(&orientation.scaled_axis_of_rotation()));
    }

    fn mul(&self, other: &Self) -> Self {
        let orientation = &self.rotation * &other.rotation;
        let location = &self.rotation * &other.translation + &self.translation;

        Self {
            translation: location,
            rotation: orientation,
        }
    }

    fn inverse(&self) -> Self {
        let orientation = self.rotation.inverse();
        let location = &orientation * -&self.translation;

        Self {
            translation: location,
            rotation: orientation,
        }
    }

    fn displacement(&self, other: &Self) -> Self {
        self.inverse().mul(other)
    }

    fn dis(&self, other: &Self) -> T {
        let l = self.displacement(other).ln();
        l.norm()
    }

    fn interpolate(&self, to: &Self, t: T) -> Self {
        let orientation = self.rotation.slerp(&to.rotation, t);
        let location = (T::one() - t).mul_by_nalgebra_matrix_ref(&self.translation) + t.mul_by_nalgebra_matrix_ref(&to.translation);

        Self {
            translation: location,
            rotation: orientation,
        }
    }
}

pub fn o3d_pose_custom_serialize<S, T: AD, P: O3DPose<T>>(value: &P, serializer: S) -> Result<S::Ok, S::Error> where S: serde::Serializer {
    let translation_slice = value.translation().as_slice();
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