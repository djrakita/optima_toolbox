use std::fmt::Debug;
use ad_trait::{AD};
use nalgebra::{UnitQuaternion, Vector3, Vector6};
use serde::{Deserialize, Serialize};
use crate::optima_3d_vec::O3DVec;
use crate::optima_3d_rotation::{O3DRotation, O3DRotationConstructor};

pub trait OSE3Pose<T: AD> :
    Debug + Serialize + for<'a> Deserialize<'a>
{
    type RotationType: O3DRotation<T>;

    fn location(&self) -> &<Self::RotationType as O3DRotation<T>>::Native3DVecType;
    fn orientation(&self) -> &Self::RotationType;
    fn update_location(&mut self, translation: &[T]);
    fn update_orientation<RC: O3DRotationConstructor<T, Self::RotationType>>(&mut self, orientation: &RC);
    fn update_orientation_native(&mut self, orientation: &Self::RotationType);
    fn update_orientation_direct<R: O3DRotation<T>>(&mut self, orientation: &R);
    fn mul(&self, other: &Self) -> Self;
    fn inverse(&self) -> Self;
    fn displacement(&self, other: &Self) -> Self;
    fn dis(&self, other: &Self) -> T;
    fn interpolate(&self, to: &Self, t: T) -> Self;
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ImplicitDualQuaternion<T: AD> {
    #[serde(deserialize_with = "Vector3::<T>::deserialize")]
    location: Vector3<T>,
    #[serde(deserialize_with = "UnitQuaternion::<T>::deserialize")]
    orientation: UnitQuaternion<T>
}

impl<T: AD> ImplicitDualQuaternion<T>
{
    pub fn new<P, RC>(location: &P, orientation: &RC) -> Self
        where P: O3DVec<T>,
              RC: O3DRotationConstructor<T, UnitQuaternion<T>>
    {
        let location = Vector3::from_column_slice(location.as_slice());
        let orientation= orientation.construct();

        Self {
            location,
            orientation,
        }
    }

    pub fn ln(&self) -> Vector6<T> {
        let h_v = Vector3::new( self.orientation.i,self.orientation.j, self.orientation.k  );
        let s: T = h_v.norm();
        let c = self.orientation.w;
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

        let tmp = &self.location / T::constant(2.0);

        let translation_diff = (mu_d * tmp.dot(&rot_vec_diff)).mul_by_nalgebra_matrix_ref(&rot_vec_diff)  + mu_r.mul_by_nalgebra_matrix_ref(&tmp) + tmp.cross(&rot_vec_diff);

        let out_vec = Vector6::new(rot_vec_diff[0], rot_vec_diff[1], rot_vec_diff[2], translation_diff[0], translation_diff[1], translation_diff[2]);

        out_vec
    }
}

impl<T: AD> OSE3Pose<T> for ImplicitDualQuaternion<T>
{
    type RotationType = UnitQuaternion<T>;

    fn location(&self) -> &Vector3<T> {
        &self.location
    }

    fn orientation(&self) -> &UnitQuaternion<T> {
        &self.orientation
    }

    fn update_location(&mut self, translation: &[T]) {
        self.location = Vector3::from_column_slice(translation);
    }

    fn update_orientation<RC: O3DRotationConstructor<T, UnitQuaternion<T>>>(&mut self, orientation: &RC) {
        self.orientation = orientation.construct();
    }

    fn update_orientation_native(&mut self, orientation: &UnitQuaternion<T>) {
        self.orientation = orientation.clone();
    }

    fn update_orientation_direct<R: O3DRotation<T>>(&mut self, orientation: &R) {
        self.orientation = UnitQuaternion::from_scaled_axis(Vector3::from_column_slice(&orientation.scaled_axis_of_rotation()));
    }

    fn mul(&self, other: &Self) -> Self {
        let orientation = &self.orientation * &other.orientation;
        let location = &self.orientation * &other.location + &self.location;

        Self {
            location,
            orientation,
        }
    }

    fn inverse(&self) -> Self {
        let orientation = self.orientation.inverse();
        let location = &orientation * -&self.location;

        Self {
            location,
            orientation,
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
        let orientation = self.orientation.slerp(&to.orientation, t);
        let location = (T::one() - t).mul_by_nalgebra_matrix_ref(&self.location) + t.mul_by_nalgebra_matrix_ref(&to.location);

        Self {
            location,
            orientation,
        }
    }
}
