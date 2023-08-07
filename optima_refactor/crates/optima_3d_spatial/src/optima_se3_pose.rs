use std::fmt::Debug;
use std::marker::PhantomData;
use ad_trait::{AD, NalgebraMatMulAD};
use nalgebra::{ArrayStorage, Const, UnitQuaternion, Vector3, Vector6};
use crate::optima_3d_vec::Optima3DVec;
use crate::optima_3d_rotation::{Optima3DRotation, Optima3DRotationConstructor};

pub trait OptimaSE3Pose<'a, T: AD> :
    Debug
{
    type RotationType: Optima3DRotation<'a, T>;

    fn location(&self) -> &<Self::RotationType as Optima3DRotation<'a, T>>::Native3DVecType;
    fn orientation(&self) -> &Self::RotationType;
    fn update_location(&mut self, translation: &[T]);
    fn update_orientation<RC: Optima3DRotationConstructor<'a, T, Self::RotationType>>(&mut self, orientation: &RC);
    fn update_orientation_native(&mut self, orientation: &Self::RotationType);
    fn update_orientation_direct<R: Optima3DRotation<'a, T>>(&mut self, orientation: &R);
    fn mul(&self, other: &Self) -> Self;
    fn inverse(&self) -> Self;
    fn displacement(&self, other: &Self) -> Self;
    fn dis(&self, other: &Self) -> T;
    fn interpolate(&self, to: &Self, t: T) -> Self;
}

#[derive(Debug)]
pub struct ImplicitDualQuaternion<'a, T: AD + NalgebraMatMulAD<'a, Const<3>, Const<1>, ArrayStorage<T, 3, 1>>> {
    location: Vector3<T>,
    orientation: UnitQuaternion<T>,
    _p: PhantomData<&'a T>
}

impl<'a, T: AD + NalgebraMatMulAD<'a, Const<3>, Const<1>, ArrayStorage<T, 3, 1>>> ImplicitDualQuaternion<'a, T>
{
    pub fn new<P, RC>(location: &P, orientation: &RC) -> Self
        where P: Optima3DVec<'a, T>,
              RC: Optima3DRotationConstructor<'a, T, UnitQuaternion<T>>
    {
        let location = Vector3::from_column_slice(location.as_slice());
        let orientation= orientation.construct();

        Self {
            location,
            orientation,
            _p: Default::default()
        }
    }

    pub fn ln(&self) -> Vector6<T> {
        let h_v = Vector3::new( self.orientation.i,self.orientation.j, self.orientation.k  );
        let s: T = h_v.norm();
        let c = self.orientation.w;
        let phi = s.atan2(c);
        let mut a = T::zero();
        if s > T::zero() { a = phi / s; }
        let rot_vec_diff = a * h_v;

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

        let translation_diff = mu_d * tmp.dot(&rot_vec_diff)  * rot_vec_diff + mu_r * tmp + tmp.cross(&rot_vec_diff);

        let out_vec = Vector6::new(rot_vec_diff[0], rot_vec_diff[1], rot_vec_diff[2], translation_diff[0], translation_diff[1], translation_diff[2]);

        out_vec
    }
}

impl<'a, T: AD + NalgebraMatMulAD<'a, Const<3>, Const<1>, ArrayStorage<T, 3, 1>>> OptimaSE3Pose<'a, T> for ImplicitDualQuaternion<'a, T>
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

    fn update_orientation<RC: Optima3DRotationConstructor<'a, T, UnitQuaternion<T>>>(&mut self, orientation: &RC) {
        self.orientation = orientation.construct();
    }

    fn update_orientation_native(&mut self, orientation: &UnitQuaternion<T>) {
        self.orientation = orientation.clone();
    }

    fn update_orientation_direct<R: Optima3DRotation<'a, T>>(&mut self, orientation: &R) {
        self.orientation = UnitQuaternion::from_scaled_axis(Vector3::from_column_slice(&orientation.scaled_axis_of_rotation()));
    }

    fn mul(&self, other: &Self) -> Self {
        let orientation = &self.orientation * &other.orientation;
        let location = &self.orientation * &other.location + &self.location;

        Self {
            location,
            orientation,
            _p: Default::default()
        }
    }

    fn inverse(&self) -> Self {
        let orientation = self.orientation.inverse();
        let location = &orientation * -&self.location;

        Self {
            location,
            orientation,
            _p: Default::default()
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
        let location = (T::one() - t) * self.location + t * to.location;

        Self {
            location,
            orientation,
            _p: Default::default()
        }
    }
}
