use std::fmt::{Debug};
use ad_trait::{AD, NalgebraMatMulAD};
use nalgebra::{ArrayStorage, Const, Vector3};

pub trait Optima3DVec<'a, T: AD> :
    Debug
{
    fn x(&self) -> T;
    fn y(&self) -> T;
    fn z(&self) -> T;
    fn from_slice(slice: &[T]) -> Self;
    fn as_slice(&self) -> &[T];
    fn add(&self, other: &Self) -> Self;
    fn sub(&self, other: &Self) -> Self;
    fn scalar_mul(&'a self, scalar: T) -> Self;
    fn norm(&self) -> T;
    fn dis(&self, other: &Self) -> T;
}

impl<'a, T: AD> Optima3DVec<'a, T> for [T; 3] {
    #[inline(always)]
    fn x(&self) -> T {
        self[0]
    }

    #[inline(always)]
    fn y(&self) -> T {
        self[1]
    }

    #[inline(always)]
    fn z(&self) -> T {
        self[2]
    }

    #[inline(always)]
    fn from_slice(slice: &[T]) -> Self {
        [slice[0], slice[1], slice[2]]
    }

    #[inline(always)]
    fn as_slice(&self) -> &[T] {
        self
    }

    #[inline]
    fn add(&self, other: &Self) -> Self {
        [ self[0] + other[0], self[1] + other[1], self[2] + other[2] ]
    }

    #[inline]
    fn sub(&self, other: &Self) -> Self {
        [ self[0] - other[0], self[1] - other[1], self[2] - other[2] ]
    }

    #[inline]
    fn scalar_mul(&'a self, scalar: T) -> Self {
        [ scalar * self[0], scalar * self[1], scalar * self[2] ]
    }

    #[inline]
    fn norm(&self) -> T {
        (self[0].powi(2) + self[1].powi(2) + self[2].powi(2)).sqrt()
    }

    #[inline]
    fn dis(&self, other: &Self) -> T {
        self.sub(other).norm()
    }
}

impl<'a, T: AD> Optima3DVec<'a, T> for Vec<T> {
    #[inline(always)]
    fn x(&self) -> T {
        self[0]
    }

    #[inline(always)]
    fn y(&self) -> T {
        self[1]
    }

    #[inline(always)]
    fn z(&self) -> T {
        self[2]
    }

    #[inline(always)]
    fn from_slice(slice: &[T]) -> Self {
        vec![slice[0], slice[1], slice[2]]
    }

    #[inline(always)]
    fn as_slice(&self) -> &[T] {
        self
    }

    #[inline]
    fn add(&self, other: &Self) -> Self {
        vec![ self[0] + other[0], self[1] + other[1], self[2] + other[2] ]
    }

    #[inline]
    fn sub(&self, other: &Self) -> Self {
        vec![ self[0] - other[0], self[1] - other[1], self[2] - other[2] ]
    }

    #[inline]
    fn scalar_mul(&'a self, scalar: T) -> Self {
        vec![ scalar * self[0], scalar * self[1], scalar * self[2] ]
    }

    #[inline]
    fn norm(&self) -> T {
        (self[0].powi(2) + self[1].powi(2) + self[2].powi(2)).sqrt()
    }

    #[inline]
    fn dis(&self, other: &Self) -> T {
        self.sub(other).norm()
    }
}

impl<'a, T: AD + NalgebraMatMulAD<'a, Const<3>, Const<1>, ArrayStorage<T, 3, 1>>> Optima3DVec<'a, T> for Vector3<T> {
    #[inline(always)]
    fn x(&self) -> T {
        self.x
    }

    #[inline(always)]
    fn y(&self) -> T {
        self.y
    }

    #[inline(always)]
    fn z(&self) -> T {
        self.z
    }

    #[inline(always)]
    fn from_slice(slice: &[T]) -> Self {
        Vector3::from_column_slice(slice)
    }

    #[inline(always)]
    fn as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn add(&self, other: &Self) -> Self {
        self + other
    }

    #[inline]
    fn sub(&self, other: &Self) -> Self {
        self - other
    }

    #[inline]
    fn scalar_mul(&'a self, scalar: T) -> Self {
        scalar * self
        // let mut v = [T::zero(); 3];
        // v.iter_mut().zip(self.iter()).for_each(|(x,y)| *x = scalar * *y);
        // Vector3::from_column_slice(&v)
    }

    #[inline]
    fn norm(&self) -> T {
        self.norm()
    }

    #[inline]
    fn dis(&self, other: &Self) -> T {
        self.sub(other).norm()
    }
}