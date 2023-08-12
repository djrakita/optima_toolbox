use std::fmt::{Debug};
use ad_trait::{AD};
use nalgebra::{Point3, Vector3};
use serde::{Deserialize, Serialize};

pub trait O3DVec<T: AD> :
    Debug + Serialize + for<'a> Deserialize<'a>
{
    fn x(&self) -> T;
    fn y(&self) -> T;
    fn z(&self) -> T;
    fn from_slice(slice: &[T]) -> Self;
    fn as_slice(&self) -> &[T];
    fn add(&self, other: &Self) -> Self;
    fn sub(&self, other: &Self) -> Self;
    fn scalar_mul(&self, scalar: T) -> Self;
    fn norm(&self) -> T;
    fn dis(&self, other: &Self) -> T;
}

impl<T: AD> O3DVec<T> for [T; 3] {
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
    fn scalar_mul(&self, scalar: T) -> Self {
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

impl<T: AD> O3DVec<T> for Vec<T> {
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
    fn scalar_mul(&self, scalar: T) -> Self {
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

impl<T: AD> O3DVec<T> for Vector3<T> {
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
    fn scalar_mul(&self, scalar: T) -> Self {
        scalar.mul_by_nalgebra_matrix_ref(self)
        // scalar * self
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
        (self - other).norm()
    }
}

impl<T: AD> O3DVec<T> for Point3<T> {
    fn x(&self) -> T {
        self.x
    }

    fn y(&self) -> T {
        self.y
    }

    fn z(&self) -> T {
        self.z
    }

    fn from_slice(slice: &[T]) -> Self {
        Point3::from_slice(slice)
    }

    fn as_slice(&self) -> &[T] {
        self.coords.as_slice()
    }

    fn add(&self, other: &Self) -> Self {
        (&self.coords + &other.coords).into()
    }

    fn sub(&self, other: &Self) -> Self {
        (&self.coords - &other.coords).into()
    }

    fn scalar_mul(&self, scalar: T) -> Self {
        scalar.mul_by_nalgebra_matrix_ref(&self.coords).into()
    }

    fn norm(&self) -> T {
        self.coords.norm()
    }

    fn dis(&self, other: &Self) -> T {
        (self - other).norm()
    }
}
