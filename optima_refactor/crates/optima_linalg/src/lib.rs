use std::any::Any;
use std::borrow::Cow;
use std::fmt;
use std::fmt::{Debug};
use std::marker::PhantomData;
use ad_trait::AD;
use arrayvec::ArrayVec;
use as_any::{AsAny, Downcast};
use nalgebra::{DMatrix, DVector, SVector};
use ndarray::{Array, ArrayBase, Dim, Ix, Ix1, OwnedRepr};
use serde::de::{SeqAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde::ser::SerializeTuple;
use serde_with::{DeserializeAs, SerializeAs};

#[derive(Clone, Debug, Copy, Eq, PartialEq)]
pub enum OLinalgType {
    Nalgebra, NDarray
}

pub trait OLinalgCategory: Clone + Debug + Serialize + for<'a> Deserialize<'a> + Send + Sync {
    type VecType<T: AD>: OVec<T>;
    type MatType<T: AD>: OMat<T, VecMulInType=Self::VecType<T>, VecMulOutType=Self::VecType<T>, MatMulInType=Self::MatType<T>, MatMulOutType=Self::MatType<T>>;

    fn type_identifier() -> OLinalgType;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OLinalgCategoryNalgebra;
impl OLinalgCategory for OLinalgCategoryNalgebra {
    type VecType<T: AD> = DVector<T>;
    type MatType<T: AD> = DMatrix<T>;

    #[inline]
    fn type_identifier() -> OLinalgType {
        OLinalgType::Nalgebra
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OLinalgCategoryNDarray;
impl OLinalgCategory for OLinalgCategoryNDarray {
    type VecType<T: AD> = ArrayBase<OwnedRepr<T>, Ix1>;
    type MatType<T: AD> = ArrayBase<OwnedRepr<T>, Dim<[Ix; 2]>>;

    #[inline]
    fn type_identifier() -> OLinalgType {
        OLinalgType::NDarray
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug, Copy, Eq, PartialEq)]
pub enum OVecType {
    Slice, Arr, Vec, ArrayVec, DVector, SVector, NDarray,
}

pub trait OVecCategoryTrait : Debug + Clone + Send + Sync {
    type V<T: AD>: OVec<T>;
}

pub trait OVec<T: AD> : Debug + Clone + Send + Sync + AsAny  {
    type Category: OVecCategoryTrait;

    fn ovec_type_identifier() -> OVecType;
    fn ovec_from_slice(slice: &[T]) -> Self;
    fn ovec_as_slice(&self) -> &[T];
    fn ovec_as_mut_slice(&mut self) -> &mut [T];
    /// start idx is inclusive, end idx is exclusive
    /// #[inline(always)]
    fn subslice(&self, start: usize, end: usize) -> &[T] { &self.ovec_as_slice()[start..end]  }
    fn ovec_dot(&self, other: &Self) -> T;
    fn ovec_add(&self, other: &Self) -> Self;
    fn ovec_sub(&self, other: &Self) -> Self;
    fn ovec_scalar_mul(&self, scalar: &T) -> Self;
    fn ovec_scalar_div(&self, scalar: &T) -> Self;
    #[inline(always)]
    fn ovec_p_norm(&self, p: &T) -> T {
        let mut out = T::zero();
        let slice = self.ovec_as_slice();
        if slice.len() == 0 { return out; }
        slice.iter().for_each(|x| { out += x.powf(*p); });
        out.powf(p.recip())
    }
    #[inline(always)]
    fn ovec_linf_norm(&self) -> T {
        let slice = self.ovec_as_slice();
        let max = slice.iter().max_by(|x, y| x.abs().partial_cmp(&y.abs()).expect(&format!("error: {}, {}", x, y)) );
        return match max {
            None => { panic!("cannot take linf norm of empty vec") }
            Some(max) => { max.abs() }
        }
    }
    #[inline(always)]
    fn normalize(&self) -> Self {
        self.ovec_scalar_div(&self.ovec_p_norm(&T::constant(2.0)))
    }
    fn ovec_get_element(&self, i: usize) -> &T;
    fn ovec_get_element_mut(&mut self, i: usize) -> &mut T;
    fn ovec_set_element(&mut self, i: usize, element: T);
    #[inline(always)]
    fn ovec_split_into_sub_vecs(&self, len_of_subvec: usize) -> Vec<&[T]> {
        let l = self.len();
        assert_eq!(l % len_of_subvec, 0);
        let mut out = vec![];

        let mut curr_start_idx = 0;
        let mut curr_end_idx = len_of_subvec;
        while curr_end_idx <= l {
            out.push(self.subslice(curr_start_idx, curr_end_idx));

            curr_start_idx += len_of_subvec;
            curr_end_idx += len_of_subvec;
        }

        out
    }
    fn len(&self) -> usize;
    #[inline(always)]
    fn to_constant_vec(&self) -> Vec<f64> {
        self.ovec_as_slice().iter().map(|x| x.to_constant()).collect()
    }
    #[inline(always)]
    fn ovec_to_constant_ads(&self) -> Self {
        let tmp: Vec<T> = self.ovec_as_slice().iter().map(|x| T::constant(x.to_constant())).collect();
        Self::ovec_from_slice(&tmp)
    }
    #[inline(always)]
    fn ovec_to_other_generic_category<T2: AD, C: OVecCategoryTrait>(&self) -> C::V<T2> {
        let s: Vec<T2> = self.to_constant_vec().iter().map(|x| T2::constant(*x)).collect();
        C::V::ovec_from_slice(&s)
    }
    #[inline(always)]
    fn ovec_to_other_ad_type<T2: AD>(&self) -> <Self::Category as OVecCategoryTrait>::V<T2> {
        self.ovec_to_other_generic_category::<T2, Self::Category>()
    }
    #[inline(always)]
    fn ovec_downcast_or_convert<V: OVec<T>>(&self) -> Cow<V> {
        let downcast = self.downcast_ref();
        match downcast {
            Some(d) => { Cow::Borrowed(d) }
            None => {
                let out = V::ovec_from_slice(&self.ovec_as_slice());
                Cow::Owned(out)
            }
        }
    }
    #[inline]
    fn eq<T2: AD, V: OVec<T2>>(&self, other: V) -> bool {
        if self.len() != other.len() { return false; }

        for x in self.ovec_as_slice() {
            for y in other.ovec_as_slice() {
                let xc = x.to_constant();
                let yc = y.to_constant();
                if xc != yc { return false; }
            }
        }

        return true;
    }
}

/*
impl<'a, T: AD> OVec<T> for &'a [T] {
    type Category = OVecCategorySliceRef<'a>;

    #[inline]
    fn type_identifier() -> OVecType {
        OVecType::Slice
    }

    #[inline]
    fn from_slice_ovec(_slice: &[T]) -> Self {
        panic!("cannot make slice from slice.")
    }

    #[inline]
    fn as_slice_ovec(&self) -> &[T] {
        self
    }

    fn as_mut_slice_ovec(&mut self) -> &mut [T] {
        panic!("cannot borrow a slice as slice mut")
    }

    #[inline]
    fn dot(&self, other: &Self) -> T {
        let mut out = T::zero();
        self.iter().zip(other.iter()).for_each(|(x, y)| out +=  *x * *y);
        out
    }

    #[inline]
    fn add(&self, _other: &Self) -> Self {
        panic!("cannot do general math on slice")
    }

    #[inline]
    fn sub(&self, _other: &Self) -> Self {
        panic!("cannot do general math on slice")
    }

    #[inline]
    fn scalar_mul(&self, _scalar: &T) -> Self {
        panic!("cannot do general math on slice")
    }

    #[inline]
    fn scalar_div(&self, _scalar: &T) -> Self { panic!("cannot do general math on slice") }

    #[inline]
    fn get_element(&self, i: usize) -> &T {
        &self[i]
    }

    #[inline]
    fn get_element_mut(&mut self, _i: usize) -> &mut T {
        panic!("cannot get mutable element on immutable slice")
    }

    #[inline]
    fn set_element(&mut self, _i: usize, _element: T) {
        panic!("cannot set mutable element on immutable slice")
    }

    #[inline]
    fn len(&self) -> usize {
        <[T]>::len(self)
    }
}
*/

/*
#[derive(Debug, Clone)]
pub struct OVecCategorySliceRef<'a>(&'a PhantomData<()>);
impl<'a> OVecCategoryTrait for OVecCategorySliceRef<'a> {
    type V<T: AD> = &'a [T];
}
*/

impl<T: AD, const N: usize> OVec<T> for [T; N] {
    type Category = OVecCategoryArr<N>;

    #[inline]
    fn ovec_type_identifier() -> OVecType {
        OVecType::Arr
    }

    #[inline]
    fn ovec_from_slice(slice: &[T]) -> Self {
        assert_eq!(slice.len(), N);
        let mut out = [T::zero(); N];
        out.iter_mut().zip(slice.iter()).for_each(|(x,y)| *x = *y );
        out
    }

    #[inline]
    fn ovec_as_slice(&self) -> &[T] {
        self
    }

    #[inline]
    fn ovec_as_mut_slice(&mut self) -> &mut [T] {
        self.as_mut_slice()
    }

    #[inline]
    fn ovec_dot(&self, other: &Self) -> T {
        let mut out = T::zero();
        self.iter().zip(other.iter()).for_each(|(x, y)| out +=  *x * *y);
        out
    }

    #[inline]
    fn ovec_add(&self, other: &Self) -> Self {
        let mut out = [T::zero(); N];
        out.iter_mut().enumerate().for_each(|(i, x)| *x = self[i]+other[i]);
        out
    }

    #[inline]
    fn ovec_sub(&self, other: &Self) -> Self {
        let mut out = [T::zero(); N];
        out.iter_mut().enumerate().for_each(|(i, x)| *x = self[i]-other[i]);
        out
    }

    #[inline]
    fn ovec_scalar_mul(&self, scalar: &T) -> Self {
        let mut out = self.clone();
        out.iter_mut().for_each(|x| *x *= *scalar);
        out
    }

    #[inline]
    fn ovec_scalar_div(&self, scalar: &T) -> Self {
        let mut out = self.clone();
        out.iter_mut().for_each(|x| *x /= *scalar);
        out
    }

    #[inline]
    fn ovec_get_element(&self, i: usize) -> &T {
        &self[i]
    }

    #[inline]
    fn ovec_get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    #[inline]
    fn ovec_set_element(&mut self, i: usize, element: T) {
        self[i] = element
    }

    #[inline]
    fn len(&self) -> usize {
        N
    }
}

#[derive(Debug, Clone)]
pub struct OVecCategoryArr<const N: usize>;
impl<const N: usize> OVecCategoryTrait for OVecCategoryArr<N> {
    type V<T: AD> = [T; N];
}

impl<T: AD> OVec<T> for Vec<T> {
    type Category = OVecCategoryVec;

    #[inline]
    fn ovec_type_identifier() -> OVecType {
        OVecType::Vec
    }

    #[inline]
    fn ovec_from_slice(slice: &[T]) -> Self {
        slice.into()
    }

    #[inline]
    fn ovec_as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn ovec_as_mut_slice(&mut self) -> &mut [T] {
        self.as_mut_slice()
    }

    #[inline]
    fn ovec_dot(&self, other: &Self) -> T {
        let mut out = T::zero();
        self.iter().zip(other.iter()).for_each(|(x, y)| out +=  *x * *y);
        out
    }

    #[inline]
    fn ovec_add(&self, other: &Self) -> Self {
        assert_eq!(self.len(), other.len());
        let mut out = vec![T::zero(); self.len()];
        out.iter_mut().enumerate().for_each(|(i, x)| *x = self[i]+other[i]);
        out
    }

    #[inline]
    fn ovec_sub(&self, other: &Self) -> Self {
        assert_eq!(self.len(), other.len());
        let mut out = vec![T::zero(); self.len()];
        out.iter_mut().enumerate().for_each(|(i, x)| *x = self[i]-other[i]);
        out
    }

    #[inline]
    fn ovec_scalar_mul(&self, scalar: &T) -> Self {
        let mut out = self.clone();
        out.iter_mut().for_each(|x| *x *= *scalar);
        out
    }

    #[inline]
    fn ovec_scalar_div(&self, scalar: &T) -> Self {
        let mut out = self.clone();
        out.iter_mut().for_each(|x| *x /= *scalar);
        out
    }

    #[inline]
    fn ovec_get_element(&self, i: usize) -> &T {
        &self[i]
    }

    #[inline]
    fn ovec_get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    #[inline]
    fn ovec_set_element(&mut self, i: usize, element: T) {
        self[i] = element
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }
}
#[derive(Debug, Clone)]
pub struct OVecCategoryVec;
impl OVecCategoryTrait for OVecCategoryVec {
    type V<T: AD> = Vec<T>;
}

impl<T: AD, const M: usize> OVec<T> for ArrayVec<T, M> {
    type Category = OVecCategoryArrayVec<M>;

    #[inline]
    fn ovec_type_identifier() -> OVecType {
        OVecType::ArrayVec
    }

    #[inline]
    fn ovec_from_slice(slice: &[T]) -> Self {
        ArrayVec::try_from(slice).expect("error")
    }

    #[inline]
    fn ovec_as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn ovec_as_mut_slice(&mut self) -> &mut [T] {
        self.as_mut_slice()
    }

    #[inline]
    fn ovec_dot(&self, other: &Self) -> T {
        let mut out = T::zero();
        self.iter().zip(other.iter()).for_each(|(x, y)| out +=  *x * *y);
        out
    }

    #[inline]
    fn ovec_add(&self, other: &Self) -> Self {
        let mut out = ArrayVec::new();
        self.iter().zip(other.iter()).for_each(|(x, y)| out.push(*x + *y));
        out
    }

    #[inline]
    fn ovec_sub(&self, other: &Self) -> Self {
        let mut out = ArrayVec::new();
        self.iter().zip(other.iter()).for_each(|(x, y)| out.push(*x - *y));
        out
    }

    #[inline]
    fn ovec_scalar_mul(&self, scalar: &T) -> Self {
        let mut out = self.clone();
        out.iter_mut().for_each(|x| *x *= *scalar);
        out
    }

    #[inline]
    fn ovec_scalar_div(&self, scalar: &T) -> Self {
        let mut out = self.clone();
        out.iter_mut().for_each(|x| *x /= *scalar);
        out
    }

    #[inline]
    fn ovec_get_element(&self, i: usize) -> &T {
        &self[i]
    }

    #[inline]
    fn ovec_get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    #[inline]
    fn ovec_set_element(&mut self, i: usize, element: T) {
        self[i] = element
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }
}
#[derive(Debug, Clone)]
pub struct OVecCategoryArrayVec<const M: usize>;
impl<const M: usize> OVecCategoryTrait for OVecCategoryArrayVec<M> {
    type V<T: AD> = ArrayVec<T, M>;
}

impl<T: AD> OVec<T> for DVector<T> {
    type Category = OVecCategoryDVector;

    #[inline]
    fn ovec_type_identifier() -> OVecType {
        OVecType::DVector
    }

    #[inline]
    fn ovec_from_slice(slice: &[T]) -> Self {
        DVector::from_column_slice(slice)
    }

    #[inline]
    fn ovec_as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn ovec_as_mut_slice(&mut self) -> &mut [T] {
        self.as_mut_slice()
    }

    #[inline]
    fn ovec_dot(&self, other: &Self) -> T {
        self.dot(other)
    }

    #[inline]
    fn ovec_add(&self, other: &Self) -> Self {
        self + other
    }

    #[inline]
    fn ovec_sub(&self, other: &Self) -> Self {
        self - other
    }

    #[inline]
    fn ovec_scalar_mul(&self, scalar: &T) -> Self {
        scalar.mul_by_nalgebra_matrix_ref(self)
    }

    #[inline]
    fn ovec_scalar_div(&self, scalar: &T) -> Self {
        (T::one() / *scalar).mul_by_nalgebra_matrix_ref(self)
    }

    #[inline]
    fn ovec_get_element(&self, i: usize) -> &T {
        &self[i]
    }

    #[inline]
    fn ovec_get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    #[inline]
    fn ovec_set_element(&mut self, i: usize, element: T) {
        self[i] = element
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }
}
#[derive(Debug, Clone)]
pub struct OVecCategoryDVector;
impl OVecCategoryTrait for OVecCategoryDVector {
    type V<T: AD> = DVector<T>;
}

impl<T: AD, const N: usize> OVec<T> for SVector<T, N> {
    type Category = OVecCategorySVector<N>;

    #[inline]
    fn ovec_type_identifier() -> OVecType {
        OVecType::SVector
    }

    #[inline]
    fn ovec_from_slice(slice: &[T]) -> Self {
        SVector::from_column_slice(slice)
    }

    #[inline]
    fn ovec_as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn ovec_as_mut_slice(&mut self) -> &mut [T] {
        self.as_mut_slice()
    }

    #[inline]
    fn ovec_dot(&self, other: &Self) -> T {
        self.dot(other)
    }

    #[inline]
    fn ovec_add(&self, other: &Self) -> Self {
        self + other
    }

    #[inline]
    fn ovec_sub(&self, other: &Self) -> Self {
        self - other
    }

    #[inline]
    fn ovec_scalar_mul(&self, scalar: &T) -> Self {
        scalar.mul_by_nalgebra_matrix_ref(self)
    }

    fn ovec_scalar_div(&self, scalar: &T) -> Self {
        (T::one() / *scalar).mul_by_nalgebra_matrix_ref(self)
    }

    #[inline]
    fn ovec_get_element(&self, i: usize) -> &T {
        &self[i]
    }

    #[inline]
    fn ovec_get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    #[inline]
    fn ovec_set_element(&mut self, i: usize, element: T) {
        self[i] = element;
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }
}
#[derive(Debug, Clone)]
pub struct OVecCategorySVector<const N: usize>;
impl<const N: usize> OVecCategoryTrait for OVecCategorySVector<N> {
    type V<T: AD> = SVector<T, N>;
}

impl<T: AD> OVec<T> for ArrayBase<OwnedRepr<T>, Ix1> {
    type Category = OVecCategoryNDarray;

    #[inline]
    fn ovec_type_identifier() -> OVecType {
        OVecType::NDarray
    }

    #[inline]
    fn ovec_from_slice(slice: &[T]) -> Self {
        ArrayBase::from_vec(slice.to_vec())
    }

    #[inline]
    fn ovec_as_slice(&self) -> &[T] {
        self.as_slice().unwrap()
    }

    #[inline]
    fn ovec_as_mut_slice(&mut self) -> &mut [T] {
        self.as_slice_mut().unwrap()
    }

    #[inline]
    fn ovec_dot(&self, other: &Self) -> T {
        self.dot(other)
    }

    #[inline]
    fn ovec_add(&self, other: &Self) -> Self {
        self + other
    }

    #[inline]
    fn ovec_sub(&self, other: &Self) -> Self {
        self - other
    }

    #[inline]
    fn ovec_scalar_mul(&self, scalar: &T) -> Self {
        scalar.mul_by_ndarray_matrix_ref(self)
    }

    fn ovec_scalar_div(&self, scalar: &T) -> Self {
        (T::one() / *scalar).mul_by_ndarray_matrix_ref(self)
    }

    #[inline]
    fn ovec_get_element(&self, i: usize) -> &T {
        &self[i]
    }

    #[inline]
    fn ovec_get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    #[inline]
    fn ovec_set_element(&mut self, i: usize, element: T) {
        self[i] = element;
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }
}
#[derive(Debug, Clone)]
pub struct OVecCategoryNDarray;
impl OVecCategoryTrait for OVecCategoryNDarray {
    type V<T: AD> = ArrayBase<OwnedRepr<T>, Ix1>;
}

// Custom serialization for types implementing the `OVec` trait
pub fn ovec_custom_serialize<S, T: AD, V: OVec<T>>(value: &V, serializer: S) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer
{
    let slice = value.ovec_as_slice();
    let slice_as_f64: Vec<f64> = slice.iter().map(|x| x.to_constant()).collect();
    let mut tuple = serializer.serialize_tuple(slice_as_f64.len())?;
    for element in &slice_as_f64 {
        tuple.serialize_element(element)?;
    }
    tuple.end()
}

// Custom deserialization for types implementing the `OVec` trait
struct OVecVisitor<T2: AD, V2: OVec<T2>> {
    _phantom_data: PhantomData<(T2, V2)>
}

impl<'de, T2: AD, V2: OVec<T2>> Visitor<'de> for OVecVisitor<T2, V2> {
    type Value = V2;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a sequence of values")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let mut values = Vec::new();
        while let Some(val) = seq.next_element()? {
            values.push(T2::constant(val));
        }
        Ok(V2::ovec_from_slice(&values))
    }
}

pub fn ovec_custom_deserialize<'de, D, T: AD, V: OVec<T>>(deserializer: D) -> Result<V, D::Error>
where
    D: Deserializer<'de>,
{
    deserializer.deserialize_seq(OVecVisitor::<T, V> { _phantom_data: PhantomData::default() })
}

pub struct SerdeOVec<T: AD, V: OVec<T>>(pub V, PhantomData<T>);

impl<T: AD, V: OVec<T>> SerializeAs<V> for SerdeOVec<T, V> {
    fn serialize_as<S>(source: &V, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
        ovec_custom_serialize(source, serializer)
    }
}
impl<'de, T: AD, V: OVec<T>> DeserializeAs<'de, V> for SerdeOVec<T, V> {
    fn deserialize_as<D>(deserializer: D) -> Result<V, D::Error> where D: Deserializer<'de> {
        ovec_custom_deserialize(deserializer)
    }
}

/*
impl<T: AD, R: Dim + Clone> OVec<T> for OMatrix<T, R, U1> where DefaultAllocator: Allocator<T, R, U1> {
    fn from_slice(slice: &[T]) -> Self {
        // OMatrix::<T, R, U1>::from_column_slice(slice)
        todo!()
    }

    fn as_slice(&self) -> &[T] {
        self.as_slice()
    }

    fn dot(&self, other: &Self) -> T {
        self.dot(other)
    }

    fn add(&self, other: &Self) -> Self {
        self + other
    }

    fn sub(&self, other: &Self) -> Self {
        self - other
    }

    fn scalar_mul(&self, scalar: &T) -> Self {
        scalar.mul_by_nalgebra_matrix_ref(self)
    }

    fn get_element(&self, i: usize) -> &T {
        &self[i]
    }

    fn get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    fn set_element(&mut self, i: usize, element: T) {
        self[i] = element
    }
}
*/
/*
impl<T: AD, R: Dim + Clone, S: RawStorageMut<T, R, U1> + Storage<T, R> + Clone> OVec<T> for Matrix<T, R, U1, S> {
    fn as_slice(&self) -> &[T] {
        return unsafe{ self.data.as_slice_unchecked() }
    }

    fn dot(&self, other: &Self) -> T {
        self.dot(other)
    }

    fn add(&self, other: &Self) -> Self {
        let mut out = self.clone();
        self.add_to(other, &mut out);
        out
    }

    fn sub(&self, other: &Self) -> Self {
        let mut out = self.clone();
        self.sub_to(other, &mut out);
        out
    }

    fn scalar_mul(&self, scalar: &T) -> Self {
        scalar.mul_by_nalgebra_matrix_ref(self)
    }

    fn get_element(&self, i: usize) -> &T {
        &self[i]
    }

    fn get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    fn set_element(&mut self, i: usize, element: T) {
        self[i] = element;
    }
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug, Copy, Eq, PartialEq)]
pub enum OMatType {
    DMatrix, NDMatrix
}

pub trait OMatCategoryTrait : Debug + Clone + Send + Sync {
    type M<T: AD>: OMat<T>;
}

pub trait OMat<T: AD> : Debug + Clone + Send + Sync  {
    type Category : OMatCategoryTrait;

    type VecMulInType;
    type VecMulOutType;
    type MatMulInType;
    type MatMulOutType;

    fn as_any(&self) -> &dyn Any;

    fn type_identifier(&self) -> OMatType;
    fn from_column_major_slice(slice: &[T], nrows: usize, ncols: usize) -> Self;
    fn as_column_major_slice(&self) -> &[T];
    fn vec_mul(&self, other: &Self::VecMulInType) -> Self::VecMulOutType;
    fn mat_mul(&self, other: &Self::MatMulInType) -> Self::MatMulOutType;
    fn mat_elementwise_add(&self, other: &Self) -> Self;
    fn mat_elementwise_sub(&self, other: &Self) -> Self;
    fn scalar_mul(&self, scalar: T) -> Self;
    /// (rows, cols)
    fn dims(&self) -> (usize, usize);
    fn omat_to_other_generic_category<T2: AD, C: OMatCategoryTrait>(&self) -> C::M<T2> {
        let column_major_slice = self.as_column_major_slice().to_vec().ovec_to_other_generic_category::<T2, OVecCategoryVec>();
        let dims = self.dims();
        C::M::from_column_major_slice(&column_major_slice, dims.0, dims.1)
    }
    fn omat_to_other_ad_type<T2: AD>(&self) -> <Self::Category as OMatCategoryTrait>::M<T2> {
        self.omat_to_other_generic_category::<T2, Self::Category>()
    }
    /*
    #[inline(always)]
    fn downcast_or_convert<M: OMat<T>>(&self) -> Cow<M> {
        let downcast = self.downcast_ref();
        match downcast {
            Some(d) => { Cow::Borrowed(d) }
            None => {
                let dims = self.dims();
                let out = M::from_column_major_slice(self.as_column_major_slice(), dims.0, dims.1);
                Cow::Owned(out)
            }
        }
    }

     */
}

impl<T: AD> OMat<T> for DMatrix<T> {
    type Category = OMatCategoryDMatrix;
    type VecMulInType = DVector<T>;
    type VecMulOutType = DVector<T>;
    type MatMulInType = DMatrix<T>;
    type MatMulOutType = DMatrix<T>;

    #[inline(always)]
    fn as_any(&self) -> &dyn Any {
        self
    }

    #[inline]
    fn type_identifier(&self) -> OMatType {
        OMatType::DMatrix
    }

    #[inline]
    fn from_column_major_slice(slice: &[T], nrows: usize, ncols: usize) -> Self {
        DMatrix::from_column_slice(nrows, ncols, slice)
    }

    #[inline]
    fn as_column_major_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn vec_mul(&self, other: &Self::VecMulInType) -> Self::VecMulOutType {
        self * other
    }

    #[inline]
    fn mat_mul(&self, other: &Self::MatMulInType) -> Self::MatMulOutType {
        self * other
    }

    #[inline]
    fn mat_elementwise_add(&self, other: &Self) -> Self {
        self + other
    }

    #[inline]
    fn mat_elementwise_sub(&self, other: &Self) -> Self {
        self - other
    }

    #[inline]
    fn scalar_mul(&self, scalar: T) -> Self {
        scalar.mul_by_nalgebra_matrix_ref(self)
    }

    #[inline]
    fn dims(&self) -> (usize, usize) {
        self.shape()
    }
}
#[derive(Debug, Clone)]
pub struct OMatCategoryDMatrix;
impl OMatCategoryTrait for OMatCategoryDMatrix {
    type M<T: AD> = DMatrix<T>;
}

impl<T: AD> OMat<T> for ArrayBase<OwnedRepr<T>, Dim<[Ix; 2]>> {
    type Category = OMatCategoryNDarray;
    type VecMulInType = ArrayBase<OwnedRepr<T>, Ix1>;
    type VecMulOutType = ArrayBase<OwnedRepr<T>, Ix1>;
    type MatMulInType = ArrayBase<OwnedRepr<T>, Dim<[Ix; 2]>>;
    type MatMulOutType = ArrayBase<OwnedRepr<T>, Dim<[Ix; 2]>>;

    #[inline(always)]
    fn as_any(&self) -> &dyn Any {
        self
    }

    #[inline]
    fn type_identifier(&self) -> OMatType {
        OMatType::NDMatrix
    }

    #[inline]
    fn from_column_major_slice(slice: &[T], nrows: usize, ncols: usize) -> Self {
        Array::from_shape_vec((nrows, ncols), slice.to_vec()).unwrap()
    }

    #[inline]
    fn as_column_major_slice(&self) -> &[T] {
        self.as_slice().unwrap()
    }

    #[inline]
    fn vec_mul(&self, other: &Self::VecMulInType) -> Self::VecMulOutType {
        self.dot(other)
    }

    #[inline]
    fn mat_mul(&self, other: &Self::MatMulInType) -> Self::MatMulOutType {
        self.dot(other)
    }

    #[inline]
    fn mat_elementwise_add(&self, other: &Self) -> Self {
        self + other
    }

    #[inline]
    fn mat_elementwise_sub(&self, other: &Self) -> Self {
        self - other
    }

    #[inline]
    fn scalar_mul(&self, scalar: T) -> Self {
        scalar.mul_by_ndarray_matrix_ref(self)
    }

    #[inline]
    fn dims(&self) -> (usize, usize) {
        (self.nrows(), self.ncols())
    }
}
#[derive(Debug, Clone)]
pub struct OMatCategoryNDarray;
impl OMatCategoryTrait for OMatCategoryNDarray {
    type M<T: AD> = ArrayBase<OwnedRepr<T>, Dim<[Ix; 2]>>;
}

// Custom serialization for types implementing the `OMat` trait
pub fn omat_custom_serialize<S, T: AD, M: OMat<T>>(matrix: &M, serializer: S) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer,
{
    // Convert the matrix data to a column-major slice of constants
    let data_as_f64: Vec<f64> = matrix.as_column_major_slice()
                                      .iter()
                                      .map(|&val| val.to_constant())
                                      .collect();

    // Serialize as a tuple: (data, (num_rows, num_columns))
    let dims = matrix.dims();
    (data_as_f64, dims).serialize(serializer)
}

// Custom deserialization for types implementing the `OMat` trait
struct OMatVisitor<T2: AD, M2: OMat<T2>> {
    _phantom_data: PhantomData<(T2, M2)>
}

impl<'de, T2: AD, M2: OMat<T2>> Visitor<'de> for OMatVisitor<T2, M2> {
    type Value = M2;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a tuple containing data and matrix dimensions")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let data: Vec<f64> = seq.next_element()?.expect("Expected data vector");
        let (rows, cols): (usize, usize) = seq.next_element()?.expect("Expected matrix dimensions");
        let data_as_ad: Vec<T2> = data.into_iter().map(T2::constant).collect();
        Ok(M2::from_column_major_slice(&data_as_ad, rows, cols))
    }
}

pub fn omat_custom_deserialize<'de, D, T: AD, M: OMat<T>>(deserializer: D) -> Result<M, D::Error>
where
    D: Deserializer<'de>,
{
    deserializer.deserialize_tuple(2, OMatVisitor::<T, M> { _phantom_data: PhantomData::default() })
}

pub struct SerdeOMat<T: AD, M: OMat<T>>(pub M, PhantomData<T>);

impl<T: AD, M: OMat<T>> SerializeAs<M> for SerdeOMat<T, M> {
    fn serialize_as<S>(source: &M, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
        omat_custom_serialize(source, serializer)
    }
}
impl<'de, T: AD, M: OMat<T>> DeserializeAs<'de, M> for SerdeOMat<T, M> {
    fn deserialize_as<D>(deserializer: D) -> Result<M, D::Error> where D: Deserializer<'de> {
        omat_custom_deserialize(deserializer)
    }
}

/*
impl<T: AD, R: Dim + Clone + DimName, C: Dim + Clone + DimName> OMat<T> for OMatrix<T, R, C>
    where DefaultAllocator: Allocator<T, R, C>, DefaultAllocator: Allocator<T, C>, DefaultAllocator: Allocator<T, C>, DefaultAllocator: Allocator<T, R>
{
    type VecMulInType = OMatrix<T, C, U1>;
    type VecMulOutType = OMatrix<T, R, U1>;
    type MatMulInType = OMatrix<T, C, Dyn>;
    type MatMulOutType = OMatrix<T, R, Dyn>;

    fn vec_mul(&self, other: &Self::VecMulInType) -> Self::VecMulOutType {
        self * other
    }

    fn mat_mul(&self, other: &Self::MatMulInType) -> Self::MatMulOutType {
        self * other
    }
}
*/
/*
impl<T: AD> OMat<T> for DMatrix<T> {

}
*/
/*
impl<T: AD, R: Dim + Clone, C: Dim + Clone, S: Clone> OMat<T> for Matrix<T, R, C, S> {

}
*/
