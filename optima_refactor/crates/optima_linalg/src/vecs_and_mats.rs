use std::fmt;
use std::fmt::{Debug};
use std::marker::PhantomData;
use ad_trait::AD;
use arrayvec::ArrayVec;
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

pub trait OLinalgTrait : Clone + Debug + Serialize + for<'a> Deserialize<'a> {
    type VecType<T: AD>: OVec<T>;
    type MatType<T: AD>: OMat<T, VecMulInType=Self::VecType<T>, VecMulOutType=Self::VecType<T>, MatMulInType=Self::MatType<T>, MatMulOutType=Self::MatType<T>>;

    fn type_identifier() -> OLinalgType;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NalgebraLinalg;
impl OLinalgTrait for NalgebraLinalg {
    type VecType<T: AD> = DVector<T>;
    type MatType<T: AD> = DMatrix<T>;

    #[inline]
    fn type_identifier() -> OLinalgType {
        OLinalgType::Nalgebra
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NdarrayLinalg;
impl OLinalgTrait for NdarrayLinalg {
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
    Arr, ArrayVec, DVector, SVector, NDarray,
}

pub trait OVec<T: AD> : Debug + Clone {
    fn type_identifier() -> OVecType;
    fn from_slice(slice: &[T]) -> Self;
    fn as_slice(&self) -> &[T];
    /// start idx is inclusive, end idx is exclusive
    fn subslice(&self, start: usize, end: usize) -> &[T] { &self.as_slice()[start..end]  }
    fn dot(&self, other: &Self) -> T;
    fn add(&self, other: &Self) -> Self;
    fn sub(&self, other: &Self) -> Self;
    fn scalar_mul(&self, scalar: &T) -> Self;
    fn get_element(&self, i: usize) -> &T;
    fn get_element_mut(&mut self, i: usize) -> &mut T;
    fn set_element(&mut self, i: usize, element: T);
    fn len(&self) -> usize;
}

impl<T: AD, const N: usize> OVec<T> for [T; N] {
    #[inline]
    fn type_identifier() -> OVecType {
        OVecType::Arr
    }

    #[inline]
    fn from_slice(slice: &[T]) -> Self {
        assert_eq!(slice.len(), N);
        let mut out = [T::zero(); N];
        out.iter_mut().zip(slice.iter()).for_each(|(x,y)| *x = *y );
        out
    }

    #[inline]
    fn as_slice(&self) -> &[T] {
        self
    }

    #[inline]
    fn dot(&self, other: &Self) -> T {
        let mut out = T::zero();
        self.iter().zip(other.iter()).for_each(|(x, y)| out +=  *x * *y);
        out
    }

    #[inline]
    fn add(&self, other: &Self) -> Self {
        let mut out = [T::zero(); N];
        out.iter_mut().enumerate().for_each(|(i, x)| *x = self[i]+other[i]);
        out
    }

    #[inline]
    fn sub(&self, other: &Self) -> Self {
        let mut out = [T::zero(); N];
        out.iter_mut().enumerate().for_each(|(i, x)| *x = self[i]-other[i]);
        out
    }

    #[inline]
    fn scalar_mul(&self, scalar: &T) -> Self {
        let mut out = self.clone();
        out.iter_mut().for_each(|x| *x *= *scalar);
        out
    }

    #[inline]
    fn get_element(&self, i: usize) -> &T {
        &self[i]
    }

    #[inline]
    fn get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    #[inline]
    fn set_element(&mut self, i: usize, element: T) {
        self[i] = element
    }

    #[inline]
    fn len(&self) -> usize {
        N
    }
}

impl<T: AD, const M: usize> OVec<T> for ArrayVec<T, M> {
    #[inline]
    fn type_identifier() -> OVecType {
        OVecType::ArrayVec
    }

    #[inline]
    fn from_slice(slice: &[T]) -> Self {
        ArrayVec::try_from(slice).expect("error")
    }

    #[inline]
    fn as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn dot(&self, other: &Self) -> T {
        let mut out = T::zero();
        self.iter().zip(other.iter()).for_each(|(x, y)| out +=  *x * *y);
        out
    }

    #[inline]
    fn add(&self, other: &Self) -> Self {
        let mut out = ArrayVec::new();
        self.iter().zip(other.iter()).for_each(|(x, y)| out.push(*x + *y));
        out
    }

    #[inline]
    fn sub(&self, other: &Self) -> Self {
        let mut out = ArrayVec::new();
        self.iter().zip(other.iter()).for_each(|(x, y)| out.push(*x - *y));
        out
    }

    #[inline]
    fn scalar_mul(&self, scalar: &T) -> Self {
        let mut out = self.clone();
        out.iter_mut().for_each(|x| *x *= *scalar);
        out
    }

    #[inline]
    fn get_element(&self, i: usize) -> &T {
        &self[i]
    }

    #[inline]
    fn get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    #[inline]
    fn set_element(&mut self, i: usize, element: T) {
        self[i] = element
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }
}

impl<T: AD> OVec<T> for DVector<T> {
    #[inline]
    fn type_identifier() -> OVecType {
        OVecType::DVector
    }

    #[inline]
    fn from_slice(slice: &[T]) -> Self {
        DVector::from_column_slice(slice)
    }

    #[inline]
    fn as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn dot(&self, other: &Self) -> T {
        self.dot(other)
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
    fn scalar_mul(&self, scalar: &T) -> Self {
        scalar.mul_by_nalgebra_matrix_ref(self)
    }

    #[inline]
    fn get_element(&self, i: usize) -> &T {
        &self[i]
    }

    #[inline]
    fn get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    #[inline]
    fn set_element(&mut self, i: usize, element: T) {
        self[i] = element
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }
}

impl<T: AD, const N: usize> OVec<T> for SVector<T, N> {
    #[inline]
    fn type_identifier() -> OVecType {
        OVecType::SVector
    }

    #[inline]
    fn from_slice(slice: &[T]) -> Self {
        SVector::from_column_slice(slice)
    }

    #[inline]
    fn as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn dot(&self, other: &Self) -> T {
        self.dot(other)
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
    fn scalar_mul(&self, scalar: &T) -> Self {
        scalar.mul_by_nalgebra_matrix_ref(self)
    }

    #[inline]
    fn get_element(&self, i: usize) -> &T {
        &self[i]
    }

    #[inline]
    fn get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    #[inline]
    fn set_element(&mut self, i: usize, element: T) {
        self[i] = element;
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }
}

impl<T: AD> OVec<T> for ArrayBase<OwnedRepr<T>, Ix1> {
    #[inline]
    fn type_identifier() -> OVecType {
        OVecType::NDarray
    }

    #[inline]
    fn from_slice(slice: &[T]) -> Self {
        ArrayBase::from_vec(slice.to_vec())
    }

    #[inline]
    fn as_slice(&self) -> &[T] {
        self.as_slice().unwrap()
    }

    #[inline]
    fn dot(&self, other: &Self) -> T {
        self.dot(other)
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
    fn scalar_mul(&self, scalar: &T) -> Self {
        scalar.mul_by_ndarray_matrix_ref(self)
    }

    #[inline]
    fn get_element(&self, i: usize) -> &T {
        &self[i]
    }

    #[inline]
    fn get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    #[inline]
    fn set_element(&mut self, i: usize, element: T) {
        self[i] = element;
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }
}

// Custom serialization for types implementing the `OVec` trait
pub fn ovec_custom_serialize<S, T: AD, V: OVec<T>>(value: &V, serializer: S) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer
{
    let slice = value.as_slice();
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
        Ok(V2::from_slice(&values))
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

pub trait OMat<T: AD> : Debug + Clone {
    type VecMulInType;
    type VecMulOutType;
    type MatMulInType;
    type MatMulOutType;

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
}

impl<T: AD> OMat<T> for DMatrix<T> {
    type VecMulInType = DVector<T>;
    type VecMulOutType = DVector<T>;
    type MatMulInType = DMatrix<T>;
    type MatMulOutType = DMatrix<T>;

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

impl<T: AD> OMat<T> for ArrayBase<OwnedRepr<T>, Dim<[Ix; 2]>> {
    type VecMulInType = ArrayBase<OwnedRepr<T>, Ix1>;
    type VecMulOutType = ArrayBase<OwnedRepr<T>, Ix1>;
    type MatMulInType = ArrayBase<OwnedRepr<T>, Dim<[Ix; 2]>>;
    type MatMulOutType = ArrayBase<OwnedRepr<T>, Dim<[Ix; 2]>>;

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