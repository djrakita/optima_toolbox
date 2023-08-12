use std::fmt::Debug;
use std::str::{Chars, Split};
use std::str::pattern::Pattern;
use arrayvec::{ArrayString, ArrayVec};
use serde::{Serialize, Deserialize};
use serde::de::DeserializeOwned;

pub trait ArrStorage: Clone + Debug + Serialize + DeserializeOwned {
    type ArrType<U: Serialize + DeserializeOwned + Clone + Debug, const N: usize> : MutArr<U, N>;
    type StrType<const N: usize> : Str<N>;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VecStor;
impl ArrStorage for VecStor {
    type ArrType<U: Serialize + DeserializeOwned + Clone + Debug, const N: usize> = Vec<U>;
    type StrType<const N: usize> = String;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ArrayVecStor;
impl ArrStorage for ArrayVecStor {
    type ArrType<U: Serialize + DeserializeOwned + Clone + Debug, const N: usize> = ArrayVec<U, N>;
    type StrType<const N: usize> = ArrayString<N>;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ImmutArr<T: Serialize + DeserializeOwned + Clone + Debug, const M: usize> :
    Serialize + DeserializeOwned + Default + Clone + Debug + FromIterator<T> + IntoIterator
{
    fn from_slice(slice: &[T]) -> Self;
    fn as_slice(&self) -> &[T];
    fn len(&self) -> usize;
    fn get_element(&self, i: usize) -> &T;
}

pub trait MutArr<T: Serialize + DeserializeOwned + Clone + Debug, const M: usize> :
    ImmutArr<T, M>
{
    fn get_mut_element(&mut self, i: usize) -> &mut T;
    fn push(&mut self, item: T);
}

impl<T: Serialize + DeserializeOwned + Clone + Debug, const M: usize> ImmutArr<T, M> for Vec<T> {
    #[inline]
    fn from_slice(slice: &[T]) -> Self {
        Vec::from(slice)
    }

    #[inline]
    fn as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }

    #[inline]
    fn get_element(&self, i: usize) -> &T {
        &self[i]
    }
}
impl<T: Serialize + DeserializeOwned + Clone + Debug, const M: usize> MutArr<T, M> for Vec<T> {
    fn get_mut_element(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    fn push(&mut self, item: T) {
        self.push(item);
    }
}

impl<T: Serialize + DeserializeOwned + Clone + Debug, const M: usize> ImmutArr<T, M> for ArrayVec<T, M> {
    #[inline]
    fn from_slice(slice: &[T]) -> Self {
        ArrayVec::try_from(slice).expect("error")
    }

    #[inline]
    fn as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }

    #[inline]
    fn get_element(&self, i: usize) -> &T {
        &self[i]
    }
}
impl<T: Serialize + DeserializeOwned + Clone + Debug, const M: usize> MutArr<T, M> for ArrayVec<T, M> {
    fn get_mut_element(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    fn push(&mut self, item: T) {
        self.push(item);
    }
}

pub trait Str<const M: usize> :
    Serialize + DeserializeOwned + Clone + Debug + Default {
    fn from_str(s: &str) -> Self;
    fn as_str(&self) -> &str;
    fn push(&mut self, c: char);
    fn push_str(&mut self, s: &str);
    fn characters(&self) -> Chars<'_>;
    fn split_at_pattern<'a, P: Pattern<'a>>(&'a self, pat: P) -> Split<'a, P>;
    fn len(&self) -> usize;
}

impl<const M: usize> Str<M> for String {
    fn from_str(s: &str) -> Self {
        String::from(s)
    }

    fn as_str(&self) -> &str {
        self.as_str()
    }

    fn push(&mut self, c: char) {
        self.push(c)
    }

    fn push_str(&mut self, s: &str) {
        self.push_str(s)
    }

    fn characters(&self) -> Chars<'_> {
        self.chars()
    }

    fn split_at_pattern<'a, P: Pattern<'a>>(&'a self, pat: P) -> Split<'a, P> {
        self.split(pat)
    }

    fn len(&self) -> usize {
        self.len()
    }
}
impl<const M: usize> Str<M> for ArrayString<M> {
    fn from_str(s: &str) -> Self {
        Self::from(s).expect("error")
    }

    fn as_str(&self) -> &str {
        self.as_str()
    }

    fn push(&mut self, c: char) {
        self.push(c)
    }

    fn push_str(&mut self, s: &str) {
        self.push_str(s)
    }

    fn characters(&self) -> Chars<'_> {
        self.chars()
    }

    fn split_at_pattern<'a, P: Pattern<'a>>(&'a self, pat: P) -> Split<'a, P> {
        self.split(pat)
    }

    fn len(&self) -> usize {
        self.len()
    }
}


