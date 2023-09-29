/*

use std::marker::PhantomData;
use ad_trait::AD;
use optima_linalg::OVec;

pub trait SplineTrait<T: AD, V: OVec<T>> {
    fn interpolate(&self, t: f64) -> V;
    fn max_allowable_t_value(&self) -> T;
}

pub struct InterpolatingSpline<T: AD, V: OVec<T>> {
    control_points: Vec<V>,
    num_spline_segments: usize,
    _phantom_data: PhantomData<T>
}

/*
#[derive(Clone, Debug, Copy)]
pub enum InterpolatingSplineType {
    Linear,
    Quadratic,
    HermiteCubic,
    NaturalCubic,
    CardinalCubic{ w: f64 },
    BezierCubic
}
impl InterpolatingSplineType {
    #[inline(always)]
    pub fn num_control_points_per_segment(&self) -> usize {
        match self {
            InterpolatingSplineType::Linear => { 2 }
            InterpolatingSplineType::Quadratic => { 3 }
            InterpolatingSplineType::HermiteCubic => { 4 }
            InterpolatingSplineType::NaturalCubic => { 4 }
            InterpolatingSplineType::CardinalCubic { .. } => { 4 }
            InterpolatingSplineType::BezierCubic => { 4 }
        }
    }
    #[inline(always)]
    pub fn num_overlap_between_segments(&self) -> usize {
        match self {
            InterpolatingSplineType::Linear => { 1 }
            InterpolatingSplineType::Quadratic => { 1 }
            InterpolatingSplineType::HermiteCubic => { 2 }
            InterpolatingSplineType::NaturalCubic => { 1 }
            InterpolatingSplineType::CardinalCubic { .. } => { 2 }
            InterpolatingSplineType::BezierCubic => { 2 }
        }
    }
    /// Returns this "matrix" in rows
    #[inline(always)]
    pub fn basis_matrix(&self) -> Vec<Vec<f64>> {
        match self {
            InterpolatingSplineType::Linear => { vec![vec![1.0, 0.0], vec![-1.0, 1.0]] }
            InterpolatingSplineType::Quadratic => { vec![vec![1.0, 0.0, 0.0], vec![-3.0, 4.0, -1.0], vec![2.0, -4.0, 2.0]] }
            InterpolatingSplineType::HermiteCubic => { vec![vec![1.,0.,0.,0.], vec![0.,1.,0.,0.], vec![-3.,-2.,3.,-1.], vec![2.,1.,-2.,1.]] }
            InterpolatingSplineType::NaturalCubic => { vec![vec![1.,0.,0.,0.], vec![0.,1.,0.,0.], vec![0.,0.,0.5,0.], vec![-1.,-1.,-0.5,1.]] }
            InterpolatingSplineType::CardinalCubic { w } => { vec![vec![0.,1.,0.,0.], vec![(*w-1.0)/2.0,0.,(1.0 - *w)/2.,0.], vec![1. -*w,0.5*(-*w - 5.),*w+2.,(*w-1.)/2.], vec![(*w-1.)/2.,(*w+3.)/2.,0.5*(-*w - 3.),(1.-*w)/2.]]  }
            InterpolatingSplineType::BezierCubic => { vec![vec![1.,0.,0.,0.], vec![-3.,3.,0.,0.], vec![3.,-6.,3.,0.], vec![-1.,3.,-3.,1.]] }
        }
    }
}
*/





 */