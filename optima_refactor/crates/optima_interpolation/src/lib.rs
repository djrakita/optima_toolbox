pub mod splines;

use std::marker::PhantomData;
use ad_trait::AD;
use optima_linalg::OVec;

pub trait InterpolatorTrait<T: AD, V: OVec<T>> : Clone {
    fn interpolate(&self, t: T) -> V;
    fn max_t(&self) -> T;
    fn interpolate_on_range(&self, range_start: T, range_stop: T, t: T) -> V {
        assert!(range_start <= t && t <= range_stop);
        let ratio = (t - range_start) / (range_stop - range_start);
        return self.interpolate_normalized(ratio);
    }
    fn interpolate_normalized(&self, u: T) -> V {
        assert!(T::zero() <= u && u <= T::one());

        let ratio = self.max_t() * u;
        return self.interpolate(ratio);
    }
    fn to_arclength_parameterized_interpolator(&self, num_arclength_markers: usize) -> ArclengthParameterizedInterpolator<T, V, Self> {
        ArclengthParameterizedInterpolator::new(self.clone(), num_arclength_markers)
    }
    fn to_timed_interpolator(&self, max_time: T) -> TimedInterpolator<T, V, Self> {
        TimedInterpolator::new(self.clone(), max_time)
    }
}

#[derive(Clone)]
pub struct ArclengthParameterizedInterpolator<T: AD, V: OVec<T>, I: InterpolatorTrait<T, V>> {
    interpolator: I,
    arclength_markers: Vec<(T, T)>,
    total_arclength: T,
    phantom_data: PhantomData<V>
}
impl<T: AD, V: OVec<T>, I: InterpolatorTrait<T, V>> ArclengthParameterizedInterpolator<T, V, I> {
    pub fn new(interpolator: I, num_arclength_markers: usize) -> Self {
        assert!(num_arclength_markers > 10);

        let mut arclength_markers = vec![];

        let mut t = T::zero();
        let max_allowable_t_value = interpolator.max_t();
        let step_size = max_allowable_t_value / T::constant(num_arclength_markers as f64);
        let mut accumulated_distance = T::zero();

        let mut prev_point = interpolator.interpolate(T::zero());

        let mut passed_m = false;
        while !passed_m {
            if t >= max_allowable_t_value {
                passed_m = true;
                t = max_allowable_t_value;
            }
            let curr_point = interpolator.interpolate(t);
            // let dis = (&curr_point - &prev_point).norm();
            let dis = curr_point.ovec_sub(&prev_point).ovec_p_norm(&T::constant(2.0));
            accumulated_distance += dis;

            arclength_markers.push((accumulated_distance, t));

            prev_point = curr_point;
            t += step_size;
        }

        Self { interpolator, arclength_markers, total_arclength: accumulated_distance, phantom_data: PhantomData::default() }
    }
}
impl<T: AD, V: OVec<T>, I: InterpolatorTrait<T, V>> InterpolatorTrait<T, V> for ArclengthParameterizedInterpolator<T, V, I> {
    fn interpolate(&self, s: T) -> V {
        assert!( T::zero() <= s && s <= T::one() );

        let r = s * self.total_arclength;

        let binary_search_res = self.arclength_markers.binary_search_by(|x| x.0.partial_cmp(&r).unwrap());
        // let binary_search_res = self.arclength_markers.binary_search_by(|x| r.partial_cmp(&x.0).unwrap() );

        return match binary_search_res {
            Ok(idx) => {
                self.interpolator.interpolate(self.arclength_markers[idx].1)
            }
            Err(idx) => {
                if idx == 0 { return self.interpolator.interpolate(T::zero()) }
                let upper_bound_idx = idx;
                let lower_bound_idx = idx - 1;

                // if upper_bound_idx == self.arclength_markers.len() { return self.interpolator.interpolate(self.interpolator.max_t()); }

                let upper_bound_dis = self.arclength_markers[upper_bound_idx].0;
                let lower_bound_dis = self.arclength_markers[lower_bound_idx].0;

                assert!(lower_bound_dis <= r && r <= upper_bound_dis);

                let upper_bound_t = self.arclength_markers[upper_bound_idx].1;
                let lower_bound_t = self.arclength_markers[lower_bound_idx].1;

                let dis_ratio = (r - lower_bound_dis) / (upper_bound_dis - lower_bound_dis);

                let t = lower_bound_t + dis_ratio * (upper_bound_t - lower_bound_t);

                self.interpolator.interpolate(t)
            }
        }
    }

    fn max_t(&self) -> T {
        T::one()
    }
}

#[derive(Clone)]
pub struct TimedInterpolator<T: AD, V: OVec<T>, I: InterpolatorTrait<T, V>> {
    interpolator: I,
    max_time: T,
    phantom_data: PhantomData<V>
}
impl<T: AD, V: OVec<T>, I: InterpolatorTrait<T, V>> TimedInterpolator<T, V, I> {
    pub fn new(interpolator: I, max_time: T) -> Self {
        Self { interpolator, max_time, phantom_data: PhantomData::default() }
    }
}
impl<T: AD, V: OVec<T>, I: InterpolatorTrait<T, V>> InterpolatorTrait<T, V> for TimedInterpolator<T, V, I> {
    fn interpolate(&self, t: T) -> V {
        self.interpolator.interpolate_on_range(T::zero(), self.max_time, t)
    }

    fn max_t(&self) -> T {
        self.max_time
    }
}

/*
pub struct SpacetimeInterpolator<T: AD, V: OVec<T>, SI: InterpolatorTrait<T, V>, TI: InterpolatorTrait<T, V>> {
    space_interpolator: SI,
    time_interpolator: TI,
    max_time: T,
    phantom_data: PhantomData<V>
}
impl<T: AD, V: OVec<T>, SI: InterpolatorTrait<T, V>, TI: InterpolatorTrait<T, V>> SpacetimeInterpolator<T, V, SI, TI> {
    pub fn new(space_interpolator: SI, time_interpolator: TI) -> Self {
        let v = time_interpolator.interpolate_normalized(time_interpolator.max_t());
        assert_eq!(v.len(), 1);
        Self { space_interpolator, time_interpolator, max_time: *v.get_element(0), phantom_data: Default::default() }
    }
    fn spacetime_interpolate(&self, t: T) -> V {
        let binding = self.time_interpolator.interpolate_on_range(T::zero(), self.max_time, t);
        let t2 = binding.get_element(0);
        let ratio = *t2 / self.max_time;
        return self.space_interpolator.interpolate_normalized(ratio)
    }
}
impl<T: AD, V: OVec<T>, SI: InterpolatorTrait<T, V>, TI: InterpolatorTrait<T, V>> InterpolatorTrait<T, V> for SpacetimeInterpolator<T, V, SI, TI> {
    fn interpolate(&self, t: T) -> V {
        self.spacetime_interpolate(t)
    }

    fn max_t(&self) -> T {
        self.max_time
    }
}
*/

pub fn get_range<T: AD>(range_start: T, range_stop: T, step_size: T) -> Vec<T> {
    let mut out_range = Vec::new();
    out_range.push(range_start);
    let mut last_added_val = range_start;

    while !( (range_stop - last_added_val).abs() < step_size ) {
        if range_stop > range_start {
            last_added_val = last_added_val + step_size;
        } else {
            last_added_val = last_added_val - step_size;
        }
        out_range.push(last_added_val);
    }

    out_range.push(range_stop);

    out_range
}