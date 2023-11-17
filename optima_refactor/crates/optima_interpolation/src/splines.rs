use ad_trait::AD;
use optima_linalg::OVec;
use crate::InterpolatorTrait;

#[derive(Clone, Debug)]
pub struct InterpolatingSpline<T: AD, V: OVec<T>> {
    control_points: Vec<V>,
    spline_segment_a_coefficients: Vec<Vec<V>>,
    spline_type: InterpolatingSplineType<T>,
    num_spline_segments: usize
}
impl<T: AD, V: OVec<T>> InterpolatingSpline<T, V> {
    pub fn new(control_points: Vec<V>, spline_type: InterpolatingSplineType<T>) -> Self {
        let num_points = control_points.len();
        assert!(num_points > 0);
        let legal = (num_points - spline_type.num_overlap_between_segments()) % (spline_type.num_control_points_per_segment() - spline_type.num_overlap_between_segments()) == 0;
        if !legal {
            let mut control_points_clone = control_points.clone();
            control_points_clone.push(control_points.last().unwrap().clone());
            return Self::new(control_points_clone, spline_type);
        }
        let num_spline_segments = (num_points - spline_type.num_overlap_between_segments()) / (spline_type.num_control_points_per_segment() - spline_type.num_overlap_between_segments());
        let control_point_dim = control_points[0].len();

        let spline_segment_a_coefficients = vec![vec![V::from_slice_ovec(&vec![T::zero(); control_point_dim]); spline_type.num_control_points_per_segment()]; num_spline_segments];

        let mut out_self = Self {
            control_points,
            spline_segment_a_coefficients,
            spline_type,
            num_spline_segments
        };

        for i in 0..num_spline_segments {
            out_self.calculate_a_vector_coefficients(i);
        }

        out_self
    }
    #[inline]
    pub fn update_control_point(&mut self, idx: usize, control_point: V) {
        self.control_points[idx] = control_point;
        let spline_segment_idxs = self.map_control_point_idx_to_spline_segment_idxs(idx);
        for s in spline_segment_idxs {
            self.calculate_a_vector_coefficients(s);
        }
    }
    #[inline]
    fn interpolating_spline_interpolate(&self, t: T) -> V {
        if t == self.max_allowable_t_value() { return self.interpolating_spline_interpolate(t - T::constant(0.00000001)); }

        assert!(t >= T::zero());
        let rt = t.fract();
        let spline_segment_idx = t.floor().to_constant() as usize;
        assert!(spline_segment_idx < self.num_spline_segments, "t: {}", t);

        let a_vecs = &self.spline_segment_a_coefficients[spline_segment_idx];

        let mut out = V::from_slice_ovec(&vec![T::zero(); a_vecs[0].len()]);
        for (i, a) in a_vecs.iter().enumerate() {
            out = out.add(&a.scalar_mul(&rt.powi(i as i32)));
        }

        return out;
    }
    #[inline]
    pub fn map_control_point_idx_to_spline_segment_idxs(&self, control_point_idx: usize) -> Vec<usize> {
        assert!(control_point_idx < self.control_points.len());

        let num_control_points_per_segment = self.spline_type.num_control_points_per_segment();
        let num_overlap_between_segments = self.spline_type.num_overlap_between_segments();

        let a = control_point_idx % (num_control_points_per_segment - num_overlap_between_segments);
        let b = control_point_idx / (num_control_points_per_segment - num_overlap_between_segments);

        let dis_from_segment_edge_option1 = a;
        let dis_from_segment_edge_option2 = num_control_points_per_segment - a;
        let dis_from_either_edge_of_segment = usize::min(dis_from_segment_edge_option1, dis_from_segment_edge_option2);

        if dis_from_either_edge_of_segment >= num_overlap_between_segments { return vec![b]; }

        assert_ne!(dis_from_segment_edge_option1, dis_from_segment_edge_option2);
        if dis_from_segment_edge_option1 < dis_from_segment_edge_option2 && b == 0 {
            return vec![b]
        }
        if dis_from_segment_edge_option1 < dis_from_segment_edge_option2 && b == self.num_spline_segments {
            return vec![b-1];
        }
        if dis_from_segment_edge_option2 < dis_from_segment_edge_option1 && b == self.num_spline_segments {
            return vec![b-1];
        }

        return vec![b-1, b];
    }
    #[inline]
    pub fn map_control_point_idx_to_idx_in_spline_segments(&self, control_point_idx: usize) -> Vec<(usize, usize)> {
        let mut out = vec![];
        let spline_segment_idxs = self.map_control_point_idx_to_spline_segment_idxs(control_point_idx);

        for spline_segment_idx in spline_segment_idxs {
            let control_point_idxs = self.map_spline_segment_idx_to_control_point_idxs(spline_segment_idx);
            let idx = control_point_idxs.iter().position(|x| *x == control_point_idx ).unwrap();
            out.push((spline_segment_idx, idx));
        }

        out
    }
    #[inline]
    pub fn map_spline_segment_idx_to_control_point_idxs(&self, spline_segment_idx: usize) -> Vec<usize> {
        assert!(spline_segment_idx < self.num_spline_segments, "idx {}, num_spline_segments {}", spline_segment_idx, self.num_spline_segments);

        let num_control_points_per_segment = self.spline_type.num_control_points_per_segment();
        let num_overlap_between_segments = self.spline_type.num_overlap_between_segments();

        let start = (num_control_points_per_segment - num_overlap_between_segments) * spline_segment_idx;

        let mut out_vec = vec![];
        for i in 0..num_control_points_per_segment {
            out_vec.push(start + i);
        }

        out_vec
    }
    #[inline]
    fn calculate_a_vector_coefficients(&mut self, spline_segment_idx: usize) {
        let control_point_idxs = self.map_spline_segment_idx_to_control_point_idxs(spline_segment_idx);
        let control_point_refs: Vec<&V> = control_point_idxs.iter().map(|x| &self.control_points[*x]).collect();
        let basis_matrix = self.spline_type.basis_matrix();

        match self.spline_type {
            InterpolatingSplineType::Linear => {
                let a_vecs = calculate_a_vector_coefficients_generic(&control_point_refs, &basis_matrix);
                self.spline_segment_a_coefficients[spline_segment_idx] = a_vecs;
            }
            InterpolatingSplineType::Quadratic => {
                let a_vecs = calculate_a_vector_coefficients_generic(&control_point_refs, &basis_matrix);
                self.spline_segment_a_coefficients[spline_segment_idx] = a_vecs;
            }
            InterpolatingSplineType::HermiteCubic => {
                let a_vecs = calculate_a_vector_coefficients_generic(&control_point_refs, &basis_matrix);
                self.spline_segment_a_coefficients[spline_segment_idx] = a_vecs;
            }
            InterpolatingSplineType::NaturalCubic => {
                let a_vecs = calculate_a_vector_coefficients_generic(&control_point_refs, &basis_matrix);
                self.spline_segment_a_coefficients[spline_segment_idx] = a_vecs;
            }
            InterpolatingSplineType::CardinalCubic { w } => {
                let p1 = control_point_refs[0];
                let d_p1_d_t = control_point_refs[1];
                let p2 = control_point_refs[2];
                let d_p2_d_t = control_point_refs[3];

                // let p0 = p2 - (2.0/(1.0-w))* d_p1_d_t;
                // let p3 = p1 + (2.0/(1.0-w))* d_p2_d_t;

                let p0 = p2.sub(  &d_p1_d_t.scalar_mul(&T::constant(2.0 / 1.0-w.to_constant()))  );
                let p3 = p1.add(  &d_p2_d_t.scalar_mul( &T::constant(2.0/(1.0-w.to_constant())) )  );

                let control_points = vec![&p0, p1, p2, &p3];
                let a_vecs = calculate_a_vector_coefficients_generic(&control_points, &basis_matrix);
                self.spline_segment_a_coefficients[spline_segment_idx] = a_vecs;
            }
            InterpolatingSplineType::BezierCubic => {
                let p0 = control_point_refs[0];
                let d_p0_d_t = control_point_refs[1];
                let p3 = control_point_refs[2];
                let d_p3_d_t = control_point_refs[3];

                // let p1 = (1./3.)*d_p0_d_t + p0;
                // let p2 = p3 - (1./3.)*d_p3_d_t;

                let p1 = d_p0_d_t.scalar_mul( &(T::constant(1./3.)) ).add( p0 );
                let p2 = p3.sub( &d_p3_d_t.scalar_mul(&T::constant(1./3.)) );

                let control_points = vec![p0, &p1, &p2, p3];
                let a_vecs = calculate_a_vector_coefficients_generic(&control_points, &basis_matrix);
                self.spline_segment_a_coefficients[spline_segment_idx] = a_vecs;
            }
        }
    }
    #[inline]
    pub fn spline_type(&self) -> InterpolatingSplineType<T> {
        self.spline_type
    }
    #[inline]
    pub fn num_spline_segments(&self) -> usize {
        self.num_spline_segments
    }
    #[inline]
    pub fn control_points(&self) -> &Vec<V> {
        &self.control_points
    }
    #[inline]
    fn max_allowable_t_value(&self) -> T {
        return T::constant(self.num_spline_segments as f64);
    }
}
impl<T: AD, V: OVec<T>> InterpolatorTrait<T, V> for InterpolatingSpline<T, V> {
    fn interpolate(&self, t: T) -> V {
        self.interpolating_spline_interpolate(t)
    }

    fn max_t(&self) -> T {
        self.max_allowable_t_value()
    }
}

fn calculate_a_vector_coefficients_generic<T: AD, V: OVec<T>>(p: &Vec<&V>, basis_matrix: &Vec<Vec<T>>) -> Vec<V> {
    assert!(p.len() > 0);
    let mut out_vec = vec![];

    for row in basis_matrix {
        assert_eq!(row.len(), p.len());
        let mut a = V::from_slice_ovec(&vec![T::zero(); p[0].len()]);
        for (i, value) in row.iter().enumerate() {
            a = a.add( &p[i].scalar_mul(value) );
        }
        out_vec.push(a);
    }

    out_vec
}

#[derive(Clone, Debug, Copy)]
pub enum InterpolatingSplineType<T: AD> {
    Linear,
    Quadratic,
    HermiteCubic,
    NaturalCubic,
    CardinalCubic{ w: T },
    BezierCubic
}
impl<T: AD> InterpolatingSplineType<T> {
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
    #[inline(always)]
    pub fn basis_matrix(&self) -> Vec<Vec<T>> {
        match self {
            InterpolatingSplineType::Linear => { vec![vec![T::constant(1.0), T::constant(0.0)], vec![T::constant(-1.0), T::constant(1.0)]] }
            InterpolatingSplineType::Quadratic => { vec![vec![T::constant(1.0),T::constant(0.0), T::constant(0.0)], vec![T::constant(-3.0), T::constant(4.0), T::constant(-1.0)], vec![T::constant(2.0), T::constant(-4.0), T::constant(2.0)]] }
            InterpolatingSplineType::HermiteCubic => { vec![vec![T::constant(1.),T::constant(0.),T::constant(0.),T::constant(0.)], vec![T::constant(0.),T::constant(1.),T::constant(0.),T::constant(0.)], vec![T::constant(-3.),T::constant(-2.),T::constant(3.),T::constant(-1.)], vec![T::constant(2.),T::constant(1.),T::constant(-2.),T::constant(1.)]] }
            InterpolatingSplineType::NaturalCubic => { vec![vec![T::constant(1.),T::constant(0.),T::constant(0.),T::constant(0.)], vec![T::constant(0.),T::constant(1.),T::constant(0.),T::constant(0.)], vec![T::constant(0.),T::constant(0.),T::constant(0.5),T::constant(0.)], vec![T::constant(-1.),T::constant(-1.),T::constant(-0.5),T::constant(1.)]] }
            InterpolatingSplineType::CardinalCubic { w } => { vec![vec![T::constant(0.),T::constant(1.),T::constant(0.),T::constant(0.)], vec![T::constant((w.to_constant()-1.0)/2.0),T::constant(0.),T::constant((1.0 - w.to_constant())/2.),T::constant(0.)], vec![T::constant(1. -w.to_constant()),T::constant(0.5*(-w.to_constant() - 5.)),T::constant(w.to_constant()+2.),T::constant((w.to_constant()-1.)/2.)], vec![T::constant((w.to_constant()-1.)/2.),T::constant((w.to_constant()+3.)/2.),T::constant(0.5*(-w.to_constant() - 3.)),T::constant((1.-w.to_constant())/2.)]]  }
            InterpolatingSplineType::BezierCubic => { vec![vec![T::constant(1.),T::constant(0.),T::constant(0.),T::constant(0.)], vec![T::constant(-3.),T::constant(3.),T::constant(0.),T::constant(0.)], vec![T::constant(3.),T::constant(-6.),T::constant(3.),T::constant(0.)], vec![T::constant(-1.),T::constant(3.),T::constant(-3.),T::constant(1.)]] }
        }
    }
}

#[derive(Clone, Debug)]
pub struct BSpline<T: AD, V: OVec<T>> {
    control_points: Vec<V>,
    knot_vector: Vec<T>,
    k: usize,
    k_2: T
}
impl<T: AD, V: OVec<T>> BSpline<T, V> {
    pub fn new(control_points: Vec<V>, k: usize) -> Self {
        assert!(k > 1);
        let mut knot_vector = vec![];
        let k_2 = k as f64 / 2.0;
        for i in 0..(k+control_points.len()) {
            knot_vector.push( T::constant(-k_2 + i as f64))
        }
        Self {
            control_points,
            knot_vector,
            k,
            k_2: T::constant(k_2)
        }
    }
    #[inline]
    pub fn cox_de_boor_recurrence(&self, i: usize, k: usize, t: T) -> T {
        assert!(k > 0);
        if k == 1 {
            return if self.knot_vector[i] <= t && t < self.knot_vector[i + 1] { T::constant(1.0) } else { T::constant(0.0) }
        }

        let c0 = (t - self.knot_vector[i]) / (self.knot_vector[i+k-1] - self.knot_vector[i]);
        let c1 = (self.knot_vector[i+k] - t) / (self.knot_vector[i+k] - self.knot_vector[i+1]);

        return c0 * self.cox_de_boor_recurrence(i, k-1, t) + c1 * self.cox_de_boor_recurrence(i+1, k-1, t);
    }
    #[inline]
    fn bspline_interpolate(&self, t: T) -> V {
        let k_2 = self.k_2;

        let mut out_sum = V::from_slice_ovec(&vec![T::zero(); self.control_points[0].len()]);
        for (control_point_idx, control_point) in self.control_points.iter().enumerate() {
            let c = T::constant(control_point_idx as f64);
            if c - k_2 <= t && t < c + k_2 {
                // out_sum += control_point*self.cox_de_boor_recurrence(control_point_idx, self.k, t);
                out_sum = out_sum.add(&control_point.scalar_mul( &self.cox_de_boor_recurrence(control_point_idx, self.k, t) ));
            }
        }
        out_sum
    }
    #[inline(always)]
    pub fn update_control_point(&mut self, idx: usize, control_point: V) {
        self.control_points[idx] = control_point;
    }
    #[inline(always)]
    pub fn control_points(&self) -> &Vec<V> {
        &self.control_points
    }
    #[inline(always)]
    fn max_allowable_t_value(&self) -> T {
        return T::constant(self.control_points.len() as f64 - 1.0);
    }
}
impl<T: AD, V: OVec<T>> InterpolatorTrait<T, V> for BSpline<T, V> {
    #[inline]
    fn interpolate(&self, t: T) -> V {
        self.bspline_interpolate(t)
    }

    #[inline(always)]
    fn max_t(&self) -> T {
        self.max_allowable_t_value()
    }
}