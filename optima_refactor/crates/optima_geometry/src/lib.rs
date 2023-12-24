use ad_trait::AD;
use optima_interpolation::get_interpolation_range;
use optima_linalg::OVec;
use optima_sampling::SimpleSampler;

#[inline(always)]
pub fn pt_dis_to_line<T: AD, V: OVec<T>>(pt: &V, a: &V, b: &V, clamp: bool) -> (T, V) {
    let b_minus_a = b.ovec_sub(a);
    let pt_minus_a = pt.ovec_sub(a);

    let p = proj(&pt_minus_a, &b_minus_a, clamp).ovec_add(&a);
    let dis = p.ovec_sub(&pt).ovec_p_norm(&T::constant(2.0));

    (dis, p)
}

/// project v onto u
#[inline(always)]
pub fn proj<T: AD, V: OVec<T>>(v: &V, u: &V, clamp: bool) -> V {
    let mut proj = proj_scalar(v, u);
    if clamp { proj = proj.clamp(T::zero(), T::one()); }

    return u.ovec_scalar_mul(&proj);
}

#[inline(always)]
pub fn proj_scalar<T: AD, V: OVec<T>>(v: &V, u: &V) -> T {
    let n = v.ovec_dot(u);
    let d = u.ovec_dot(u).max(T::constant(f64::MIN));
    return n/d;
}

#[inline(always)]
pub fn get_orthonormal_basis<T: AD, V: OVec<T>>(initial_vector: &V, basis_dim: usize, seed: Option<u64>) -> Vec<V> {
    let dim = initial_vector.len();
    let basis_dim_copy = basis_dim.min(dim);

    let mut out_vecs = vec![initial_vector.clone()];
    for i in 0..basis_dim_copy - 1 {
        match seed {
            None => {
                let s = SimpleSampler::uniform_samples(&vec![(T::constant(-1.0), T::constant(1.0)); dim], None);
                out_vecs.push( V::ovec_from_slice(&s) );
            }
            Some(seed) => {
                let s = SimpleSampler::uniform_samples(&vec![(T::constant(-1.0), T::constant(1.0)); dim], Some(seed + i as u64));
                out_vecs.push( V::ovec_from_slice(&s ) );
            }
        }
    }

    for i in 0..basis_dim_copy {
        let tmp1 = out_vecs[i].clone();
        for j in 0..i {
            let tmp2 = out_vecs[j].clone();
            // out_vecs[i] -= &proj(&tmp1, &tmp2)
            out_vecs[i] = out_vecs[i].ovec_sub(&proj(&tmp1, &tmp2, false));
        }
    }

    for i in 0..basis_dim_copy {
        // out_vecs[i].normalize_mut();
        // out_vecs[i] = out_vecs[i].scalar_mul();
        out_vecs[i] = out_vecs[i].normalize();
    }

    out_vecs
}

#[inline(always)]
pub fn get_points_around_circle<T: AD, V: OVec<T>>(center_point: &V, rotation_axis: &V, circle_radius: T, num_samples: usize, seed: Option<u64>) -> Vec<V> {
    assert!(center_point.len() > 2);
    assert_eq!(center_point.len(), rotation_axis.len());

    let basis = get_orthonormal_basis(rotation_axis, 3, seed);

    let local_x = basis[1].clone();
    let local_y = basis[2].clone();

    let step_size = T::constant(2.0*std::f64::consts::PI / (num_samples as f64));
    let range = get_interpolation_range(T::zero(), T::constant(2.0*std::f64::consts::PI), step_size);

    let mut out_points = vec![];

    let l = range.len();
    for i in 0..l {
        let point = local_x.ovec_scalar_mul(&(circle_radius * range[i].cos())).ovec_add(&local_y.ovec_scalar_mul(&(circle_radius * range[i].sin())));
        out_points.push(point.ovec_add(&center_point));
    }

    out_points
}