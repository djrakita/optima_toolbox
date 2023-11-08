use ad_trait::AD;
use rand::Rng;
use rand_chacha::ChaCha20Rng;
use rand_chacha::rand_core::SeedableRng;
use rand_distr::Normal;
use rand_distr::Distribution;

pub fn get_rng(random_seed: Option<u64>) -> ChaCha20Rng {
    match random_seed {
        None => { ChaCha20Rng::from_entropy() }
        Some(seed) => { ChaCha20Rng::seed_from_u64(seed) }
    }
}

pub struct SimpleSampler;
impl SimpleSampler {
    pub fn uniform_samples<T: AD>(bounds: &Vec<(T, T)>, seed: Option<u64>) -> Vec<T> {
        let mut out_vec = vec![];

        let mut rng = get_rng(seed);
        for b in bounds {
            if b.0 == b.1 { out_vec.push(b.0) }
            else {
                let s = rng.gen_range(b.0.to_constant()..b.1.to_constant());
                out_vec.push(T::constant(s));
            }
        }

        out_vec
    }

    pub fn uniform_sample<T: AD>(bounds: (T, T), seed: Option<u64>) -> T {
        let mut rng = get_rng(seed);
        let s = rng.gen_range(bounds.0.to_constant()..bounds.1.to_constant());
        return T::constant(s);
    }

    pub fn normal_samples<T: AD>(means_and_standard_deviations: &Vec<(T, T)>, seed: Option<u64>) -> Vec<T> {
        let mut out_vec = vec![];
        let mut rng = get_rng(seed);
        for (mean, standard_deviation) in means_and_standard_deviations {
            let distribution = Normal::new(mean.to_constant(), standard_deviation.to_constant()).expect("error");
            out_vec.push(T::constant(distribution.sample(&mut rng)));
        }
        out_vec
    }

    pub fn normal_sample<T: AD>(mean: T, standard_deviation: T, seed: Option<u64>) -> T {
        let mut rng = get_rng(seed);
        let distribution = Normal::new(mean.to_constant(), standard_deviation.to_constant()).expect("error");
        T::constant(distribution.sample(&mut rng))
    }

    pub fn uniform_samples_i32(bounds: &Vec<(i32, i32)>, seed: Option<u64>) -> Vec<i32> {
        let bounds: Vec<(f64, f64)> = bounds.iter().map(|x| (x.0 as f64, x.1 as f64) ).collect();
        let float_samples = Self::uniform_samples(&bounds, seed);
        return float_samples.iter().map(|x| x.round() as i32).collect();
    }

    pub fn uniform_sample_i32(bounds: (i32, i32), seed: Option<u64>) -> i32 {
        let bounds = (bounds.0 as f64, bounds.1 as f64);
        let float_sample = Self::uniform_sample(bounds, seed);
        return float_sample.round() as i32;
    }

    pub fn uniform_samples_u64(bounds: &Vec<(u64, u64)>, seed: Option<u64>) -> Vec<u64> {
        let bounds: Vec<(f64, f64)> = bounds.iter().map(|x| (x.0 as f64, x.1 as f64) ).collect();
        let float_samples = Self::uniform_samples(&bounds, seed);
        return float_samples.iter().map(|x| x.round() as u64).collect();
    }

    pub fn uniform_sample_u64(bounds: (u64, u64), seed: Option<u64>) -> u64 {
        let bounds = (bounds.0 as f64, bounds.1 as f64);
        let float_sample = Self::uniform_sample(bounds, seed);
        return float_sample.round() as u64;
    }
}