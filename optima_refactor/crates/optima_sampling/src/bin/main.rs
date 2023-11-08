use optima_sampling::SimpleSampler;

fn main() {
    let s = SimpleSampler::uniform_sample_u64((u64::MIN, u64::MAX), None);
    println!("{:?}", s);
}