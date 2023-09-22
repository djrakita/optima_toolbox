use optima_sampling::SimpleSampler;

fn main() {
    let s = SimpleSampler::uniform_sample_i32((0, 100), Some(4));
    println!("{:?}", s);
}