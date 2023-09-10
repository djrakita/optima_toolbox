use ad_trait::reverse_ad::adr::adr;
use nalgebra::DMatrix;
use optima_linalg::OMat;

fn main() {
    let m = DMatrix::from_column_slice(2, 2, &[1.,2.,3.,4.]).to_other_ad_type::<adr>();
    println!("{}", m);
}