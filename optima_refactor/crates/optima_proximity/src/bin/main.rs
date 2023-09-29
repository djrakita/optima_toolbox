use parry_ad::na::{Vector3};
use parry_ad::shape::{Ball, Cuboid};
use optima_3d_spatial::optima_3d_pose::{ImplicitDualQuaternion, O3DPose};
use optima_proximity::{OShapeParry, OShapeParryWithOffset, OPairwiseShapeDisTrait};

fn main() {
    let a = Cuboid::<f64>::new(Vector3::new(1.,1.,1.));
    let b = Ball::new(1.0);

    let wa = OShapeParry::new(a);
    let wb = OShapeParryWithOffset::new(b, ImplicitDualQuaternion::from_constructors(&[3.0, 0.0, 0.0], &[0.0; 3]));

    let res = wb.distance(&ImplicitDualQuaternion::identity(), &wa, &ImplicitDualQuaternion::identity());
    println!("{:?}", res);
}