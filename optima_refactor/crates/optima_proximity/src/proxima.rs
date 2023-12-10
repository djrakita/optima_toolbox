use ad_trait::AD;
use serde::{Deserialize, Serialize};
use crate::pair_queries::{ParryDisMode, ParryShapeRep};
use serde_with::serde_as;
use ad_trait::SerdeAD;

pub struct ParryProximaDistanceGroupQry;

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct ParryProximaDistanceGroupArgs<T: AD> {
    parry_shape_rep: ParryShapeRep,
    parry_dis_mode: ParryDisMode,
    use_average_distance: bool,
    #[serde_as(as = "SerdeAD<T>")]
    termination_distance_threshold: T
}
