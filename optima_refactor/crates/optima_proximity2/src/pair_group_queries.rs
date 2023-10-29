use std::time::Instant;
use ad_trait::AD;
use as_any::AsAny;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use crate::shape_queries::{IntersectOutputTrait, OShpQryIntersectTrait};
use crate::shapes::{OParryShape, ParryIntersectOutput, ParryOutputAuxData, ParryQryShapeType, ParryShapeRep};

pub trait OPairGroupQryTrait<T: AD, P: O3DPose<T>> {
    type ShapeTypeA : AsAny;
    type ShapeTypeB : AsAny;
    type SelectorType : AsAny;
    type Args : AsAny;
    type Output : AsAny;

    fn query(&self, shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, args: &Self::Args) -> Self::Output;
}

pub trait OPairGroupTermination {
    type PairQryOutput : AsAny;

    fn terminate(&self, pair_query_output: &Self::PairQryOutput) -> bool;
}

#[derive(Clone, Debug)]
pub enum ParryPairSelector {
    AllPairs,
    HalfPairs,
    PairsByIdxs(Vec<(usize, usize)>),
    PairSubcomponentsByIdxs(Vec<((usize, usize), (usize, usize))>)
}

pub struct ParryPairGroupOutputWrapper<O> {
    output: O,
    pair_idxs: ParryPairIdxs
}
impl<O> ParryPairGroupOutputWrapper<O> {
    pub fn output(&self) -> &O {
        &self.output
    }
    pub fn pair_idxs(&self) -> &ParryPairIdxs {
        &self.pair_idxs
    }
}

#[derive(Debug, Clone)]
pub enum ParryPairIdxs {
    Shapes(usize, usize),
    ShapeSubcomponents((usize, usize), (usize, usize))
}

pub struct ParryIntersectGroupQry {
    parry_shape_rep: ParryShapeRep,
    terminate_on_first_intersection: bool,
    sort_outputs: bool
}
impl ParryIntersectGroupQry {
    pub fn new(parry_shape_rep: ParryShapeRep, terminate_on_first_intersection: bool, sort_outputs: bool) -> Self {
        Self { parry_shape_rep, terminate_on_first_intersection, sort_outputs }
    }
}
impl<T: AD, P: O3DPose<T>> OPairGroupQryTrait<T, P> for ParryIntersectGroupQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Args = ();
    type Output = ParryIntersectGroupOutput;

    fn query(&self, shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, _args: &Self::Args) -> Self::Output {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryIntersectOutput {
            shape_a.intersect(shape_b, pose_a, pose_b, &(parry_qry_shape_type.clone(), parry_shape_rep.clone()))
        };

        let termination = if self.terminate_on_first_intersection {
            |o: &ParryIntersectOutput| {
                if o.intersect() { return true } else { false }
            }
        } else {
            |_o: &ParryIntersectOutput| {
                false
            }
        };

        let (mut outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &self.parry_shape_rep, f, termination);

        if self.sort_outputs { outputs.sort_by(|x, y| x.output.partial_cmp(&y.output).unwrap()) }

        ParryIntersectGroupOutput {
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        }
    }
}

pub struct ParryIntersectGroupOutput {
    outputs: Vec<ParryPairGroupOutputWrapper<ParryIntersectOutput>>,
    aux_data: ParryOutputAuxData
}
impl ParryIntersectGroupOutput {
    pub fn outputs(&self) -> &Vec<ParryPairGroupOutputWrapper<ParryIntersectOutput>> {
        &self.outputs
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

#[inline]
fn parry_generic_pair_group_query<T: AD, P: O3DPose<T>, O: AsAny, F, Termination>(shape_group_a: &Vec<OParryShape<T, P>>,
                                                                                  shape_group_b: &Vec<OParryShape<T, P>>,
                                                                                  poses_a: &Vec<P>,
                                                                                  poses_b: &Vec<P>,
                                                                                  pair_selector: &ParryPairSelector,
                                                                                  parry_shape_rep: &ParryShapeRep,
                                                                                  f: F,
                                                                                  termination: Termination) -> (Vec<ParryPairGroupOutputWrapper<O>>, usize)
    where F: Fn(&OParryShape<T, P>, &OParryShape<T, P>, &P, &P, &ParryQryShapeType, &ParryShapeRep) -> O,
          Termination: Fn(&O) -> bool
{
    let mut out_vec = vec![];
    let mut count = 0;

    match pair_selector {
        ParryPairSelector::AllPairs => {
            'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                    count += 1;
                    let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::Standard, parry_shape_rep);
                    let terminate = termination(&o);
                    out_vec.push(ParryPairGroupOutputWrapper { output: o, pair_idxs: ParryPairIdxs::Shapes(i, j) });
                    if terminate { break 'l; }
                }
            }
        }
        ParryPairSelector::HalfPairs => {
            'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                    if i < j {
                        count += 1;
                        let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::Standard, parry_shape_rep);
                        let terminate = termination(&o);
                        out_vec.push(ParryPairGroupOutputWrapper { output: o, pair_idxs: ParryPairIdxs::Shapes(i, j) });
                        if terminate { break 'l; }
                    }
                }
            }
        }
        ParryPairSelector::PairsByIdxs(idx_pairs) => {
            'l: for idx_pair in idx_pairs {
                let idx0 = idx_pair.0;
                let idx1 = idx_pair.1;
                let shape_a = &shape_group_a[idx0];
                let shape_b = &shape_group_a[idx1];
                let pose_a = &poses_a[idx0];
                let pose_b = &poses_b[idx1];

                count += 1;
                let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::Standard, parry_shape_rep);
                let terminate = termination(&o);
                out_vec.push(ParryPairGroupOutputWrapper { output: o, pair_idxs: ParryPairIdxs::Shapes(idx0, idx1) });
                if terminate { break 'l; }
            }
        }
        ParryPairSelector::PairSubcomponentsByIdxs(idx_pairs) => {
            'l: for idx_pair in idx_pairs {
                let idxs0 = idx_pair.0;
                let idxs1 = idx_pair.1;
                let shape_a_idx = idxs0.0;
                let shape_a_subcomponent_idx = idxs0.1;
                let shape_b_idx = idxs1.0;
                let shape_b_subcomponent_idx = idxs1.1;

                let shape_a = &shape_group_a[shape_a_idx];
                let shape_b = &shape_group_a[shape_b_idx];
                let pose_a = &poses_a[shape_a_idx];
                let pose_b = &poses_b[shape_b_idx];

                count += 1;
                let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx, shape_b_subcomponent_idx }, parry_shape_rep);
                let terminate = termination(&o);
                out_vec.push(ParryPairGroupOutputWrapper { output: o, pair_idxs: ParryPairIdxs::ShapeSubcomponents((shape_a_idx, shape_a_subcomponent_idx), (shape_b_idx, shape_b_subcomponent_idx) ) });
                if terminate { break 'l; }
            }

        }
    }

    (out_vec, count)
}


/*
pub struct OParryPairGroupDefaultQry<T: AD, P: O3DPose<T>, Q: OPairQryTrait<T, P, ShapeTypeA=OParryShape<T, P>, ShapeTypeB=OParryShape<T, P>, Output : PartialOrd>, Termination: OPairGroupTermination<PairQryOutput = Q::Output>> {
    pair_query: Q,
    sort_outputs: bool,
    termination: Termination,
    phantom_data: PhantomData<(T, P)>
}
impl<T: AD, P: O3DPose<T>, Q: OPairQryTrait<T, P, ShapeTypeA=OParryShape<T, P>, ShapeTypeB=OParryShape<T, P>, Output: PartialOrd>, Termination: OPairGroupTermination<PairQryOutput = Q::Output>> OParryPairGroupDefaultQry<T, P, Q, Termination> {
    pub fn new(pair_query: Q, sort_outputs: bool, termination: Termination) -> Self {
        Self { pair_query, sort_outputs, termination, phantom_data: PhantomData::default() }
    }
}
impl<T: AD, P: O3DPose<T>, Q: OPairQryTrait<T, P, ShapeTypeA=OParryShape<T, P>, ShapeTypeB=OParryShape<T, P>, Output : PartialOrd> + 'static, Termination: OPairGroupTermination<PairQryOutput = Q::Output>> OPairGroupQryTrait<T, P> for OParryPairGroupDefaultQry<T, P, Q, Termination> {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Output = OParryPairGroupDefaultOutput<T, P, Q>;

    fn query(&self, shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &ParryPairSelector) -> Self::Output {
        let start = Instant::now();
        let mut count = 0;
        let mut outputs = vec![];

        match pair_selector {
            ParryPairSelector::AllPairs => {
                'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                    for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                        count += 1;
                        let output = self.pair_query.query(shape_a, shape_b, pose_a, pose_b);
                        let terminate = self.termination.terminate(&output);
                        outputs.push(PairGroupOutputWrapper { output, pair_idxs: (i, j) });
                        if terminate { break 'l; }
                    }
                }
            }
            ParryPairSelector::HalfPairs => {
                'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                    for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                        if i < j {
                            count += 1;
                            let output = self.pair_query.query(shape_a, shape_b, pose_a, pose_b);
                            let terminate = self.termination.terminate(&output);
                            outputs.push(PairGroupOutputWrapper { output, pair_idxs: (i, j) });
                            if terminate { break 'l; }
                        }
                    }
                }
            }
            ParryPairSelector::PairsByIdxs(idx_pairs) => {
                'l: for idx_pair in idx_pairs {
                    let idx_a = idx_pair.0;
                    let idx_b = idx_pair.1;

                    count += 1;
                    let output = self.pair_query.query(&shape_group_a[idx_a], &shape_group_b[idx_b], &poses_a[idx_a], &poses_b[idx_b]);
                    let terminate = self.termination.terminate(&output);
                    outputs.push(PairGroupOutputWrapper { output, pair_idxs: (idx_a, idx_b) });
                    if terminate { break 'l; }
                }
            }
            ParryPairSelector::PairSubcomponentsByIdxs(subcomponent_idx_pairs) => {
                'l: for subcomponent_idx_pair in subcomponent_idx_pairs {
                    let idxs_a = subcomponent_idx_pair.0;
                    let idxs_b = subcomponent_idx_pair.1;

                    let shape_idx_a = idxs_a.0;
                    let subcomponent_idx_a = idxs_a.1;

                    let shape_idx_b = idxs_b.0;
                    let subcomponent_idx_b = idxs_b.1;
                }
                todo!()
            }
        }

        if self.sort_outputs { outputs.sort_by(|x, y| x.output.partial_cmp(&y.output).unwrap()) }

        OParryPairGroupDefaultOutput {
            outputs,
            aux_data: ParryOutputAuxData { num_queries : count, duration: start.elapsed() },
        }
    }
}

pub struct OParryPairGroupDefaultOutput<T: AD, P: O3DPose<T>, Q: OPairQryTrait<T, P>> {
    outputs: Vec<PairGroupOutputWrapper<Q::Output>>,
    aux_data: ParryOutputAuxData
}
impl<T: AD, P: O3DPose<T>, Q: OPairQryTrait<T, P>> OParryPairGroupDefaultOutput<T, P, Q> {
    #[inline(always)]
    pub fn outputs(&self) -> &Vec<PairGroupOutputWrapper<Q::Output>> {
        &self.outputs
    }
    #[inline(always)]
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

pub struct NeverTerminate<O>(PhantomData<O>);
impl<O> NeverTerminate<O> {
    pub fn new() -> Self {
        Self(PhantomData::default())
    }
}
impl<O: 'static> OPairGroupTermination for NeverTerminate<O> {
    type PairQryOutput = O;

    fn terminate(&self, _pair_query_output: &Self::PairQryOutput) -> bool {
        false
    }
}
*/

