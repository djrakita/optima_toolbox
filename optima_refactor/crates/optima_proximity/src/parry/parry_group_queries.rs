use std::marker::PhantomData;
use ad_trait::AD;
use parry_ad::na::Isometry3;
use crate::parry::parry_queries::{OParryContactQry, OParryContactQryArgs, OParryIntersectQry, OParryIntersectQryArgs, ParryShapeRepresentation};
use crate::parry::parry_shapes::OParryShp;
use crate::shape_group_query_traits::{OPairShpGroupQryTrait, OPairShpGroupQryElement, OShpGroupPairSelector, OShpGroupQryTermination, OPairShpGroupCullerTrait};
use crate::shape_query_traits::{OPairShpQryTrait};

pub struct OParryPairShpGroupQryDefault<T: AD, Q: OPairShpQryTrait<T>, Termination: OShpGroupQryTermination<Q::Output> + 'static> {
    sort_outputs: bool,
    culler: Option<OParryPairGenericCuller<T>>,
    args: Q::Args,
    phantom_data: PhantomData<Termination>
}
impl<T: AD, Q: OPairShpQryTrait<T>, Termination: OShpGroupQryTermination<Q::Output> + 'static> OParryPairShpGroupQryDefault<T, Q, Termination> {
    pub fn new(sort_outputs: bool, cullers: Vec<OParryPairCullerCategory<T>>, args: Q::Args) -> Self {
        let culler = if cullers.len() == 0 {
            None
        } else {
            let v: Vec<_> = cullers.iter().map(|x| x.to_parry_culler_box() ).collect();
            Some(OParryPairGenericCuller::new(v))
        };

        Self {
            sort_outputs,
            culler,
            args,
            phantom_data: Default::default(),
        }
    }
}
impl<T: AD, Q: OPairShpQryTrait<T, ShapeType=OParryShp<T, Isometry3<T>>, PoseType=Isometry3<T>> + 'static, Termination: OShpGroupQryTermination<Q::Output> + 'static> OPairShpGroupQryTrait<T> for OParryPairShpGroupQryDefault<T, Q, Termination> {
    type ShapeType = Q::ShapeType;
    type PoseType = Q::PoseType;
    type Output = Vec<OPairShpGroupQryElement<Q::Output>>;

    fn query(&self, shape_group_a: &Vec<&Self::ShapeType>, shape_group_b: &Vec<&Self::ShapeType>, poses_a: &Vec<Self::PoseType>, poses_b: &Vec<Self::PoseType>, selector: OShpGroupPairSelector) -> Self::Output {
        assert_eq!(shape_group_a.len(), poses_a.len());
        assert_eq!(shape_group_b.len(), poses_b.len());

        let mut out = vec![];

        let selector = match &self.culler {
            None => { selector }
            Some(culler) => {
                let idxs = culler.query(shape_group_a, shape_group_b, poses_a, poses_b, selector);
                OShpGroupPairSelector::GivenPairsByIdxs(idxs)
            }
        };

        match &selector {
            OShpGroupPairSelector::AllPairs => {
                'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                    for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                        let output = Q::query(shape_a, shape_b, pose_a, pose_b, &self.args);
                        let terminate = Termination::terminate(&output);
                        out.push( OPairShpGroupQryElement { shape_indices: (i, j), output } );
                        if terminate { break 'l; }
                    }
                }
            }
            OShpGroupPairSelector::HalfPairs => {
                'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                    for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                        if i < j {
                            let output = Q::query(shape_a, shape_b, pose_a, pose_b, &self.args);
                            let terminate = Termination::terminate(&output);
                            out.push(OPairShpGroupQryElement { shape_indices: (i, j), output });
                            if terminate { break 'l; }
                        }
                    }
                }
            }
            OShpGroupPairSelector::GivenPairsByIdxs(idxs) => {
                'l: for (i,j) in idxs.iter() {
                    let shape_a = shape_group_a[*i];
                    let shape_b = shape_group_b[*j];
                    let pose_a = &poses_a[*i];
                    let pose_b = &poses_b[*j];
                    let output = Q::query(shape_a, shape_b, pose_a, pose_b, &self.args);
                    let terminate = Termination::terminate(&output);
                    out.push(OPairShpGroupQryElement { shape_indices: (*i, *j), output });
                    if terminate { break 'l; }
                }
            }
        }

        if self.sort_outputs { out.sort(); }

        out
    }
}

pub struct OParryPairGenericCuller<T: AD> {
    cullers: Vec<Box<dyn OPairShpGroupCullerTrait<T, ShapeType=OParryShp<T, Isometry3<T>>, PoseType=Isometry3<T>, Output=Vec<(usize, usize)>>>>
}
impl<T: AD> OParryPairGenericCuller<T> {
    pub fn new(cullers: Vec<Box<dyn OPairShpGroupCullerTrait<T, ShapeType=OParryShp<T, Isometry3<T>>, PoseType=Isometry3<T>, Output=Vec<(usize, usize)>>>>) -> Self {
        assert!(cullers.len() > 0);
        Self { cullers }
    }
}
impl<T: AD> OPairShpGroupQryTrait<T> for OParryPairGenericCuller<T> {
    type ShapeType = OParryShp<T, Isometry3<T>>;
    type PoseType = Isometry3<T>;
    type Output = Vec<(usize, usize)>;

    fn query(&self, shape_group_a: &Vec<&Self::ShapeType>, shape_group_b: &Vec<&Self::ShapeType>, poses_a: &Vec<Self::PoseType>, poses_b: &Vec<Self::PoseType>, selector: OShpGroupPairSelector) -> Self::Output {
        let mut curr = self.cullers[0].query(shape_group_a, shape_group_b, poses_a, poses_b, selector);

        for i in 1..self.cullers.len() {
            curr = self.cullers[i].query(shape_group_a, shape_group_b, poses_a, poses_b, OShpGroupPairSelector::GivenPairsByIdxs(curr));
        }

        curr
    }
}
impl<T: AD> OPairShpGroupCullerTrait<T> for OParryPairGenericCuller<T> { }

pub struct OParryPairIntersectCuller {
    pub rep: ParryShapeRepresentation
}
impl<T: AD> OPairShpGroupQryTrait<T> for OParryPairIntersectCuller {
    type ShapeType = OParryShp<T, Isometry3<T>>;
    type PoseType = Isometry3<T>;
    type Output = Vec<(usize, usize)>;

    fn query(&self, shape_group_a: &Vec<&Self::ShapeType>, shape_group_b: &Vec<&Self::ShapeType>, poses_a: &Vec<Self::PoseType>, poses_b: &Vec<Self::PoseType>, selector: OShpGroupPairSelector) -> Self::Output {
        let mut out = vec![];

        match &selector {
            OShpGroupPairSelector::AllPairs => {
                for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                    for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                        let intersect = OParryIntersectQry::query(shape_a, shape_b, pose_a, pose_b, &OParryIntersectQryArgs { rep_a: self.rep.clone(), rep_b: self.rep.clone() });
                        if intersect { out.push((i,j)) }
                    }
                }
            }
            OShpGroupPairSelector::HalfPairs => {
                for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                    for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                        if i < j {
                            let intersect = OParryIntersectQry::query(shape_a, shape_b, pose_a, pose_b, &OParryIntersectQryArgs { rep_a: self.rep.clone(), rep_b: self.rep.clone() });
                            if intersect { out.push((i,j)) }
                        }
                    }
                }
            }
            OShpGroupPairSelector::GivenPairsByIdxs(idxs) => {
                for (i,j) in idxs.iter() {
                    let shape_a = shape_group_a[*i];
                    let shape_b = shape_group_b[*j];
                    let pose_a = &poses_a[*i];
                    let pose_b = &poses_b[*j];
                    let intersect = OParryIntersectQry::query(shape_a, shape_b, pose_a, pose_b, &OParryIntersectQryArgs { rep_a: self.rep.clone(), rep_b: self.rep.clone() });
                    if intersect { out.push((*i,*j)) }
                }
            }
        }

        out
    }
}
impl<T: AD> OPairShpGroupCullerTrait<T> for OParryPairIntersectCuller { }

pub struct OParryPairContactCuller<T: AD> {
    pub rep: ParryShapeRepresentation,
    pub threshold: T
}
impl<T: AD> OPairShpGroupQryTrait<T> for OParryPairContactCuller<T> {
    type ShapeType = OParryShp<T, Isometry3<T>>;
    type PoseType = Isometry3<T>;
    type Output = Vec<(usize, usize)>;

    fn query(&self, shape_group_a: &Vec<&Self::ShapeType>, shape_group_b: &Vec<&Self::ShapeType>, poses_a: &Vec<Self::PoseType>, poses_b: &Vec<Self::PoseType>, selector: OShpGroupPairSelector) -> Self::Output {
        let mut out = vec![];

        match &selector {
            OShpGroupPairSelector::AllPairs => {
                for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                    for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                        let c = OParryContactQry::query(shape_a, shape_b, pose_a, pose_b, &OParryContactQryArgs { rep_a: self.rep.clone(), rep_b: self.rep.clone(), threshold: self.threshold });
                        if c.0.is_some() { out.push((i,j)) }
                    }
                }
            }
            OShpGroupPairSelector::HalfPairs => {
                for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                    for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                        if i < j {
                            let c = OParryContactQry::query(shape_a, shape_b, pose_a, pose_b, &OParryContactQryArgs { rep_a: self.rep.clone(), rep_b: self.rep.clone(), threshold: self.threshold });
                            if c.0.is_some() { out.push((i,j)) }
                        }
                    }
                }
            }
            OShpGroupPairSelector::GivenPairsByIdxs(idxs) => {
                for (i,j) in idxs.iter() {
                    let shape_a = shape_group_a[*i];
                    let shape_b = shape_group_b[*j];
                    let pose_a = &poses_a[*i];
                    let pose_b = &poses_b[*j];
                    let c = OParryContactQry::query(shape_a, shape_b, pose_a, pose_b, &OParryContactQryArgs { rep_a: self.rep.clone(), rep_b: self.rep.clone(), threshold: self.threshold });
                    if c.0.is_some() { out.push((*i,*j)) }
                }
            }
        }

        out
    }
}
impl<T: AD> OPairShpGroupCullerTrait<T> for OParryPairContactCuller<T> { }

pub enum OParryPairCullerCategory<T: AD> {
    Intersect { rep: ParryShapeRepresentation },
    Contact { rep: ParryShapeRepresentation, threshold: T }
}
impl<T: AD> OParryPairCullerCategory<T> {
    pub fn to_parry_culler_box(&self) -> Box<dyn OPairShpGroupCullerTrait<T, Output=Vec<(usize, usize)>, PoseType=Isometry3<T>, ShapeType=OParryShp<T, Isometry3<T>>>> {
        match self {
            OParryPairCullerCategory::Intersect { rep  } => {
                Box::new( OParryPairIntersectCuller { rep: rep.clone() } )
            }
            OParryPairCullerCategory::Contact { rep, threshold } => {
                Box::new( OParryPairContactCuller { rep: rep.clone(), threshold: threshold.clone() } )
            }
        }
    }
}




