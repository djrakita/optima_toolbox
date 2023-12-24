use std::borrow::Cow;
use std::marker::PhantomData;
use ad_trait::AD;
use serde::{Deserialize, Serialize};
use serde_with::serde_as;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategory};
use optima_console::output::{get_default_progress_bar};
use optima_linalg::{OLinalgCategory, OVec};
use optima_proximity::pair_group_queries::{AHashMapWrapperSkipsWithReasonsTrait, OPairGroupQryTrait, ParryDistanceGroupArgs, ParryDistanceGroupQry, ParryIntersectGroupArgs, ParryIntersectGroupQry, ParryPairIdxs, ParryPairSelector, SkipReason};
use optima_proximity::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity::shape_queries::{DistanceOutputTrait, IntersectOutputTrait};
use optima_proximity::shape_scene::ShapeSceneTrait;
use optima_proximity::shapes::OParryShape;
use optima_universal_hashmap::AHashMapWrapper;
use crate::robot::{ORobot};

/*
#[serde_as]
#[derive(Clone, Serialize, Deserialize)]
pub struct ORobotParryShapeScene<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory> {
    #[serde(deserialize_with="Vec::<OParryShape::<T, C::P<T>>>::deserialize")]
    pub (crate) shapes: Vec<OParryShape<T, C::P<T>>>,
    pub (crate) shape_idx_to_link_idx: Vec<usize>,
    pub (crate) pair_skips: AHashMapWrapper<(u64, u64), ()>,
    subcomponent_pair_skips: AHashMapWrapper<(u64, u64), ()>,
    non_collision_states_pair_skips: AHashMapWrapper<(u64, u64), ()>,
    always_and_never_collision_pair_skips: AHashMapWrapper<(u64, u64), ()>,
    #[serde_as(as = "AHashMapWrapper<(u64, u64), T>")]
    pub (crate) pair_average_distances: AHashMapWrapper<(u64, u64), T>,
    pub (crate) id_to_string: AHashMapWrapper<u64, String>,
    pub (crate) phantom_data: PhantomData<(C, L)>
}
impl<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static> ORobotParryShapeScene<T, C, L> {
    pub fn new(robot: &ORobot<T, C, L>) -> Self {
        let mut shapes = vec![];
        let mut shape_idx_to_link_idx = vec![];
        let mut pair_skips = AHashMapWrapper::new();
        let mut id_to_string = AHashMapWrapper::new();
        
        robot.links().iter().for_each(|link| {
            if link.is_present_in_model {
                if let Some(convex_hull_file_path) = &link.convex_hull_file_path {
                    // let convex_shape_trimesh = convex_hull_file_path.load_stl().to_trimesh();
                    let mut convex_shape_subcomponents_trimesh = vec![];
                    link.convex_decomposition_file_paths.iter().for_each(|x| {
                        convex_shape_subcomponents_trimesh.push(x.clone());
                    });

                    let shape = OParryShape::new_convex_shape_from_mesh_paths(convex_hull_file_path.clone(), C::P::identity(), Some(convex_shape_subcomponents_trimesh));

                    id_to_string.hashmap.insert(shape.base_shape().base_shape().id(), format!("convex shape for link {} ({})", link.link_idx, link.name));
                    id_to_string.hashmap.insert(shape.base_shape().obb().id(), format!("obb for link {} ({})", link.link_idx, link.name));
                    id_to_string.hashmap.insert(shape.base_shape().bounding_sphere().id(), format!("bounding sphere for link {} ({})", link.link_idx, link.name));
                    shape.convex_subcomponents().iter().enumerate().for_each(|(i, x)| {
                        id_to_string.hashmap.insert(x.base_shape().id(), format!("convex shape for link {} ({}) subcomponent {}", link.link_idx, link.name, i));
                        id_to_string.hashmap.insert(x.obb().id(), format!("obb for link {} ({}) subcomponent {}", link.link_idx, link.name, i));
                        id_to_string.hashmap.insert(x.bounding_sphere().id(), format!("bounding sphere for link {} ({}) subcomponent {}", link.link_idx, link.name, i));
                    });

                    shapes.push(shape);
                    shape_idx_to_link_idx.push(link.link_idx);
                }
            }
        });

        robot.sub_robots.iter().for_each(|sub_robot| {
            sub_robot.parry_shape_scene.pair_skips.hashmap.keys().for_each(|x| {
                pair_skips.hashmap.insert(*x, ());
            });
        });

        ORobotParryShapeScene {
            shapes,
            shape_idx_to_link_idx,
            pair_skips,
            subcomponent_pair_skips: AHashMapWrapper::new(),
            non_collision_states_pair_skips: AHashMapWrapper::new(),
            always_and_never_collision_pair_skips: AHashMapWrapper::new(),
            pair_average_distances: AHashMapWrapper::new(),
            id_to_string,
            phantom_data: Default::default(),
        }
    }
    pub (crate) fn add_non_collision_states_pair_skips<V: OVec<T>>(&mut self, robot: &ORobot<T, C, L>, non_collision_states: &Vec<V>) {
        let mut h = AHashMapWrapper::new();
        for non_collision_state in non_collision_states {
            let poses = self.get_poses(&(robot, non_collision_state));
            let poses = poses.as_ref();
            let shapes = self.get_shapes();
            /*
            let out = ParryIntersectGroupQry::query(&shapes, &shapes, &poses, &poses, &ParryPairSelector::HalfPairs, &(), &(),&ParryIntersectGroupArgs::new(ParryShapeRep::Full, false));
            out.outputs().iter().for_each(|x| {
               if x.data().intersect() {
                   let idxs = x.pair_idxs();
                   match idxs {
                       ParryPairIdxs::Shapes(i, j) => {
                           let shape_a = shapes[*i].base_shape();
                           let shape_b = shapes[*j].base_shape();

                           h.hashmap.insert((shape_a.base_shape().id(), shape_b.base_shape().id()), ());
                           h.hashmap.insert((shape_b.base_shape().id(), shape_a.base_shape().id()), ());
                           h.hashmap.insert((shape_a.obb().id(), shape_b.obb().id()), ());
                           h.hashmap.insert((shape_b.obb().id(), shape_a.obb().id()), ());
                           h.hashmap.insert((shape_a.bounding_sphere().id(), shape_b.bounding_sphere().id()), ());
                           h.hashmap.insert((shape_b.bounding_sphere().id(), shape_a.bounding_sphere().id()), ());
                       }
                       ParryPairIdxs::ShapeSubcomponents(_, _) => { unreachable!() }
                   }
               }
            });
            */

            let out = ParryIntersectGroupQry::query(&shapes, &shapes, &poses, &poses, &ParryPairSelector::HalfPairsSubcomponents, &(), &(), &ParryIntersectGroupArgs::new(ParryShapeRep::Full, false));
            out.outputs().iter().for_each(|x| {
                if x.data().intersect() {
                    let idxs = x.pair_idxs();

                    match idxs {
                        ParryPairIdxs::Shapes(_, _) => { unreachable!() }
                        ParryPairIdxs::ShapeSubcomponents(i, j) => {
                            let shape_a = &shapes[i.0].convex_subcomponents()[i.1];
                            let shape_b = &shapes[j.0].convex_subcomponents()[j.1];

                            h.hashmap.insert((shape_a.base_shape().id(), shape_b.base_shape().id()), ());
                            h.hashmap.insert((shape_b.base_shape().id(), shape_a.base_shape().id()), ());
                            h.hashmap.insert((shape_a.obb().id(), shape_b.obb().id()), ());
                            h.hashmap.insert((shape_b.obb().id(), shape_a.obb().id()), ());
                            h.hashmap.insert((shape_a.bounding_sphere().id(), shape_b.bounding_sphere().id()), ());
                            h.hashmap.insert((shape_b.bounding_sphere().id(), shape_a.bounding_sphere().id()), ());
                        }
                    }
                }
            });
        }

        // h.keys().for_each(|x| { self.pair_skips.hashmap.insert(*x, ()); });
        println!("found {} new non collision state pairs to skip", h.hashmap.len());
        self.non_collision_states_pair_skips = h;
        self.combine_all_skips();
    }
    pub (crate) fn add_subcomponent_pair_skips(&mut self) {
        let mut h = AHashMapWrapper::new();

        self.shapes.iter().for_each(|shape| {
            let num_subcomponents = shape.convex_subcomponents().len();
            for i in 0..num_subcomponents {
                for j in 0..num_subcomponents {
                    let shape0 = &shape.convex_subcomponents()[i];
                    let shape1 = &shape.convex_subcomponents()[j];

                    h.hashmap.insert((shape0.base_shape().id(), shape1.base_shape().id()), ());
                    h.hashmap.insert((shape1.base_shape().id(), shape0.base_shape().id()), ());
                    h.hashmap.insert((shape0.obb().id(), shape1.obb().id()), ());
                    h.hashmap.insert((shape1.obb().id(), shape0.obb().id()), ());
                    h.hashmap.insert((shape0.bounding_sphere().id(), shape1.bounding_sphere().id()), ());
                    h.hashmap.insert((shape1.bounding_sphere().id(), shape0.bounding_sphere().id()), ());
                }
            }
        });

        // h.keys().for_each(|x| { self.pair_skips.hashmap.insert(*x, ()); });
        self.subcomponent_pair_skips = h;
        self.combine_all_skips();
    }
    /*
    pub fn add_state_sampler_always_and_never_collision_pair_skips(&mut self, robot: &ORobot<T, C, L>, num_samples: usize) {
        match robot.robot_type() {
            RobotType::RobotSet => { println!("cannot add pair skips for a robot set.  returning."); return; }
            _ => { }
        }

        let mut never_in_collision = AHashMapWrapper::new();
        let mut always_in_collision = AHashMapWrapper::new();

        let mut progress_bar = get_default_progress_bar(num_samples);

        let s = self.get_shapes();
        for k in 0..num_samples {
            progress_bar.message(&format!("always and never collision sample {} of {}", k+1, num_samples));
            progress_bar.set(k as u64);

            // oprint(&format!("always and never in collision sample {}", k), PrintMode::Println, PrintColor::Cyan);
            // println!("{:?}", always_in_collision.len());

            let state = robot.sample_pseudorandom_state();

            let p = self.get_poses(&(robot, &state));

            let out = ParryIntersectGroupQry::query(s, s, p.as_ref(), p.as_ref(), &ParryPairSelector::HalfPairs, &(), &ParryIntersectGroupArgs::new(ParryShapeRep::Full, false));
            out.outputs().iter().for_each(|x| {
                let idxs = x.pair_idxs();
                match idxs {
                    ParryPairIdxs::Shapes(i, j) => {
                        let shape_a = s[*i].base_shape();
                        let shape_b = s[*j].base_shape();

                        if x.data().intersect() {
                            if k == 0 {
                                always_in_collision.hashmap.insert((shape_a.base_shape().id(), shape_b.base_shape().id()), ());
                                always_in_collision.hashmap.insert((shape_b.base_shape().id(), shape_a.base_shape().id()), ());
                                always_in_collision.hashmap.insert((shape_a.obb().id(), shape_b.obb().id()), ());
                                always_in_collision.hashmap.insert((shape_b.obb().id(), shape_a.obb().id()), ());
                                always_in_collision.hashmap.insert((shape_a.bounding_sphere().id(), shape_b.bounding_sphere().id()), ());
                                always_in_collision.hashmap.insert((shape_b.bounding_sphere().id(), shape_a.bounding_sphere().id()), ());
                            }
                            never_in_collision.hashmap.remove_entry(&(shape_a.base_shape().id(), shape_b.base_shape().id()));
                            never_in_collision.hashmap.remove_entry(&(shape_b.base_shape().id(), shape_a.base_shape().id()));
                            never_in_collision.hashmap.remove_entry(&(shape_a.obb().id(), shape_b.obb().id()));
                            never_in_collision.hashmap.remove_entry(&(shape_b.obb().id(), shape_a.obb().id()));
                            never_in_collision.hashmap.remove_entry(&(shape_a.bounding_sphere().id(), shape_b.bounding_sphere().id()));
                            never_in_collision.hashmap.remove_entry(&(shape_b.bounding_sphere().id(), shape_a.bounding_sphere().id()));
                        } else {
                            if k == 0 {
                                never_in_collision.hashmap.insert((shape_a.base_shape().id(), shape_b.base_shape().id()), ());
                                never_in_collision.hashmap.insert((shape_b.base_shape().id(), shape_a.base_shape().id()), ());
                                never_in_collision.hashmap.insert((shape_a.obb().id(), shape_b.obb().id()), ());
                                never_in_collision.hashmap.insert((shape_b.obb().id(), shape_a.obb().id()), ());
                                never_in_collision.hashmap.insert((shape_a.bounding_sphere().id(), shape_b.bounding_sphere().id()), ());
                                never_in_collision.hashmap.insert((shape_b.bounding_sphere().id(), shape_a.bounding_sphere().id()), ());
                            }
                            always_in_collision.hashmap.remove_entry(&(shape_a.base_shape().id(), shape_b.base_shape().id()));
                            always_in_collision.hashmap.remove_entry(&(shape_b.base_shape().id(), shape_a.base_shape().id()));
                            always_in_collision.hashmap.remove_entry(&(shape_a.obb().id(), shape_b.obb().id()));
                            always_in_collision.hashmap.remove_entry(&(shape_b.obb().id(), shape_a.obb().id()));
                            always_in_collision.hashmap.remove_entry(&(shape_a.bounding_sphere().id(), shape_b.bounding_sphere().id()));
                            always_in_collision.hashmap.remove_entry(&(shape_b.bounding_sphere().id(), shape_a.bounding_sphere().id()));
                        }
                    }
                    ParryPairIdxs::ShapeSubcomponents(_, _) => { unreachable!() }
                }
            });

            let filter = ParryToSubcomponentsFilter;
            let f = filter.pair_group_filter(s, s, p.as_ref(), p.as_ref(), &ParryPairSelector::HalfPairs, &());
            let out = ParryIntersectGroupQry::query(s, s, p.as_ref(), p.as_ref(), f.selector(), &(), &ParryIntersectGroupArgs::new(ParryShapeRep::Full, false));
            out.outputs().iter().for_each(|x| {
                if x.data().intersect() {
                    let idxs = x.pair_idxs();

                    match idxs {
                        ParryPairIdxs::Shapes(_, _) => { unreachable!() }
                        ParryPairIdxs::ShapeSubcomponents(i, j) => {
                            let shape_a = &s[i.0].convex_subcomponents()[i.1];
                            let shape_b = &s[j.0].convex_subcomponents()[j.1];

                            if x.data().intersect() {
                                if k == 0 {
                                    always_in_collision.hashmap.insert((shape_a.base_shape().id(), shape_b.base_shape().id()), ());
                                    always_in_collision.hashmap.insert((shape_b.base_shape().id(), shape_a.base_shape().id()), ());
                                    always_in_collision.hashmap.insert((shape_a.obb().id(), shape_b.obb().id()), ());
                                    always_in_collision.hashmap.insert((shape_b.obb().id(), shape_a.obb().id()), ());
                                    always_in_collision.hashmap.insert((shape_a.bounding_sphere().id(), shape_b.bounding_sphere().id()), ());
                                    always_in_collision.hashmap.insert((shape_b.bounding_sphere().id(), shape_a.bounding_sphere().id()), ());
                                }
                                never_in_collision.hashmap.remove(&(shape_a.base_shape().id(), shape_b.base_shape().id()));
                                never_in_collision.hashmap.remove(&(shape_b.base_shape().id(), shape_a.base_shape().id()));
                                never_in_collision.hashmap.remove(&(shape_a.obb().id(), shape_b.obb().id()));
                                never_in_collision.hashmap.remove(&(shape_b.obb().id(), shape_a.obb().id()));
                                never_in_collision.hashmap.remove(&(shape_a.bounding_sphere().id(), shape_b.bounding_sphere().id()));
                                never_in_collision.hashmap.remove(&(shape_b.bounding_sphere().id(), shape_a.bounding_sphere().id()));
                            } else {
                                if k == 0 {
                                    never_in_collision.hashmap.insert((shape_a.base_shape().id(), shape_b.base_shape().id()), ());
                                    never_in_collision.hashmap.insert((shape_b.base_shape().id(), shape_a.base_shape().id()), ());
                                    never_in_collision.hashmap.insert((shape_a.obb().id(), shape_b.obb().id()), ());
                                    never_in_collision.hashmap.insert((shape_b.obb().id(), shape_a.obb().id()), ());
                                    never_in_collision.hashmap.insert((shape_a.bounding_sphere().id(), shape_b.bounding_sphere().id()), ());
                                    never_in_collision.hashmap.insert((shape_b.bounding_sphere().id(), shape_a.bounding_sphere().id()), ());
                                }
                                always_in_collision.hashmap.remove(&(shape_a.base_shape().id(), shape_b.base_shape().id()));
                                always_in_collision.hashmap.remove(&(shape_b.base_shape().id(), shape_a.base_shape().id()));
                                always_in_collision.hashmap.remove(&(shape_a.obb().id(), shape_b.obb().id()));
                                always_in_collision.hashmap.remove(&(shape_b.obb().id(), shape_a.obb().id()));
                                always_in_collision.hashmap.remove(&(shape_a.bounding_sphere().id(), shape_b.bounding_sphere().id()));
                                always_in_collision.hashmap.remove(&(shape_b.bounding_sphere().id(), shape_a.bounding_sphere().id()));
                            }
                        }
                    }
                }
            });
        }

        progress_bar.finish();
        println!();
        oprint(&format!("found {} pairs always in collision", always_in_collision.hashmap.len()), PrintMode::Println, PrintColor::Cyan);
        oprint(&format!("found {} pairs never in collision", never_in_collision.hashmap.len()), PrintMode::Println, PrintColor::Cyan);

        never_in_collision.hashmap.keys().for_each(|x| { self.pair_skips.hashmap.insert(*x, ()); });
        always_in_collision.hashmap.keys().for_each(|x| { self.pair_skips.hashmap.insert(*x, ()); });
    }
    */
    pub (crate) fn add_state_sampler_always_and_never_collision_pair_skips(&mut self, robot: &ORobot<T, C, L>, stasis_point_cutoff: usize) {
        oprint(&format!("computing always and never collision pair skips.  This may take a while, and it's normal for the bar to keep starting over..."), PrintMode::Println, PrintColor::Cyan);

        let mut h = AHashMapWrapper::new();

        let s = self.get_shapes();
        let mut npi = get_all_parry_pairs_idxs(s, s, true, false);
        let mut api = get_all_parry_pairs_idxs(s, s, true, false);
        npi.sort();
        api.sort();

        let mut npsi = get_all_parry_pairs_idxs(s, s, true, true);
        let mut apsi = get_all_parry_pairs_idxs(s, s, true, true);
        npsi.sort();
        apsi.sort();

        let mut progress_bar = get_default_progress_bar(stasis_point_cutoff);

        let mut consecutive_no_change_count = 0;
        'l: loop {
            progress_bar.message(&format!("always and never collision {} of {}", consecutive_no_change_count, stasis_point_cutoff));
            progress_bar.set(consecutive_no_change_count as u64);

            let mut change = false;
            let state = robot.sample_pseudorandom_state();
            let p = self.get_poses(&(robot, &state));
            let npi_res = ParryIntersectGroupQry::query(s, s, p.as_ref(), p.as_ref(), &ParryPairSelector::PairsByIdxs(npi.clone()), &self.pair_skips, &self.pair_average_distances, &ParryIntersectGroupArgs::new(ParryShapeRep::Full, false));
            let api_res = ParryIntersectGroupQry::query(s, s, p.as_ref(), p.as_ref(), &ParryPairSelector::PairsByIdxs(api.clone()), &self.pair_skips, &self.pair_average_distances, &ParryIntersectGroupArgs::new(ParryShapeRep::Full, false));
            let npsi_res = ParryIntersectGroupQry::query(s, s, p.as_ref(), p.as_ref(), &ParryPairSelector::PairsByIdxs(npsi.clone()), &self.pair_skips, &self.pair_average_distances, &ParryIntersectGroupArgs::new(ParryShapeRep::Full, false));
            let apsi_res = ParryIntersectGroupQry::query(s, s, p.as_ref(), p.as_ref(), &ParryPairSelector::PairsByIdxs(apsi.clone()), &self.pair_skips, &self.pair_average_distances, &ParryIntersectGroupArgs::new(ParryShapeRep::Full, false));

            npi_res.outputs().iter().for_each(|x| {
               if x.data().intersect() { change = true; let idxs = x.pair_idxs(); npi = npi.iter().filter(|y| **y != *idxs ).map(|y| y.clone()).collect(); }
            });
            npsi_res.outputs().iter().for_each(|x| {
                if x.data().intersect() { change = true; let idxs = x.pair_idxs(); npsi = npsi.iter().filter(|y| **y != *idxs ).map(|y| y.clone()).collect(); }
            });

            api_res.outputs().iter().for_each(|x| {
                if !x.data().intersect() { change = true; let idxs = x.pair_idxs(); api = api.iter().filter(|y| **y != *idxs ).map(|y| y.clone()).collect(); }
            });
            apsi_res.outputs().iter().for_each(|x| {
                if !x.data().intersect() { change = true; let idxs = x.pair_idxs(); apsi = apsi.iter().filter(|y| **y != *idxs ).map(|y| y.clone()).collect(); }
            });

            if !change { consecutive_no_change_count += 1; } else { consecutive_no_change_count = 0; }
            if consecutive_no_change_count > stasis_point_cutoff { break 'l; }
        }

        let lists = vec![&npi, &api, &npsi, &apsi];
        let shape_reps = vec![ParryShapeRep::Full, ParryShapeRep::OBB, ParryShapeRep::BoundingSphere];

        for l in lists {
            for pair_idxs in l {
                let shapes = match pair_idxs {
                    ParryPairIdxs::Shapes(i, j) => {
                        let shape_a = s[*i].base_shape();
                        let shape_b = s[*j].base_shape();

                        (shape_a, shape_b)
                    }
                    ParryPairIdxs::ShapeSubcomponents(i, j) => {
                        let shape_a = &s[i.0].convex_subcomponents()[i.1];
                        let shape_b = &s[j.0].convex_subcomponents()[j.1];

                        (shape_a, shape_b)
                    }
                };

                for shape_rep in &shape_reps {
                    let id0 = shapes.0.id_from_shape_rep(shape_rep);
                    let id1 = shapes.1.id_from_shape_rep(shape_rep);

                    h.hashmap.insert((id0, id1), ());
                    h.hashmap.insert((id1, id0), ());
                }
            }
        }

        progress_bar.finish();
        println!();
        oprint(&format!("found {} pairs always in collision", api.len() + apsi.len()), PrintMode::Println, PrintColor::Cyan);
        oprint(&format!("found {} pairs never in collision", npi.len() + npsi.len()), PrintMode::Println, PrintColor::Cyan);

        // h.hashmap.keys().for_each(|x| { self.pair_skips.hashmap.insert(*x, ()); });
        self.always_and_never_collision_pair_skips = h;
        self.combine_all_skips();
    }
    /*
    pub fn compute_shape_average_distances(&mut self, robot: &ORobot<T, C, L>, num_samples: usize) {
        oprint(&format!("computing shape average distances..."), PrintMode::Println, PrintColor::Cyan);

        let mut h = AHashMapWrapper::new();

        let s = self.get_shapes();

        let mut progress_bar = get_default_progress_bar(num_samples);

        let pair_selectors = vec![ParryPairSelector::HalfPairs, ParryPairSelector::HalfPairsSubcomponents];
        let shape_reps = vec![ParryShapeRep::Full, ParryShapeRep::OBB, ParryShapeRep::BoundingSphere];

        for i in 0..num_samples {
            let state = robot.sample_pseudorandom_state();
            let p = self.get_poses(&(robot, &state));
            progress_bar.message(&format!("average distance sample {} of {}", i, num_samples));
            progress_bar.set(i as u64);
            pair_selectors.iter().for_each(|pair_selector| {
                shape_reps.iter().for_each(|shape_rep| {
                    let res = ParryDistanceGroupQry::query(s, s, p.as_ref(), p.as_ref(), &pair_selector, &(), &(), &ParryDistanceGroupArgs::new(shape_rep.clone(), ParryDisMode::ContactDis, T::constant(-1000000000.0)));
                    res.outputs().iter().for_each(|output| {
                        let ids = output.pair_ids();
                        let id0 = ids.0;
                        let id1 = ids.1;
                        let dis = output.data().distance();

                        let g = h.hashmap.get_mut(&(id0, id1));
                        match g {
                            None => { h.hashmap.insert((id0, id1), dis); }
                            Some(d) => { *d += dis; }
                        }

                        let g = h.hashmap.get_mut(&(id1, id0));
                        match g {
                            None => { h.hashmap.insert((id1, id0), dis); }
                            Some(d) => { *d += dis; }
                        }
                    });
                });
            });
        }

        let n = T::constant(num_samples as f64);
        h.hashmap.values_mut().for_each(|v| *v /= n );
        h.hashmap.values_mut().for_each(|v| {
            if *v <= T::zero() { *v = T::constant(0.1); }
            else if *v > T::one() { *v = T::one(); }
        });

        progress_bar.finish();
        println!();
        oprint(&format!("done computing shape average distances."), PrintMode::Println, PrintColor::Cyan);
        self.pair_average_distances = h;
    }
    */
    pub (crate) fn compute_shape_average_distances(&mut self, robot: &ORobot<T, C, L>, num_samples: usize) {
        oprint(&format!("computing shape average distances..."), PrintMode::Println, PrintColor::Cyan);

        let mut h = AHashMapWrapper::new();

        let s = self.get_shapes();

        let mut progress_bar = get_default_progress_bar(num_samples);

        let pair_selectors = vec![ParryPairSelector::HalfPairs, ParryPairSelector::HalfPairsSubcomponents];

        for i in 0..num_samples {
            let state = robot.sample_pseudorandom_state();
            let p = self.get_poses(&(robot, &state));
            progress_bar.message(&format!("average distance sample {} of {}", i, num_samples));
            progress_bar.set(i as u64);
            pair_selectors.iter().for_each(|pair_selector| {
                let res = ParryDistanceGroupQry::query(s, s, p.as_ref(), p.as_ref(), pair_selector, &(), &(), &ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, false, T::constant(f64::MIN)));
                res.outputs().iter().for_each(|output| {
                    let idxs = output.pair_idxs();
                    let all_ids = match idxs {
                        ParryPairIdxs::Shapes(i, j) => {
                            let shape0 = self.shapes[*i].base_shape();
                            let shape1 = self.shapes[*j].base_shape();

                            let ids1 = (shape0.base_shape().id(), shape1.base_shape().id());
                            let ids2 = (shape0.obb().id(), shape1.obb().id());
                            let ids3 = (shape0.bounding_sphere().id(), shape1.bounding_sphere().id());

                            vec![ids1, ids2, ids3]
                        }
                        ParryPairIdxs::ShapeSubcomponents(i, j) => {
                            let shape0 = &self.shapes[i.0].convex_subcomponents()[i.1];
                            let shape1 = &self.shapes[j.0].convex_subcomponents()[j.1];

                            let ids1 = (shape0.base_shape().id(), shape1.base_shape().id());
                            let ids2 = (shape0.obb().id(), shape1.obb().id());
                            let ids3 = (shape0.bounding_sphere().id(), shape1.bounding_sphere().id());

                            vec![ids1, ids2, ids3]
                        }
                    };
                    for ids in all_ids {
                        // let ids = output.pair_ids();
                        let id0 = ids.0;
                        let id1 = ids.1;
                        let dis = output.data().distance();

                        let g = h.hashmap.get_mut(&(id0, id1));
                        match g {
                            None => { h.hashmap.insert((id0, id1), dis); }
                            Some(d) => { *d += dis; }
                        }

                        let g = h.hashmap.get_mut(&(id1, id0));
                        match g {
                            None => { h.hashmap.insert((id1, id0), dis); }
                            Some(d) => { *d += dis; }
                        }
                    }
                });
            });
        }

        let n = T::constant(num_samples as f64);
        h.hashmap.values_mut().for_each(|v| *v /= n );
        h.hashmap.values_mut().for_each(|v| {
            if *v <= T::constant(0.1) { *v = T::constant(0.1); }
            else if *v > T::one() { *v = T::one(); }
        });

        progress_bar.finish();
        println!();
        oprint(&format!("done computing shape average distances."), PrintMode::Println, PrintColor::Cyan);
        self.pair_average_distances = h;
    }
    #[inline(always)]
    pub fn get_pair_average_distances(&self) -> &AHashMapWrapper<(u64, u64), T> {
        &self.pair_average_distances
    }
    /*
    pub (crate) fn compute_shape_average_distances(&mut self, robot: &ORobot<T, C, L>, num_samples: usize) {
        let mut shape_average_distances = AHashMapWrapper::new();

        let s = self.get_shapes();

        let f = ParryToSubcomponentsFilter;
        let ff = f.pair_group_filter(s, s, &(), &(), &ParryPairSelector::HalfPairs, &());

        for _i in 0..num_samples {
            let state = robot.sample_pseudorandom_state();
            let p = self.get_poses(&(robot, &state));

            let res = ParryDistanceGroupQry::query(s, s, p.as_ref(), p.as_ref(),&ParryPairSelector::HalfPairs, &(), &ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, T::constant(-f64::MIN)));
            res.outputs().iter().for_each(|x| {
                let dis = x.data().distance();

                let idxs = x.pair_idxs();
                match idxs {
                    ParryPairIdxs::Shapes(i, j) => {
                        let shape_a = s[*i].base_shape();
                        let shape_b = s[*j].base_shape();

                        let a = shape_average_distances.hashmap.get_mut( &(shape_a.base_shape().id(), shape_b.base_shape().id()));
                    }
                    ParryPairIdxs::ShapeSubcomponents(_, _) => { unreachable!() }
                }
            })

        }

        self.shape_average_distances = shape_average_distances;
    }
    */
    pub (crate) fn new_default() -> Self {
        Self {
            shapes: vec![],
            shape_idx_to_link_idx: vec![],
            pair_skips: AHashMapWrapper::new(),
            subcomponent_pair_skips: AHashMapWrapper::new(),
            non_collision_states_pair_skips: AHashMapWrapper::new(),
            always_and_never_collision_pair_skips: AHashMapWrapper::new(),
            pair_average_distances: AHashMapWrapper::new(),
            id_to_string: AHashMapWrapper::new(),
            phantom_data: Default::default(),
        }
    }
    pub (crate) fn resample_ids(&mut self) {
        let mut h = AHashMapWrapper::new();

        self.shapes.iter_mut().for_each(|x| {
            let changes = x.resample_all_ids();
            for change in changes { h.hashmap.insert(change.0, change.1); }
        });

        let mut new_pair_skips = AHashMapWrapper::new();
        self.pair_skips.hashmap.keys().for_each(|(x, y)| {
            let x_update = h.hashmap.get(x).expect("error");
            let y_update = h.hashmap.get(y).expect("error");

            new_pair_skips.hashmap.insert((*x_update, *y_update), ());
        });

        self.pair_skips = new_pair_skips;

        let mut new_pair_average_distances = AHashMapWrapper::new();
        self.pair_average_distances.hashmap.iter().for_each(|((x,y), z)| {
            let x_update = h.hashmap.get(x).expect("error");
            let y_update = h.hashmap.get(y).expect("error");

            new_pair_average_distances.hashmap.insert((*x_update, *y_update), *z);
        });

        self.pair_average_distances = new_pair_average_distances;

        let mut new_id_to_string = AHashMapWrapper::new();
        self.id_to_string.hashmap.iter().for_each(|(x, y)| {
            let update = h.hashmap.get(x).expect("error");

            new_id_to_string.hashmap.insert(*update, y.clone());
        });

        self.id_to_string = new_id_to_string;
    }
    pub (crate) fn combine_all_skips(&mut self) {
        let mut pair_skips = AHashMapWrapper::new();

        let hs = vec![&self.subcomponent_pair_skips, &self.non_collision_states_pair_skips, &self.always_and_never_collision_pair_skips];
        for h in hs {
            h.hashmap.keys().for_each(|k| { pair_skips.hashmap.insert(k.clone(), ()); });
        }

        self.pair_skips = pair_skips;
    }
}
impl<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static> ShapeSceneTrait<T, C::P<T>> for ORobotParryShapeScene<T, C, L> {
    type ShapeType = OParryShape<T, C::P<T>>;
    type GetPosesInput<'a, V: OVec<T>> = (&'a ORobot<T, C, L>, &'a V);
    type PairSkipsType = AHashMapWrapper<(u64, u64), ()>;

    #[inline(always)]
    fn get_shapes(&self) -> &Vec<Self::ShapeType> {
        &self.shapes
    }

    #[inline(always)]
    fn get_poses<'a, V: OVec<T>>(&self, input: &Self::GetPosesInput<'a, V>) -> Cow<'a, Vec<C::P<T>>> {
        input.0.get_shape_poses(input.1)
    }

    #[inline(always)]
    fn get_pair_skips(&self) -> &Self::PairSkipsType {
        &self.pair_skips
    }

    #[inline(always)]
    fn shape_id_to_shape_str(&self, id: u64) -> String {
        let res = self.id_to_string.hashmap.get(&id);
        return res.expect("not found").clone()
    }
}
*/

#[serde_as]
#[derive(Clone, Serialize, Deserialize)]
pub struct ORobotParryShapeScene<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory> {
    #[serde(deserialize_with="Vec::<OParryShape::<T, C::P<T>>>::deserialize")]
    pub (crate) shapes: Vec<OParryShape<T, C::P<T>>>,
    pub (crate) shape_idx_to_link_idx: Vec<usize>,
    pub pair_skips: AHashMapWrapper<(u64, u64), Vec<SkipReason>>,
    #[serde_as(as = "AHashMapWrapper<(u64, u64), T>")]
    pub pair_average_distances: AHashMapWrapper<(u64, u64), T>,
    pub (crate) id_to_string: AHashMapWrapper<u64, String>,
    phantom_data: PhantomData<(C, L)>
}
impl<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static> ORobotParryShapeScene<T, C, L> {
    pub fn new(robot: &ORobot<T, C, L>) -> Self {
        let mut shapes = vec![];
        let mut shape_idx_to_link_idx = vec![];
        let mut id_to_string = AHashMapWrapper::new();

        robot.links().iter().for_each(|link| {
            if link.is_present_in_model {
                if let Some(convex_hull_file_path) = &link.convex_hull_file_path {
                    let mut convex_shape_subcomponents_trimesh = vec![];
                    link.convex_decomposition_file_paths.iter().for_each(|x| {
                        convex_shape_subcomponents_trimesh.push(x.clone());
                    });

                    let shape = OParryShape::new_default_convex_shape_from_mesh_paths(convex_hull_file_path.clone(), C::P::identity(), Some(convex_shape_subcomponents_trimesh));

                    id_to_string.hashmap.insert(shape.base_shape().base_shape().id(), format!("convex shape for link {} ({})", link.link_idx, link.name));
                    id_to_string.hashmap.insert(shape.base_shape().obb().id(), format!("obb for link {} ({})", link.link_idx, link.name));
                    id_to_string.hashmap.insert(shape.base_shape().bounding_sphere().id(), format!("bounding sphere for link {} ({})", link.link_idx, link.name));
                    shape.convex_subcomponents().iter().enumerate().for_each(|(i, x)| {
                        id_to_string.hashmap.insert(x.base_shape().id(), format!("convex shape for link {} ({}) subcomponent {}", link.link_idx, link.name, i));
                        id_to_string.hashmap.insert(x.obb().id(), format!("obb for link {} ({}) subcomponent {}", link.link_idx, link.name, i));
                        id_to_string.hashmap.insert(x.bounding_sphere().id(), format!("bounding sphere for link {} ({}) subcomponent {}", link.link_idx, link.name, i));
                    });

                    shapes.push(shape);
                    shape_idx_to_link_idx.push(link.link_idx);
                }
            }
        });

        ORobotParryShapeScene {
            shapes,
            shape_idx_to_link_idx,
            pair_skips: AHashMapWrapper::new(),
            pair_average_distances: AHashMapWrapper::new(),
            id_to_string,
            phantom_data: Default::default(),
        }
    }
    pub fn preprocess_non_collision_states_pair_skips<V: OVec<T>>(&mut self, robot: &ORobot<T, C, L>, non_collision_states: &Vec<V>) {
        self.pair_skips.clear_skip_reason_type(SkipReason::FromNonCollisionExample);

        let shape_reps = vec![ ParryShapeRep::BoundingSphere, ParryShapeRep::OBB, ParryShapeRep::Full ];
        let selectors = vec![ParryPairSelector::HalfPairs, ParryPairSelector::HalfPairsSubcomponents];

        let shapes = &self.shapes;
        for state in non_collision_states {
            let poses = self.get_shape_poses(&(robot, state));
            let poses = poses.as_ref().clone();

            for shape_rep in &shape_reps {
                for selector in &selectors {
                    let out = ParryIntersectGroupQry::query(&shapes, &shapes, &poses, &poses, selector, &(), &(), false, &ParryIntersectGroupArgs::new(shape_rep.clone(), false, false));
                    out.outputs().iter().for_each(|x| {
                       if x.data().intersect() {
                           let (id_a, id_b) = x.pair_ids();
                           self.pair_skips.add_skip_reason(id_a, id_b, SkipReason::FromNonCollisionExample);
                           self.pair_skips.add_skip_reason(id_b, id_a, SkipReason::FromNonCollisionExample);
                       }
                    });
                }
            }
        }
    }
    pub fn add_close_proximity_states_pair_skips<V: OVec<T>>(&mut self, robot: &ORobot<T, C, L>, close_proximity_state: V, threshold: T) {
        // self.pair_skips.clear_skip_reason_type(SkipReason::CloseProximityWrtAverageExample);

        let shape_reps = vec![ ParryShapeRep::BoundingSphere, ParryShapeRep::OBB, ParryShapeRep::Full ];
        let selectors = vec![ParryPairSelector::HalfPairs, ParryPairSelector::HalfPairsSubcomponents];

        let shapes = &self.shapes;
        let poses = self.get_shape_poses(&(robot, &close_proximity_state));
        let poses = poses.as_ref().clone();

        for shape_rep in &shape_reps {
            for selector in &selectors {
                // let out = ParryIntersectGroupQry::query(&shapes, &shapes, &poses, &poses, selector, &(), &(), false, &ParryIntersectGroupArgs::new(shape_rep.clone(), false, false));
                let out = ParryDistanceGroupQry::query(&shapes, &shapes, &poses, &poses, selector, &(), &self.pair_average_distances, false, &ParryDistanceGroupArgs::new(shape_rep.clone(), ParryDisMode::ContactDis, true, false, T::constant(-1000.0), false));
                out.outputs().iter().for_each(|x| {
                    if x.data().distance() < threshold {
                        let (id_a, id_b) = x.pair_ids();
                        self.pair_skips.add_skip_reason(id_a, id_b, SkipReason::CloseProximityWrtAverageExample);
                        self.pair_skips.add_skip_reason(id_b, id_a, SkipReason::CloseProximityWrtAverageExample);
                    }
                });
            }
        }
    }
    pub fn clear_close_proximity_states_pair_skips(&mut self) {
        self.pair_skips.clear_skip_reason_type(SkipReason::CloseProximityWrtAverageExample);
    }
    pub fn preprocess_always_in_collision_states_pair_skips(&mut self, robot: &ORobot<T, C, L>, num_same: usize) {
        self.pair_skips.clear_skip_reason_type(SkipReason::AlwaysInCollision);

        let shape_reps = vec![ ParryShapeRep::BoundingSphere, ParryShapeRep::OBB, ParryShapeRep::Full ];
        let selectors = vec![ParryPairSelector::HalfPairs, ParryPairSelector::HalfPairsSubcomponents];

        let shapes = &self.shapes;

        for shape_rep in &shape_reps {
            for selector in &selectors {
                let mut progress_bar = get_default_progress_bar(num_same);

                let mut h = AHashMapWrapper::new();

                let mut count = 0;
                let mut first_loop = true;
                'l: loop {
                    progress_bar.message(&format!("shape rep {:?}, selector {:?}: always collision {} of {}", shape_rep, selector, count, num_same));
                    progress_bar.set(count as u64);

                    let sample = robot.sample_pseudorandom_state();
                    let poses = self.get_shape_poses(&(robot, &sample));
                    let poses = poses.as_ref();

                    let out = ParryIntersectGroupQry::query(&shapes, &shapes, &poses, &poses, selector, &(), &(), false, &ParryIntersectGroupArgs::new(shape_rep.clone(), false, false));
                    out.outputs().iter().for_each(|x| {
                        let ids = x.pair_ids();
                        if x.data().intersect() && first_loop {
                            h.hashmap.insert((ids.0, ids.1), ());
                            h.hashmap.insert((ids.1, ids.0), ());
                        } else if !x.data().intersect() {
                            let res0 = h.hashmap.remove(&(ids.0, ids.1));
                            let res1 = h.hashmap.remove(&(ids.1, ids.0));
                            if res0.is_some() && res1.is_some() { count = 0; }
                        }
                    });
                    first_loop = false;
                    if count > num_same { progress_bar.finish(); println!(); break 'l; }
                    else { count += 1; }
                }

                h.hashmap.keys().for_each(|(x, y)| {
                    self.pair_skips.add_skip_reason(*x, *y, SkipReason::AlwaysInCollision);
                });
            }
        }
    }
    pub fn preprocess_never_in_collision_states_pair_skips(&mut self, robot: &ORobot<T, C, L>, num_same: usize) {
        self.pair_skips.clear_skip_reason_type(SkipReason::NeverInCollision);

        let shape_reps = vec![ ParryShapeRep::BoundingSphere, ParryShapeRep::OBB, ParryShapeRep::Full ];
        let selectors = vec![ParryPairSelector::HalfPairs, ParryPairSelector::HalfPairsSubcomponents];

        let shapes = &self.shapes;

        for shape_rep in &shape_reps {
            for selector in &selectors {
                let mut progress_bar = get_default_progress_bar(num_same);

                let mut h = AHashMapWrapper::new();

                let mut count = 0;
                let mut first_loop = true;
                'l: loop {
                    progress_bar.message(&format!("shape rep {:?}, selector {:?}: never collision {} of {}", shape_rep, selector, count, num_same));
                    progress_bar.set(count as u64);

                    let sample = robot.sample_pseudorandom_state();
                    let poses = self.get_shape_poses(&(robot, &sample));
                    let poses = poses.as_ref();

                    let out = ParryIntersectGroupQry::query(&shapes, &shapes, &poses, &poses, selector, &(), &(), false, &ParryIntersectGroupArgs::new(shape_rep.clone(), false, false));
                    out.outputs().iter().for_each(|x| {
                        let ids = x.pair_ids();
                        if !x.data().intersect() && first_loop {
                            h.hashmap.insert((ids.0, ids.1), ());
                            h.hashmap.insert((ids.1, ids.0), ());
                        } else if x.data().intersect() {
                            let res0 = h.hashmap.remove(&(ids.0, ids.1));
                            let res1 = h.hashmap.remove(&(ids.1, ids.0));
                            if res0.is_some() && res1.is_some() { count = 0; }
                        }
                    });
                    first_loop = false;
                    if count > num_same { progress_bar.finish(); println!(); break 'l; }
                    else { count += 1; }
                }

                h.hashmap.keys().for_each(|(x, y)| {
                    self.pair_skips.add_skip_reason(*x, *y, SkipReason::NeverInCollision);
                });
            }
        }
    }
    pub fn preprocess_shape_average_distances(&mut self, robot: &ORobot<T, C, L>, num_samples: usize) {
        self.pair_average_distances.hashmap.clear();

        let shape_reps = vec![ ParryShapeRep::Full ];
        let selectors = vec![ParryPairSelector::HalfPairs, ParryPairSelector::HalfPairsSubcomponents];

        let shapes = &self.shapes;

        for shape_rep in &shape_reps {
            for selector in &selectors {
                let mut progress_bar = get_default_progress_bar(num_samples);
                for i in 0..num_samples {
                    let state = robot.sample_pseudorandom_state();
                    let poses = self.get_shape_poses(&(robot, &state));
                    let poses = poses.as_ref();
                    progress_bar.message(&format!("shape rep {:?}, selector {:?}: average distance sample {} of {}", shape_rep, selector, i, num_samples));
                    progress_bar.set(i as u64);

                    let res = ParryDistanceGroupQry::query(shapes, shapes, poses, poses, selector, &(), &(), false, &ParryDistanceGroupArgs::new(shape_rep.clone(), ParryDisMode::ContactDis, false, false, T::constant(f64::MIN), false));
                    res.outputs().iter().for_each(|output| {
                        let ids = output.pair_ids();

                        let pair_shape_idxs = output.pair_idxs();
                        let id_pairs = match &pair_shape_idxs {
                            ParryPairIdxs::Shapes(x, y) => {
                                let bounding_sphere_a = shapes[*x].base_shape().bounding_sphere();
                                let bounding_sphere_b = shapes[*y].base_shape().bounding_sphere();

                                let obb_a = shapes[*x].base_shape().obb();
                                let obb_b = shapes[*y].base_shape().obb();

                                [(bounding_sphere_a.id(), bounding_sphere_b.id()), (obb_a.id(), obb_b.id())]
                            }
                            ParryPairIdxs::ShapeSubcomponents(x, y) => {
                                let bounding_sphere_a = shapes[x.0].convex_subcomponents()[x.1].bounding_sphere();
                                let bounding_sphere_b = shapes[y.0].convex_subcomponents()[y.1].bounding_sphere();

                                let obb_a = shapes[x.0].convex_subcomponents()[x.1].obb();
                                let obb_b = shapes[y.0].convex_subcomponents()[y.1].obb();

                                [(bounding_sphere_a.id(), bounding_sphere_b.id()), (obb_a.id(), obb_b.id())]
                            }
                        };

                        let all_ids = [ ids, id_pairs[0], id_pairs[1] ];
                        for ids in all_ids {
                            let a = self.pair_average_distances.hashmap.get_mut(&(ids.0, ids.1));
                            match a {
                                None => { self.pair_average_distances.hashmap.insert((ids.0, ids.1), output.data().distance()); }
                                Some(a) => { *a += output.data().distance(); }
                            }
                            let a = self.pair_average_distances.hashmap.get_mut(&(ids.1, ids.0));
                            match a {
                                None => { self.pair_average_distances.hashmap.insert((ids.1, ids.0), output.data().distance()); }
                                Some(a) => { *a += output.data().distance(); }
                            }
                        }

                    });
                }
                progress_bar.finish();
                println!();
            }
        }

        self.pair_average_distances.hashmap.values_mut().for_each(|x| *x /= T::constant(num_samples as f64));

        self.pair_average_distances.hashmap.values_mut().for_each(|x| {
            if *x < T::constant(0.1) { *x = T::constant(0.1); }
            if *x > T::constant(1.0) { *x = T::constant(1.0); }
        });
    }
    #[inline(always)]
    pub fn get_pair_average_distances(&self) -> &AHashMapWrapper<(u64, u64), T> {
        &self.pair_average_distances
    }
    pub (crate) fn resample_ids(&mut self) {
        let mut h = AHashMapWrapper::new();

        self.shapes.iter_mut().for_each(|x| {
            let changes = x.resample_all_ids();
            for change in changes { h.hashmap.insert(change.0, change.1); }
        });

        let mut new_pair_skips = AHashMapWrapper::new();
        self.pair_skips.hashmap.iter().for_each(|((x, y), z)| {
            let x_update = h.hashmap.get(x).expect("error");
            let y_update = h.hashmap.get(y).expect("error");

            new_pair_skips.hashmap.insert((*x_update, *y_update), z.clone());
        });

        self.pair_skips = new_pair_skips;

        let mut new_pair_average_distances = AHashMapWrapper::new();
        self.pair_average_distances.hashmap.iter().for_each(|((x,y), z)| {
            let x_update = h.hashmap.get(x).expect("error");
            let y_update = h.hashmap.get(y).expect("error");

            new_pair_average_distances.hashmap.insert((*x_update, *y_update), *z);
        });

        self.pair_average_distances = new_pair_average_distances;

        let mut new_id_to_string = AHashMapWrapper::new();
        self.id_to_string.hashmap.iter().for_each(|(x, y)| {
            let update = h.hashmap.get(x).expect("error");

            new_id_to_string.hashmap.insert(*update, y.clone());
        });

        self.id_to_string = new_id_to_string;
    }
    pub (crate) fn new_default() -> Self {
        Self {
            shapes: vec![],
            shape_idx_to_link_idx: vec![],
            pair_skips: AHashMapWrapper::new(),
            pair_average_distances: AHashMapWrapper::new(),
            id_to_string: AHashMapWrapper::new(),
            phantom_data: Default::default(),
        }
    }
}
impl<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static> ShapeSceneTrait<T, C::P<T>> for ORobotParryShapeScene<T, C, L> {
    type ShapeType = OParryShape<T, C::P<T>>;
    type GetPosesInput<'a, V: OVec<T>> = (&'a ORobot<T, C, L>, &'a V);
    type PairSkipsType = AHashMapWrapper<(u64, u64), Vec<SkipReason>>;

    #[inline(always)]
    fn get_shapes(&self) -> &Vec<Self::ShapeType> {
        &self.shapes
    }

    #[inline(always)]
    fn get_shape_poses<'a, V: OVec<T>>(&'a self, input: &Self::GetPosesInput<'a, V>) -> Cow<Vec<C::P<T>>> {
        input.0.get_shape_poses_internal(input.1)
    }

    #[inline(always)]
    fn get_pair_skips(&self) -> &Self::PairSkipsType {
        &self.pair_skips
    }

    #[inline(always)]
    fn shape_id_to_shape_str(&self, id: u64) -> String {
        let res = self.id_to_string.hashmap.get(&id);
        return res.expect("not found").clone()
    }
}

