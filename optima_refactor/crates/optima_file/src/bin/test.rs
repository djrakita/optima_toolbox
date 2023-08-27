use dae_parser::{Geometry, Primitive, Semantic, Source, Transform, Vertices};
use optima_file::path::{OAssetLocation, OStemCellPath};

fn main() {
    let mut p = OStemCellPath::new_asset_path();
    p.append_file_location(&OAssetLocation::ChainOriginalMeshes { chain_name: "ur5" });
    p.append("base.dae");
    let d = p.load_dae();

    let vs = d.get_visual_scene().unwrap();
    let url = vs.nodes[0].instance_geometry[0].url.clone();
    let g = d.local_map::<Geometry>().unwrap().get_raw(&url).unwrap();
    // println!("{:?}", g.element.as_mesh().unwrap().elements[0].as_triangles());

    let vertices_map = d.local_map::<Vertices>().unwrap();
    let sources_map = d.local_map::<Source>().unwrap();

    let mesh = g.element.as_mesh().unwrap();
    mesh.elements.iter().for_each(|x| {
        match x {
            Primitive::Lines(_) => {
                println!("lines");
            }
            Primitive::LineStrips(_) => {
                println!("line strips");
            }
            Primitive::Polygons(_) => {
                println!("polygons");
            }
            Primitive::PolyList(_) => { }
            Primitive::Triangles(t) => {
                t.inputs.iter().for_each(|y| {
                    match &y.semantic {
                        Semantic::Normal => {
                            let url = y.input.source.clone();
                            let source = sources_map.get_raw(&url).unwrap();
                            println!("{:?}", source.array);
                        }
                        Semantic::Vertex => {
                            let url = y.source.clone();
                            let vertices = vertices_map.get_raw(&url).unwrap();
                            // let vertices = vertices_map.get_raw(&url).unwrap();
                            let url = vertices.position_input().source.clone();
                            let positions = sources_map.get_raw(&url).unwrap();
                            println!("{:?}", positions.array.unwrap());
                        }
                        _ => { println!(" >>> {:?}", y.semantic); }
                    }
                });
                println!("{:?}", t.data.prim.as_ref().unwrap());
            }
            Primitive::TriFans(_) => {
                println!("tri_fans");
            }
            Primitive::TriStrips(_) => {
                println!("tri_strips");
            }
        }
        println!("---");
    });

    match &vs.nodes[0].transforms[0] {
        Transform::LookAt(_) => {}
        Transform::Matrix(m) => {
            let mat = &m.0;
            println!("{:?}", mat);
        }
        Transform::Rotate(_) => {}
        Transform::Scale(_) => {}
        Transform::Skew(_) => {}
        Transform::Translate(_) => {}
    }
}