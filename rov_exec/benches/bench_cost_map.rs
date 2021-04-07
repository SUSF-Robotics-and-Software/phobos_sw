//! # Cost Map Benchmark

use criterion::{criterion_group, criterion_main, Criterion};

use comms_if::tc::auto::PathSpec;
use rov_lib::auto::{
    map::{TerrainMap, Point2, CostMap, CostMapParams, TerrainMapLayer},
    path::Path,
    loc::Pose
};
use nalgebra::{Vector3, UnitQuaternion};

fn cost_map_benchmark(c: &mut Criterion) {
    // ---- Build dummy terrain and cost map ----

    // CostMapParams
    let cost_map_params = CostMapParams {
        max_safe_gradient: 0.4668,
        cost_onset_semi_width_m: 0.2,
        max_cost_semi_width_m: 1.0,
        max_added_cost: 1.0,
    };

    
    // Generate a random terrain map
    let terrain_map = TerrainMap::generate_random(
        Point2::new(0.1, 0.1), 
        Point2::new(100, 100), 
        Point2::new(5.0, 5.0),
        Point2::new(0.1, 0.1),
        Point2::new(0.0, 0.0)
    ).unwrap();
    
    // Ground planned path to test
    let ground_planned_path = Path::from_path_spec(
        PathSpec::AckSeq {
            separation_m: 0.05,
            seq: vec![0.5, 3.1415, 0.0, 4.0, -0.5, 3.1415, 0.0, 2.0, -0.5, 3.1415, 0.0, 6.0],
        },
        &Pose {
            position_m_lm: Vector3::new(
                1.0, 1.0, 
                terrain_map.get_position(
                    TerrainMapLayer::Height, 
                    &(1.0, 1.0).into()
                ).unwrap().unwrap()
            ),
            attitude_q_lm: UnitQuaternion::identity(),
        }
    ).unwrap();

    // Bench normal calc function
    c.bench_function(
        "CostMap::calculate", 
        |b| b.iter(|| CostMap::calculate(&cost_map_params, &terrain_map).unwrap())  
    );

    let mut cost_map_quadtree = CostMap::calculate(&cost_map_params, &terrain_map).unwrap();
    let mut cost_map_linear = cost_map_quadtree.clone();

    c.bench_function(
        "CostMap::apply_ground_planned_path::quadtree", 
        |b| b.iter(|| 
            cost_map_quadtree.apply_ground_planned_path_quadtree(&cost_map_params, &ground_planned_path).unwrap())
    );
    c.bench_function(
        "CostMap::apply_ground_planned_path::linear", 
        |b| b.iter(|| 
            cost_map_linear.apply_ground_planned_path_linear(&cost_map_params, &ground_planned_path).unwrap())
    );
}

criterion_group!(benches, cost_map_benchmark);
criterion_main!(benches);