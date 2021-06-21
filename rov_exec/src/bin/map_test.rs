//! # Simple Map Test

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use comms_if::tc::auto::PathSpec;
use rov_lib::auto::{
    auto_mgr::AutoMgrParams, 
    map::{CostMap, Point2, TerrainMap, TerrainMapLayer}, 
    path::Path, loc::Pose
};
use color_eyre::Result;
use nalgebra::{Vector3, UnitQuaternion};
use util::params;

fn main() -> Result<()> {
    color_eyre::install()?;

    // CostMapParams, loaded from automgr params
    let auto_mgr_params: AutoMgrParams = params::load("auto_mgr.toml")?;
    
    // Generate a random terrain map
    let mut terrain_map = TerrainMap::generate_random(
        Point2::new(0.05, 0.05), 
        Point2::new(200, 200), 
        Point2::new(5.0, 5.0),
        Point2::new(0.1, 0.1),
        Point2::new(0.0, 0.0)
    )?;
    
    // Ground planned path to test
    let ground_planned_path = Path::from_path_spec(
        PathSpec::AckSeq {
            separation_m: 0.05,
            seq: vec![0.5, 3.1415, 0.0, 4.0, -0.5, 3.1415, 0.0, 2.0, -0.5, 3.1415, 0.0, 6.0]
        },
        &Pose {
            position_m_lm: Vector3::new(
                1.0, 1.0, 
                terrain_map.get_position(
                    TerrainMapLayer::Height, 
                    &(1.0, 1.0).into()
                )?.unwrap()
            ),
            attitude_q_lm: UnitQuaternion::identity(),
        }
    )?;

    // Write the path to a file
    serde_json::to_writer_pretty(
        std::fs::File::create("test_path.json")?, 
        &ground_planned_path
    )?;

    terrain_map.save("random_terr_map.json")?;

    // // Clip the terrain to rover field of view
    // terrain_map.clip_to_rov_view(
    //     Point2::new(1.0, 1.0), 
    //     0.0, 
    //     0.3..2.0, 
    //     1.0482
    // )?;
    
    terrain_map.save("random_terr_map_clipped.json")?;

    // Calculate the cost map from the terrain map, and time it
    let start = std::time::Instant::now();
    let mut cost_map = CostMap::calculate(&auto_mgr_params.cost_map_params, &terrain_map)?;
    let calc_stop = std::time::Instant::now();
    cost_map.apply_ground_planned_path(&auto_mgr_params.cost_map_params, &ground_planned_path)?;
    let gpp_stop = std::time::Instant::now();
    let ground_path_cost = cost_map.get_path_cost(&ground_planned_path);
    let gpp_cost_stop = std::time::Instant::now();

    println!("Ground path cost = {:?}", ground_path_cost);
    println!("CostMap::calculate took {} ms", (calc_stop - start).as_millis());
    println!("CostMap::apply_ground_planned_path took {} ms", (gpp_stop - calc_stop).as_millis());
    println!("CostMap::get_path_cost took {} ms", (gpp_cost_stop - gpp_stop).as_millis());

    // calculate the cost of the ground path

    // Save the map
    cost_map.save("random_cost_map.json")?;

    Ok(())
}