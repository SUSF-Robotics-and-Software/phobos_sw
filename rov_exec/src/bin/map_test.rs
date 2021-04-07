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
    let terrain_map = TerrainMap::generate_random(
        Point2::new(0.1, 0.1), 
        Point2::new(100, 100), 
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

    // Clip the terrain
    // terrain_map.clip_to_rov_view(
    //     Point2::new(2.0, 2.0), 
    //     -0.7, 
    //     0.3..5.0, 
    //     1.0482
    // )?;
    
    terrain_map.save("random_terr_map.json")?;

    // Calculate the cost map from that
    let mut cost_map = CostMap::calculate(&auto_mgr_params.cost_map_params, &terrain_map)?;
    cost_map.apply_ground_planned_path(&auto_mgr_params.cost_map_params, &ground_planned_path)?;

    // Save the map
    cost_map.save("random_cost_map.json")?;

    Ok(())
}