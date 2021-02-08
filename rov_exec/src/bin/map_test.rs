//! # Simple Map Test

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use rov_lib::auto::map::{CostMap, Point2, TerrainMap};
use color_eyre::Result;

fn main() -> Result<()> {
    color_eyre::install()?;

    // Generate a random terrain map
    let mut terrain_map = TerrainMap::generate_random(
        Point2::new(0.1, 0.1), 
        Point2::new(100, 100), 
        Point2::new(5.0, 5.0),
        Point2::new(0.1, 0.1),
        Point2::new(0.0, 0.0)
    )?;

    // Clip the terrain
    terrain_map.clip_to_rov_view(
        Point2::new(2.0, 2.0), 
        -0.7, 
        0.3..5.0, 
        1.0482
    )?;
    
    terrain_map.save("random_terr_map.json")?;

    // Calculate the cost map from that
    let cost_map = CostMap::calculate(&terrain_map)?;

    // Save the map
    cost_map.save("random_cost_map.json")?;

    Ok(())
}