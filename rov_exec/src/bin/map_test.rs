//! # Simple Map Test

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::time::Instant;

use color_eyre::{eyre::Context, Result};
use comms_if::tc::auto::PathSpec;
use log::{info, LevelFilter};
use nalgebra::{Point2, UnitQuaternion, Vector3};
use rov_lib::auto::{
    auto_mgr::AutoMgrParams,
    loc::Pose,
    map::{CostMap, TerrainMap},
    nav::{path_planner::PathPlanner, NavError, NavPose},
    path::Path,
};
use util::{logger::logger_init, params, session::Session};

fn main() -> Result<()> {
    color_eyre::install()?;

    // Initialise session
    let session = Session::new("map_test", "sessions").wrap_err("Failed to create the session")?;

    // Initialise logger
    logger_init(LevelFilter::Trace, &session).wrap_err("Failed to initialise logging")?;

    // CostMapParams, loaded from automgr params
    let auto_mgr_params: AutoMgrParams = params::load("auto_mgr.toml")?;

    // Generate a random terrain map
    let terrain_map = TerrainMap::generate_random(
        auto_mgr_params.terrain_map_params,
        Point2::new(0.1, 0.1),
        Point2::new(0.0, 0.0),
    )?;

    // Starting pose
    let start_pose = &Pose {
        position_m: Vector3::new(1.0, 1.0, 0.0),
        attitude_q: UnitQuaternion::identity(),
    };

    // Ground planned path to test
    let ground_planned_path = Path::from_path_spec(
        PathSpec::AckSeq {
            separation_m: 0.05,
            seq: vec![
                0.5,
                std::f64::consts::PI,
                0.0,
                4.0,
                -0.5,
                std::f64::consts::PI,
                0.0,
                2.0,
                -0.5,
                std::f64::consts::PI,
                0.0,
                6.0,
            ],
        },
        start_pose,
    )?;

    // Write the path to a file
    util::session::save("test_path.json", ground_planned_path.clone());
    util::session::save("random_terr_map.json", terrain_map.clone());

    // Calculate the cost map from the terrain map, and time it
    let start = std::time::Instant::now();
    let mut cost_map = CostMap::calculate(auto_mgr_params.cost_map_params, &terrain_map)?;
    let calc_stop = std::time::Instant::now();
    cost_map.apply_ground_planned_path(&ground_planned_path)?;
    let gpp_stop = std::time::Instant::now();
    let ground_path_cost = cost_map.get_path_cost(&ground_planned_path);
    let gpp_cost_stop = std::time::Instant::now();

    info!("Ground path cost = {:?}", ground_path_cost);
    info!(
        "CostMap::calculate took {} ns",
        (calc_stop - start).as_nanos()
    );
    info!(
        "CostMap::apply_ground_planned_path took {} ns",
        (gpp_stop - calc_stop).as_nanos()
    );
    info!(
        "CostMap::get_path_cost took {} ns",
        (gpp_cost_stop - gpp_stop).as_nanos()
    );

    // Save the map
    util::session::save("random_cost_map.json", cost_map.clone());

    // Create the path planner
    let path_planner = PathPlanner::new(auto_mgr_params.path_planner);

    // Plan a path over the cost map
    let start = Instant::now();
    let planner_result = match path_planner.plan_direct(
        &cost_map,
        &NavPose::from_parent_pose(&start_pose),
        &NavPose::from_parts(&Point2::new(4.0, 4.0), &std::f64::consts::FRAC_PI_2),
        // &NavPose::from_path_last_point(&ground_planned_path),
        2,
    ) {
        Ok(p) => p,
        Err(NavError::BestPathNotAtTarget(p)) => p,
        Err(e) => return Err(e).wrap_err("Couldn't plan path"),
    };
    let stop = Instant::now();

    info!("Path planning took {} ns", (stop - start).as_nanos());

    // Write the paths to a file
    util::session::save("planned_path.json", planner_result);

    // Wait for the session to stop
    session.exit();

    Ok(())
}
