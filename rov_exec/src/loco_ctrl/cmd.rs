//! Commands passed into LocoCtrl

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// A command to execute a particular manouvre
#[derive(Clone, Copy, Debug)]
pub struct MnvrCommand {
    
    /// The type of manouvre to perform
    pub mnvr_type: MnvrType,

    /// The curvature (1/ radius of turn) to use. Used for Ackerman and Skid
    /// Steer
    /// 
    /// Units: 1/meters
    pub curvature_m: Option<f64>,

    /// The speed with which the Rover body should move relative to the terrain.
    /// Used for Ackerman and Skid Steer.
    /// 
    /// Units: meters/second
    pub speed_ms: Option<f64>,

    /// The rate at which the rover shall turn about it's centre. Used for
    /// Point Turn only.
    /// 
    /// Units: radians/second
    pub turn_rate_rads: Option<f64>,
}

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// Possible manouvres to be executed by LocoCtrl.
#[derive(Clone, Copy, Debug)]
pub enum MnvrType {
    /// No manouvre - interpreted as continue with last manouvre.
    None,
    /// Stop - bring the rover to a full stop.
    Stop,
    /// Ackerman - perform an ackerman coordinated turn (COR outside wheelbase)
    Ackerman,
    /// Point turn - perform a turn about the centre of the wheelbase
    PointTurn,
    /// Skid steer - tank like steering using differential speed between left 
    /// and right wheels.
    SkidSteer
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl MnvrCommand {

    /// Determine if the command is valid (i.e. contains all required data).
    pub fn is_valid(&self) -> bool {
        match self.mnvr_type {
            MnvrType::Stop | MnvrType::None => true,
            MnvrType::Ackerman | MnvrType::SkidSteer => 
                !vec![self.curvature_m, self.speed_ms].contains(&None),
            MnvrType::PointTurn => 
                self.turn_rate_rads.is_some()
        }
    }
}