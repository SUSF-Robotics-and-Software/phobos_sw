//! # Autonomy Module
//!
//! This module is responsible for managing the execution of the autonomy system, i.e. the high
//! level autonomy state machine. The state machine is broken down into a number of modes:
//!
//! - `Off` - The autonomy processing is not active
//! - `Pause` - The autonomy system has been paused. It may be resumed with the `auto resume` tc.
//! - `Stop` - The autonomy system is stopping, ready to move into Off mode.
//! - `ImgStop` - The autonomy system is acquiring a pair of images while stationary.
//! - `Mnvr` - Perform an autonomous LocoCtrl manouvre, TrajCtrl is not used.
//! - `Follow` - The autonomy system is following a ground-planned path using TrajCtrl.
//! - `Check` - The autonomy system is checking a ground-planned path and navigating around
//!   obstacles in the path.
//! - `Goto` - The autonomy system is navigating itself towards a given coordinate.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

// use comms_if::tc::{
//     auto::AutoCmd,
//     loco_ctrl::MnvrCmd,
// };
// use log::{info, warn};
use map::GridMapError;
// use nav::NavCtrl;

// use self::{loc::LocMgr, nav::{NavCtrlType, NavError}};

pub use auto_mgr::AutoMgr;

// ------------------------------------------------------------------------------------------------
// MODULES
// ------------------------------------------------------------------------------------------------

/// Automation Manager module
pub mod auto_mgr;

/// Localisation module - provides the rover with an idea of where it is in the world
pub mod loc;

/// Trajectory control module - keeps the rover on the given path
pub mod traj_ctrl;

/// Defines path types
pub mod path;

/// Navigation module - provides path planning using cost maps
pub mod nav;

/// Map module - provides implementations for terrain and cost maps
pub mod map;

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

// impl AutoMgr {
//     /// Initialise a new instance of the autonomy manager
//     pub fn init(params_path: &str) -> Result<Self, AutoMgrError> {

//         let params = match util::params::load(params_path) {
//             Ok(p) => p,
//             Err(e) => return Err(AutoMgrError::ParamLoadError(e))
//         };

//         Ok(Self {
//             params,
//             mode: AutoMgrMode::Off,
//             auto_cmd: None,
//             output_mnvr_cmd: None,
//             paused_from_mode: None,
//             mnvr_state: None,
//             nav_ctrl: None,
//             cycles_in_current_mode: 0
//         })
//     }

//     /// Cyclically process the AutoMgr.
//     pub fn step(&mut self, auto_cmd: Option<AutoCmd>) -> Result<Option<MnvrCmd>, AutoMgrError> {
//         // Set the command in self
//         if auto_cmd.is_some() {
//             self.auto_cmd = auto_cmd;
//         }

//         // Execute the state machine
//         match self.mode {
//             AutoMgrMode::Off => self.mode_off()?,
//             AutoMgrMode::Pause => self.mode_pause()?,
//             AutoMgrMode::Stop => self.mode_stop()?,
//             AutoMgrMode::Mnvr => self.mode_mnvr()?,
//             AutoMgrMode::Check | AutoMgrMode::Follow | AutoMgrMode::Goto => {
//                 match self.nav_ctrl {
//                     Some(ref mut nc) => {
//                         // Get and set the pose of the controller. If no pose move to pause
//                         let pose = match LocMgr::_get_pose() {
//                             Some(p) => p,
//                             None => {
//                                 self.pause()?;
//                                 return Ok(None)
//                             }
//                         };
//                         nc.set_pose(&pose);
                        
//                         // Step the controller
//                         nc.step()?;
//                     },
//                     None => return Err(AutoMgrError::NavCtrlStateNotInit)
//                 }
//             }
//             m => {
//                 warn!("AutoMgr does not yet support the {:?} mode", m);
//                 self.set_mode(AutoMgrMode::Stop);
//             }
//         }

//         // Increment the number of cycles in the current mode
//         self.cycles_in_current_mode += 1;

//         Ok(self.output_mnvr_cmd)
//     }

//     /// Returns true if the manager is on.
//     pub fn is_on(&self) -> bool {
//         match self.mode {
//             AutoMgrMode::Off => false,
//             _ => true
//         }
//     }

//     /// Set the current mode
//     fn set_mode(&mut self, mode: AutoMgrMode) {
//         self.mode = mode;
//         self.cycles_in_current_mode = 0;
//     }

//     /// Pause current command execution
//     fn pause(&mut self) -> Result<(), AutoMgrError> {
        
//         self.paused_from_mode = Some(self.mode);
        
//         self.set_mode(AutoMgrMode::Pause);

//         info!("Autonomy execution paused, issue Resume to continue or Abort to be able to issue new\
//          commands");

//         Ok(())
//     }

//     /// Abort execution by clearing the current command and switching to stop mode.
//     fn abort(&mut self) -> Result<(), AutoMgrError> {
//         info!("Aborting autonomy execution");

//         // Switch into stop mode
//         self.set_mode(AutoMgrMode::Stop);

//         // Clear current auto_cmd
//         self.auto_cmd = None;

//         Ok(())
//     }

//     /// Execute the off mode.
//     ///
//     /// In this mode no processing may take place however the following commands may be issued:
//     /// - Mnvr,
//     /// - Follow,
//     /// - Check,
//     /// - Goto
//     fn mode_off(&mut self) -> Result<(), AutoMgrError> {
//         // Handle incoming commands
//         match &self.auto_cmd {
//             // If recieved a manouvre command switch into manouvre mode
//             Some(AutoCmd::Manouvre(_)) => {
//                 info!("Recieved new autonomy command: {:#?}", self.auto_cmd);
//                 self.set_mode(AutoMgrMode::Mnvr);
//             },
//             // Switch into follow mode
//             Some(AutoCmd::Follow { .. }) => {
//                 info!("Recieved new autonomy command: {:#?}", self.auto_cmd);
//                 self.set_mode(AutoMgrMode::Follow);
//             },
//             // Switch into check mode
//             Some(AutoCmd::Check{ .. }) => {
//                 info!("Recieved new autonomy command: {:#?}", self.auto_cmd);
//                 self.set_mode(AutoMgrMode::Follow);
//             },
//             // Switch into goto mode
//             Some(AutoCmd::Goto{ .. }) => {
//                 info!("Recieved new autonomy command: {:#?}", self.auto_cmd);
//                 self.set_mode(AutoMgrMode::Goto);
//             },
//             Some(c) => {
//                 warn!("Cannot execute {:?} as AutoMgr is in the Off mode", c);
//                 self.auto_cmd.take();
//             },
//             None => ()
//         }

//         Ok(())
//     }

//     /// Execute the pause mode.
//     ///
//     /// In this mode the execution of the current command is postponed until the resume command is 
//     /// issued.
//     fn mode_pause(&mut self) -> Result<(), AutoMgrError> {
        
//         match &self.auto_cmd {
//             // If the current command is resume exit pause mode
//             Some(AutoCmd::Resume) => {
//                 match self.paused_from_mode.take() {
//                     Some(m) => {
//                         info!("Resuming autonomy execution");
//                         self.set_mode(m);
//                     },
//                     None => {
//                         self.abort()?;
//                         return Err(AutoMgrError::NoModeToResumeTo);
//                     }
//                 }
//             },
//             // Or abort
//             Some(AutoCmd::Abort) => {
//                 self.abort()?
//             }
//             _ => ()
//         }

//         Ok(())
//     }

//     /// Execute the stop mode.
//     ///
//     /// This mode issues a MnvrCmd::Stop to LocoCtrl before switching to Off mode.
//     fn mode_stop(&mut self) -> Result<(), AutoMgrError> {
//         // Clear auto command
//         self.auto_cmd = None;

//         self.output_mnvr_cmd = Some(MnvrCmd::Stop);

//         // Must spend at least mode in stop
//         if self.cycles_in_current_mode >= 2 {
//             self.set_mode(AutoMgrMode::Off);
//         }

//         Ok(())
//     }
// }