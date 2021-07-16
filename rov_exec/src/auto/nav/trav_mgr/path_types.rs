//! # Path Types
//!
//! Provides the different types of `Path` for the traverse manager

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use std::marker::PhantomData;

use crate::auto::path::Path;

// -----------------------------------------------------------------------------------------------
// TRAITS
// -----------------------------------------------------------------------------------------------

/// Represents the state of a path that can be traversed
pub trait TraverseState {}

/// Represents the state of a path that can be planned
pub trait PlanningState {}

/// Marker trait which shows the underlying path has been set and exists.
pub trait PathExists {}

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct PrimaryPath<S: TraverseState> {
    path: Option<Path>,
    _phantom: PhantomData<S>,
}

#[derive(Debug, Clone)]
pub struct SecondaryPath<S: PlanningState> {
    path: Option<Path>,
    _phantom: PhantomData<S>,
}

/// The underlying path hasn't been planned yet
pub struct NotPlanned;
impl TraverseState for NotPlanned {}
impl PlanningState for NotPlanned {}

/// The path has been planned but hasn't been started yet
pub struct WaitToStart;
impl TraverseState for WaitToStart {}
impl PathExists for WaitToStart {}

/// The path is being traversed
pub struct Traversing;
impl TraverseState for Traversing {}
impl PathExists for Traversing {}

/// The path is being planned
pub struct Planning;
impl PlanningState for Planning {}

/// The operation on the path has been completed
pub struct Complete;
impl TraverseState for Complete {}
impl PlanningState for Complete {}
impl PathExists for Complete {}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl<S: TraverseState> PrimaryPath<S> {
    pub fn new() -> PrimaryPath<NotPlanned> {
        PrimaryPath::<NotPlanned> {
            path: None,
            _phantom: PhantomData,
        }
    }
}

impl<S: PlanningState> SecondaryPath<S> {
    pub fn new() -> SecondaryPath<NotPlanned> {
        SecondaryPath::<NotPlanned> {
            path: None,
            _phantom: PhantomData,
        }
    }
}

impl PrimaryPath<NotPlanned> {
    pub fn set_path(self, path: Path) -> PrimaryPath<WaitToStart> {
        PrimaryPath::<WaitToStart> {
            path: Some(path),
            _phantom: PhantomData,
        }
    }
}

impl PrimaryPath<WaitToStart> {
    pub fn start(self) -> PrimaryPath<Traversing> {
        PrimaryPath::<Traversing> {
            path: self.path,
            _phantom: PhantomData,
        }
    }
}

impl PrimaryPath<Traversing> {
    pub fn complete(self) -> PrimaryPath<Complete> {
        PrimaryPath::<Complete> {
            path: self.path,
            _phantom: PhantomData,
        }
    }
}

impl<S: PlanningState + PathExists> SecondaryPath<S> {
    pub fn get_path(&self) -> &Path {
        &self.path.as_ref().unwrap()
    }
}

impl<S: TraverseState + PathExists> PrimaryPath<S> {
    pub fn get_path(&self) -> &Path {
        &self.path.as_ref().unwrap()
    }
}
