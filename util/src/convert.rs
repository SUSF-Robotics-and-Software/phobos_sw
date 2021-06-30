//! Implements `Convert` functions between various external types.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use nalgebra::{allocator::Allocator, DefaultAllocator, DimName, Point, Scalar, VectorN};

// ------------------------------------------------------------------------------------------------
// TRAITS
// ------------------------------------------------------------------------------------------------

pub trait Convert<O> {
    fn convert(&self) -> O;
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl<N, D> Convert<VectorN<N, D>> for Point<N, D>
where
    N: Scalar,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
{
    fn convert(&self) -> VectorN<N, D> {
        self.coords.clone()
    }
}

impl<N, D> Convert<Point<N, D>> for VectorN<N, D>
where
    N: Scalar,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
{
    fn convert(&self) -> Point<N, D> {
        Point {
            coords: self.clone(),
        }
    }
}
