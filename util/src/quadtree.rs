//! # Quadtree Implementation
//!
//! This is an implementation of a quadtree, as described in [the wikipedia
//! article](https://en.wikipedia.org/wiki/Quadtree).

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use std::fmt::{Debug, Formatter};
use serde::{Serialize, Deserialize};
use nalgebra::Vector2;

// -----------------------------------------------------------------------------------------------
// CONSTANTS
// -----------------------------------------------------------------------------------------------

/// Number of points per QuadTree
pub const CAPACITY: usize = 4;

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

/// Represents a quad with a centre and half-width.
#[derive(Clone, Copy, Default, Serialize, Deserialize)]
pub struct Quad {
    centre: Vector2<f64>,
    half_width: f64
}

/// An implementation of a QuadTree
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct QuadTree {
    /// The bounds of this node
    boundary: Quad,

    /// Points stored in this node
    points: Vec<Vector2<f64>>,

    /// North West child of the node
    north_west: Option<Box<QuadTree>>,

    /// North East child of the node
    north_east: Option<Box<QuadTree>>,

    /// South West child of the node
    south_west: Option<Box<QuadTree>>,

    /// South East child of the node
    south_east: Option<Box<QuadTree>>
}

// -----------------------------------------------------------------------------------------------
// ENUMS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, thiserror::Error)]
pub enum QuadTreeError {
    #[error("The given point {0} was not in the bounds of the quadtree {1:?}")]
    PointNotInBounds(Vector2<f64>, Quad),
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl Quad {
    /// Creates a new quad with the given `centre` and `half_width`.
    pub fn new(centre: Vector2<f64>, half_width: f64) -> Self {
        Self {
            centre,
            half_width
        }
    }

    /// Returns `true` if `point` is inside this [`Quad`]
    pub fn contains(&self, point: &Vector2<f64>) -> bool {
        (self.centre[0] - self.half_width) <= point[0]
        && (self.centre[0] + self.half_width) > point[0]
        && (self.centre[1] - self.half_width) <= point[1]
        && (self.centre[1] + self.half_width) > point[1]
    }

    /// Returns `true` if `other` intersects with this [`Quad`].
    pub fn intersects(&self, other: &Quad) -> bool {
        other.centre[0] - other.half_width > self.centre[0] + self.half_width
        || other.centre[0] + other.half_width < self.centre[0] - self.half_width
        || other.centre[1] + other.half_width > self.centre[1] - self.half_width
        || other.centre[1] - other.half_width < self.centre[1] + self.half_width
    }
}

impl Debug for Quad {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f, 
            "Quad [{}, {}; {}, {}]", 
            self.centre[0] - self.half_width,
            self.centre[0] + self.half_width,
            self.centre[1] - self.half_width,
            self.centre[1] + self.half_width
        )
    }
}

impl QuadTree {
    pub fn new(boundary: Quad) -> Self {
        Self {
            boundary,
            points: Vec::new(),
            north_west: None,
            north_east: None,
            south_west: None,
            south_east: None,
        }
    }

    /// Insert a point into the QuadTree.
    pub fn insert(&mut self, point: Vector2<f64>) -> Result<(), QuadTreeError> {

        // Check if it's in the tree
        if !self.boundary.contains(&point) {
            return Err(QuadTreeError::PointNotInBounds(point, self.boundary.clone()));
        }

        // If there's a space in the tree and its's not been divided add it to the points list
        if self.points.len() < CAPACITY && self.north_west.is_none() {
            self.points.push(point);
            return Ok(())
        }

        // Otherwise subdivide if needed
        if self.north_west.is_none() {
            self.subdivide();

            // Remove all points from self and put them into the children
            let my_points = self.points.clone();
            self.points.clear();
            for point in my_points {
                self.insert(point).ok();
            }
        }


        // And add the point to the first quad it will fit into
        match self.north_west {
            Some(ref mut qt) => if qt.insert(point).is_ok() {
                return Ok(())
            }
            _ => unreachable!()
        }
        match self.north_east {
            Some(ref mut qt) => if qt.insert(point).is_ok() {
                return Ok(())
            }
            _ => unreachable!()
        }
        match self.south_west {
            Some(ref mut qt) => if qt.insert(point).is_ok() {
                return Ok(())
            }
            _ => unreachable!()
        }
        match self.south_east {
            Some(ref mut qt) => if qt.insert(point).is_ok() {
                return Ok(())
            }
            _ => unreachable!()
        }

        unreachable!("The point couldn't be added to the quad tree, for some unknown reason")
    }

    /// Return a list of all points within the given [`Quad`].
    pub fn query_in_quad(&self, quad: &Quad) -> Vec<Vector2<f64>> {
        
        // Create points list
        let mut points = Vec::new();

        // Check that quad is in the tree, if not return an empty list
        if !self.boundary.intersects(&quad) {
            return points
        }

        // Check self for the points
        for point in self.points.iter() {
            if quad.contains(point) {
                points.push(point.clone())
            }
        }

        // If the tree has no children exit now
        if self.north_west.is_none() {
            return points
        }

        // Otherwise search the children
        match self.north_west {
            Some(ref qt) => points.extend(&qt.query_in_quad(quad)),
            None => unreachable!()
        }
        match self.north_east {
            Some(ref qt) => points.extend(&qt.query_in_quad(quad)),
            None => unreachable!()
        }
        match self.south_west {
            Some(ref qt) => points.extend(&qt.query_in_quad(quad)),
            None => unreachable!()
        }
        match self.south_east {
            Some(ref qt) => points.extend(&qt.query_in_quad(quad)),
            None => unreachable!()
        }

        points
    }

    fn subdivide(&mut self) {
        let hw = self.boundary.half_width / 2.0;

        self.north_west = Some(Box::new(QuadTree::new(Quad::new(
            self.boundary.centre + Vector2::new(-hw, hw),
            hw
        ))));
        self.north_east = Some(Box::new(QuadTree::new(Quad::new(
            self.boundary.centre + Vector2::new(hw, hw),
            hw
        ))));
        self.south_west = Some(Box::new(QuadTree::new(Quad::new(
            self.boundary.centre + Vector2::new(-hw, -hw),
            hw
        ))));
        self.south_east = Some(Box::new(QuadTree::new(Quad::new(
            self.boundary.centre + Vector2::new(hw, -hw),
            hw
        ))));
    }

    /// Returns the number of points stored in the quadtree, including in all children
    pub fn len(&self) -> usize {
        let mut len = self.points.len();

        match self.north_west {
            Some(ref qt) => len += qt.len(),
            None => ()
        }
        match self.north_east {
            Some(ref qt) => len += qt.len(),
            None => ()
        }
        match self.south_west {
            Some(ref qt) => len += qt.len(),
            None => ()
        }
        match self.south_east {
            Some(ref qt) => len += qt.len(),
            None => ()
        }

        return len;
    }
}

// -----------------------------------------------------------------------------------------------
// TESTS
// -----------------------------------------------------------------------------------------------

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_query() -> Result<(), Box<dyn std::error::Error>> {

        // Build bounding quad
        let bounding_quad = Quad::new([5.0, 5.0].into(), 5.0);

        println!("Bounding quad is {:?}", bounding_quad);

        // Build quadtree
        let mut qt = QuadTree::new(bounding_quad);

        // Insert a known group of points into the tree
        for val in 1..9 {
            println!("Adding for {}, {}", val, val);
            qt.insert([val as f64, val as f64].into())?;
            qt.insert([val as f64, 1.0].into())?;
            qt.insert([1.0, val as f64].into())?;
        }

        // Test many queries of these points, to confirm we're getting what we expect
        let mut query_quad = Quad::new([1.5, 1.5].into(), 1.0);
        assert_eq!(qt.query_in_quad(&query_quad).len(), 6);

        query_quad = Quad::new([5.0, 5.0].into(), 2.0);
        assert_eq!(qt.query_in_quad(&query_quad).len(), 5);

        Ok(())
    }
}