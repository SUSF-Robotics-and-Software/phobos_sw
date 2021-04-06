//! # Grid Map
//!
//! [`GridMap`] is inspired by
//! [grid_map](https://github.com/ANYbotics/grid_map) by ANYbotics. The
//! [paper](https://www.researchgate.net/publication/284415855_A_Universal_Grid_Map_Library_Implementation_and_Use_Case_for_Rough_Terrain_Navigation)
//! gives a good intro the concepts behind `grid_map`.

// ------------------------------------------------------------------------------------------------
// INCLUDES
// ------------------------------------------------------------------------------------------------

use std::{collections::HashMap, fs, hash::Hash, io, ops::Deref, path::Path};

use ndarray::{Array1, Array2, Array3, ArrayView2, arr1, s};
use ndarray_stats::errors::MinMaxError;
use serde::{Deserialize, Serialize, de::DeserializeOwned};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct Point2<T>(Array1<T>)
where
    T: Copy + Clone;

/// A grid-based map containing many layers of information.
///
/// Based on [grid_map](https://github.com/ANYbotics/grid_map) by ANYbotics.
#[derive(Clone, Debug)]
pub struct GridMap<T, L>
where
    T: Clone,
    L: Hash + Eq
{
    /// The size of each grid cell in meters per cell
    pub cell_size: Point2<f64>,

    /// The number of cells in each axis of the map
    pub num_cells: Point2<usize>,

    /// Position of the centre of the map
    pub centre_position: Point2<f64>,

    /// Position of the upper-left point of the map (cell (0, 0))
    upper_left_position: Point2<f64>,

    /// Position of the lower-right point of the map (cell at num_cells - 1)
    lower_right_position: Point2<f64>,

    /// The length of each axis of the map in meters
    axis_length: Point2<f64>,

    /// A map between layer name/index and index into the map data array
    layer_map: HashMap<L, usize>,

    /// Raw map data, a 3D array with dimension order layer, x cell, y cell
    data: Array3<T>
}

/// A simpler version of GridMap which minimises the data serialized
#[derive(Serialize, Deserialize)]
pub(super) struct SerializableGridMap<T, L>
where
    L: Hash + Eq
{
    /// The type of data contained in the map
    data_type: String,

    /// The size of each grid cell in meters per cell
    cell_size: (f64, f64),

    /// The number of cells in each axis of the map
    num_cells: (usize, usize),

    /// Position of the centre of the map
    centre_position: (f64, f64),

    /// A map between layer name/index and index into the map data array
    layer_map: HashMap<L, usize>,

    /// Raw map data, a 3D array with dimension order layer, x cell, y cell
    data: Array3<T>
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, thiserror::Error)]
pub enum GridMapError {
    #[error("Requested position or cell outside map bounds")]
    OutsideMap,

    #[error("Attempted to access unknown layer")]
    UnknownLayer,

    #[error("Map created with no layers, there must be at least one")]
    NoLayers,

    #[error("An IO operation failed: {0}")]
    IoError(io::Error),

    #[error("Couldn't deserialize the given GridMap: {0}")]
    DeserializeError(serde_json::Error),

    #[error("Couldn't serialize the given GridMap: {0}")]
    SerializeError(serde_json::Error),

    #[error("Provided array shape doesn't match the expected shape")]
    IncompatibleShape,

    #[error("Couldn't calculate min/max of map: {0}")]
    MinMaxError(MinMaxError),

    #[error("Grid map is empty")]
    Empty
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl<T, L> GridMap<T, L>
where
    T: Clone,
    L: Hash + Eq + Clone
{
    /// Create a new GridMap with the given cell size, number of cells, position of the centre of
    /// the map, number of layers, and initial empty value.
    pub fn new(
        cell_size: Point2<f64>, 
        num_cells: Point2<usize>, 
        centre_position: Point2<f64>,
        layers: &[L], 
        empty_value: T
    ) -> Result<Self, GridMapError> {
        // Create layer map
        let mut layer_map = HashMap::new();
        
        for (i, layer) in layers.iter().enumerate() {
            layer_map.insert(layer.clone(), i);
        }

        // Check for empty map
        if layer_map.len() == 0 {
            return Err(GridMapError::NoLayers)
        }

        // Calculate the map size
        let axis_length = Point2::new(
            cell_size.x() * (num_cells.x() as f64), 
            cell_size.y() * (num_cells.y() as f64)
        );

        // Calculate extremes of map
        let upper_left_position = Point2::from(
            centre_position.0.clone() + (0.5 * axis_length.0.clone())
        );
        let lower_right_position = Point2::from(
            centre_position.0.clone() - (0.5 * axis_length.0.clone())
        );

        Ok(Self {
            cell_size,
            num_cells: num_cells.clone(),
            centre_position,
            upper_left_position,
            lower_right_position,
            axis_length,
            layer_map,
            data: Array3::from_elem(
                (layers.len(), num_cells.x(), num_cells.y()), 
                empty_value
            )
        })
    }

    pub fn set_layer(&mut self, layer: L, data: Array2<T>) -> Result<(), GridMapError> {
        let layer_idx = self.layer_index(layer)?;

        // Check the data is of the correct size
        if *data.shape() != [self.num_cells.x(), self.num_cells.y()] {
            return Err(GridMapError::IncompatibleShape)
        }

        self.data.slice_mut(s![layer_idx, .., ..]).assign(&data);

        Ok(())
    }

    fn layer_index(&self, layer: L) -> Result<usize, GridMapError> {
        match self.layer_map.get(&layer) {
            Some(l) => Ok(*l),
            None => Err(GridMapError::UnknownLayer)
        }
    }

    pub fn position_in_map(&self, position: &Point2<f64>) -> bool {
        // Test to see if the position is inside the box bound by a and b
        position.x() <= self.upper_left_position.x() 
            && position.x() >= self.lower_right_position.x() 
            && position.y() <= self.upper_left_position.y() 
            && position.y() >= self.lower_right_position.y()
    }

    pub fn cell_in_map(&self, cell: &Point2<usize>) -> bool {
        // Check that the cell indexes are less than the number of cells
        cell.x() < self.num_cells.x() && cell.y() < self.num_cells.y()
    }

    pub fn get(&self, layer: L, cell: &Point2<usize>) -> Result<T, GridMapError> {
        let layer_idx = self.layer_index(layer)?;

        Ok(self.data[[layer_idx, cell.x(), cell.y()]].clone())
    }

    pub fn get_position(&self, layer: L, position: &Point2<f64>) -> Result<T, GridMapError> {
        let layer_idx = self.layer_index(layer)?;

        let cell = self.position_to_cell(position)?;

        Ok(self.data[[layer_idx, cell.x(), cell.y()]].clone())
    }

    pub fn get_mut(&mut self, layer: L, cell: &Point2<usize>) -> Result<&mut T, GridMapError> {
        let layer_idx = self.layer_index(layer)?;

        Ok(&mut self.data[[layer_idx, cell.x(), cell.y()]])
    }

    pub fn get_position_mut(
        &mut self, 
        layer: L, 
        position: &Point2<f64>
    ) -> Result<&mut T, GridMapError> {
        let layer_idx = self.layer_index(layer)?;

        let cell = self.position_to_cell(position)?;

        Ok(&mut self.data[[layer_idx, cell.x(), cell.y()]])
    }

    pub fn get_layer(&self, layer: L) -> Result<ArrayView2<T>, GridMapError> {
        let layer_idx = self.layer_index(layer)?;

        Ok(self.data.slice(s![layer_idx, .., ..]))
    }

    pub fn get_layer_owned(&self, layer: L) -> Result<Array2<T>, GridMapError> {
        let layer_idx = self.layer_index(layer)?;

        Ok(self.data.slice(s![layer_idx, .., ..]).to_owned())
    }

    pub fn cell_position(&self, cell: &Point2<usize>) -> Result<Point2<f64>, GridMapError> {
        // Check cell in map
        if !self.cell_in_map(cell) {
            return Err(GridMapError::OutsideMap)
        }

        // To calculate a cell position we add the upper_left cell's position to the cell_size*cell,
        // which gives us the position of the upper-left of the cell, then we add half a cell size
        // to get the centre.
        let pos = Point2::from(
            self.upper_left_position.0.clone() 
            - (self.cell_size.0.clone() * cell.map(|&v| v as f64))
            - self.cell_size.0.clone()*0.5
        );

        // Check that it's in the map again
        match self.position_in_map(&pos) {
            true => Ok(pos),
            false => Err(GridMapError::OutsideMap)
        }
    }

    pub fn centre_position(&self) -> Point2<f64> {
        self.centre_position.clone()
    }

    pub fn position_to_cell(&self, position: &Point2<f64>) -> Result<Point2<usize>, GridMapError> {
        // Check position in map
        if !self.position_in_map(position) {
            return Err(GridMapError::OutsideMap)
        }

        // Cell of a given position is equal to the difference between the upper left, normalized by
        // the cell size, and floored to get to usize.
        let cell = Point2::from(
            ((self.upper_left_position.0.clone() - position.0.clone()) / self.cell_size.0.clone())
            .map(|v| v.floor() as usize)
        );

        // Double check cell in map
        match self.cell_in_map(&cell) {
            true => Ok(cell),
            false => Err(GridMapError::OutsideMap)
        }   
    }

    pub fn map<F: Fn(Point2<usize>, Point2<f64>, T) -> T>(
        &self, 
        layer: L, 
        f: F
    ) -> Result<Self, GridMapError> {
        let layer_idx = self.layer_index(layer)?;

        let mut out = self.clone();

        for (idx, t) in out.data.slice_mut(s![layer_idx, .., ..]).indexed_iter_mut() {
            // Get cell index and position
            let cell = Point2::new(idx.0, idx.1);
            let pos = self.cell_position(&cell)?;

            *t = f(cell, pos, t.clone());
        }

        Ok(out)
    }

    /// Maps data from one grid map into another.
    pub fn map_into<N, M, F>(
        &self, 
        from_layer: L, 
        into_layer: M,
        into: &GridMap<N, M>,
        f: F
    ) -> Result<GridMap<N, M>, GridMapError> 
    where
        N: Clone,
        M: Hash + Eq + Clone,
        F: Fn(Point2<usize>, Point2<f64>, T) -> N
    {
        let into_layer_idx = into.layer_index(into_layer)?;

        let mut out = into.clone();

        for (idx, t) in out.data.slice_mut(s![into_layer_idx, .., ..]).indexed_iter_mut() {
            // Get cell index and position
            let cell = Point2::new(idx.0, idx.1);
            let pos = self.cell_position(&cell)?;

            *t = f(cell.clone(), pos, self.get(from_layer.clone(), &cell)?);
        }

        Ok(out)
    }
}

impl<T, L> GridMap<T, L> 
where 
    T: Clone + PartialOrd,
    L: Hash + Eq + Clone
{
    pub fn min(&self, _layer: L) -> Result<T, GridMapError> {
        todo!();
        // let layer_idx = self.layer_index(layer)?;

        // self.data.slice(s![layer_idx, .., ..]).min()
        //     .map_err(|e| GridMapError::MinMaxError(e))
        //     .map(|v| v.clone())
    }

    pub fn max(&self, _layer: L) -> Result<T, GridMapError> {
        todo!();
        // let layer_idx = self.layer_index(layer)?;

        // self.data.slice(s![layer_idx, .., ..]).max()
        //     .map_err(|e| GridMapError::MinMaxError(e))
        //     .map(|v| v.clone())
    }
}

impl<'de, T, L> GridMap<T, L>
where 
    T: Clone + Serialize + DeserializeOwned,
    L: Hash + Eq + Clone + Serialize + DeserializeOwned
{
    /// Load the grid map from the given path
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self, GridMapError> {
        // Read file into string
        let s = fs::read_to_string(path)
            .map_err(|e| GridMapError::IoError(e))?;

        // Deserialize the gridmap from the path
        let ser: SerializableGridMap<T, L> = serde_json::from_str(&s)
            .map_err(|e| GridMapError::DeserializeError(e))?;

        ser.to_grid_map()
    }

    /// Save the grid map to the given path
    pub fn save<P: AsRef<Path>>(&self, path: P) -> Result<(), GridMapError> {
        // Serialize the map to a string
        let s = serde_json::to_string(&SerializableGridMap::from_grid_map(self))
            .map_err(|e| GridMapError::SerializeError(e))?;

        fs::write(path, s).map_err(|e| GridMapError::IoError(e))
    }
}

impl<T, L> SerializableGridMap<T, L>
where
    T: Clone,
    L: Hash + Eq + Clone
{
    pub(super) fn from_grid_map(map: &GridMap<T, L>) -> Self {
        Self {
            data_type: std::any::type_name::<T>().into(),
            cell_size: (map.cell_size.x(), map.cell_size.y()),
            num_cells: (map.num_cells.x(), map.num_cells.y()),
            centre_position: (map.centre_position.x(), map.centre_position.y()),
            layer_map: map.layer_map.clone(),
            data: map.data.clone(),
        }
    }

    pub(super) fn to_grid_map(self) -> Result<GridMap<T, L>, GridMapError> {
        let mut lms: Vec<(&L, &usize)> = self.layer_map.iter().collect();
        lms.sort_by(|a, b| a.1.partial_cmp(b.1).unwrap());
        let layers: Vec<L> = lms.iter().map(|v| v.0.clone()).collect();

        let mut map = GridMap::new(
            Point2::new(self.cell_size.0, self.cell_size.1),
            Point2::new(self.num_cells.0, self.num_cells.1),
            Point2::new(self.centre_position.0, self.centre_position.1),
            &layers,
            self.data[[0, 0, 0]].clone()
        )?;

        map.data = self.data;

        Ok(map)
    }
}

impl<T> Point2<T>
where 
    T: Copy + Clone
{
    pub fn new(x: T, y: T) -> Self {
        Self(arr1(&[x, y]))
    }

    pub fn x(&self) -> T {
        self.0[0]
    }

    pub fn y(&self) -> T {
        self.0[1]
    }

    pub fn x_ref(&self) -> &T {
        &self.0[0]
    }

    pub fn y_ref(&self) -> &T {
        &self.0[1]
    }
}

impl<T> From<Array1<T>> for Point2<T>
where 
    T: Copy + Clone
{
    fn from(array: Array1<T>) -> Self {
        Self(array)
    }
}

impl<T> From<(T, T)> for Point2<T>
where 
    T: Copy + Clone
{
    fn from(tuple: (T, T)) -> Self {
        Point2::new(tuple.0, tuple.1)
    }
}

impl<T> Deref for Point2<T>
where 
    T: Copy + Clone
{
    type Target = Array1<T>;

    fn deref(&self) -> &Self::Target {
        &self.0    
    }
}

// ------------------------------------------------------------------------------------------------
// TESTS
// ------------------------------------------------------------------------------------------------

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_grid_map() -> Result<(), GridMapError> { 
        // Create new grid map
        let map: GridMap<Option<f64>, i32> = GridMap::new(
            Point2::new(1.0, 1.0),
            Point2::new(20, 30),
            Point2::new(10.0, 15.0),
            &[0, 1],
            None
        )?;

        // Test out of bounds detection
        assert!(map.position_in_map(&Point2::new(10.0, 15.0)));
        assert!(map.position_in_map(&Point2::new(0.0, 0.0)));
        assert!(map.position_in_map(&Point2::new(20.0, 30.0)));
        assert_eq!(map.position_in_map(&Point2::new(-20.0, 30.0)), false);
        assert_eq!(map.position_in_map(&Point2::new(20.1, 30.0)), false);

        // Test position->cell
        assert_eq!(map.position_to_cell(&Point2::new(20.0, 30.0))?, Point2::new(0, 0));
        assert_eq!(map.position_to_cell(&Point2::new(10.0, 10.0))?, Point2::new(10,20));
        assert_eq!(map.position_to_cell(&Point2::new(5.9, 12.5))?, Point2::new(14, 17));
        
        // Test cell->position
        assert_eq!(map.cell_position(&Point2::new(0, 0))?, Point2::new(19.5, 29.5));
        assert_eq!(map.cell_position(&Point2::new(10, 20))?, Point2::new(9.5, 9.5));
        assert_eq!(map.cell_position(&Point2::new(14, 17))?, Point2::new(5.5, 12.5));

        Ok(())
    }
}