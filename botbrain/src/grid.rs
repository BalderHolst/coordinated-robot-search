use std::fmt::Debug;

use emath::Pos2;

/// A 2D grid of cells
#[derive(Clone)]
pub struct Grid<C: Clone + Default> {
    cells: Vec<C>,
    width: usize,
    height: usize,
}

impl<C: Clone + Default> Default for Grid<C> {
    fn default() -> Self {
        Self {
            cells: vec![],
            width: 0,
            height: 0,
        }
    }
}

impl<C: Clone + Default> Grid<C> {
    /// Create a new grid with the given width and height
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            cells: vec![C::default(); width * height],
            width,
            height,
        }
    }

    /// Width of the grid
    pub fn width(&self) -> usize {
        self.width
    }

    /// Height of the grid
    pub fn height(&self) -> usize {
        self.height
    }

    /// Size of the grid (width, height)
    pub fn size(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    pub fn get_mut(&mut self, x: usize, y: usize) -> Option<&mut C> {
        if x >= self.width || y >= self.height {
            return None;
        }
        self.cells.get_mut(y * self.width + x)
    }

    /// Get the cell at the given position. Returns `None` if the position is out of bounds.
    pub fn get(&self, x: usize, y: usize) -> Option<&C> {
        if x >= self.width || y >= self.height {
            return None;
        }
        self.cells.get(y * self.width + x)
    }

    /// Set the cell at the given position
    pub fn set(&mut self, x: usize, y: usize, cell: C) {
        if let Some(inner) = self.cells.get_mut(y * self.width + x) {
            *inner = cell;
        }
    }

    /// Iterate over all cells in the grid
    pub fn iter(&self) -> impl Iterator<Item = (usize, usize, &C)> + '_ {
        self.cells.iter().enumerate().map(|(i, cell)| {
            let x = i % self.width;
            let y = i / self.width;
            (x, y, cell)
        })
    }

    /// Iterate over all cells in the grid mutably
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (usize, usize, &mut C)> + '_ {
        self.cells.iter_mut().enumerate().map(|(i, cell)| {
            let x = i % self.width;
            let y = i / self.width;
            (x, y, cell)
        })
    }

    /// Fill the grid with a value
    pub fn fill(&mut self, value: C) {
        for cell in self.cells.iter_mut() {
            *cell = value.clone();
        }
    }

    /// Draw a line of cells on the grid
    pub fn line(&mut self, start: Pos2, end: Pos2, width: f32, cell: C) {
        let delta = end - start;
        let length = delta.length();
        for step in 0..=length as usize {
            let t = step as f32 / length;
            let pos = start + delta * t;
            self.circle(pos, width, cell.clone());
        }
    }

    /// Draw a circle of cells on the grid
    pub fn circle(&mut self, center: Pos2, radius: f32, cell: C) {
        self.iter_circle(center, radius).for_each(|(x, y)| {
            self.set(x, y, cell.clone());
        });
    }

    /// Iterate over coordinates within a circle
    pub fn iter_circle(&self, center: Pos2, radius: f32) -> impl Iterator<Item = (usize, usize)> {
        let radius2 = radius * radius;

        let min_y = f32::max((center.y - radius).ceil(), 0.0) as usize;
        let max_y = f32::min((center.y + radius).floor(), self.height as f32 - 1.0) as usize;

        let min_x = f32::max((center.x - radius).ceil(), 0.0) as usize;
        let max_x = f32::min((center.x + radius).floor(), self.width as f32 - 1.0) as usize;

        (min_y..=max_y).flat_map(move |y| {
            (min_x..=max_x)
                .filter(move |x| {
                    let pos = Pos2 {
                        x: *x as f32,
                        y: y as f32,
                    };
                    (pos - center).length().powf(2.0) <= radius2
                })
                .map(move |x| (x, y))
        })
    }
}

impl<C: Clone + Default> Debug for Grid<C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Grid({}x{})", self.width, self.height)
    }
}
