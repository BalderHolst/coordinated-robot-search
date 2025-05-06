//! Simple 2D fixed size grid for storing generic cells

use std::fmt::Debug;

use emath::{normalized_angle, Pos2};

/// A 2D grid of cells
#[derive(Clone)]
pub struct Grid<C: Clone + Default> {
    cells: Vec<C>,
    width: usize,
    height: usize,
}

impl<C: Clone + Default> Default for Grid<C> {
    fn default() -> Self {
        Self::empty()
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

    /// Create a new grid with the given dimensions
    pub fn with_size((width, height): (usize, usize)) -> Self {
        Self::new(width, height)
    }

    /// Create an empty grid of zero size
    pub fn empty() -> Self {
        Self {
            cells: vec![],
            width: 0,
            height: 0,
        }
    }

    /// Turn the grid into a vector of cells
    pub fn into_cells(self) -> Vec<C> {
        self.cells
    }

    /// Access the underlying cell array
    pub fn cells(&self) -> &[C] {
        &self.cells
    }

    /// Access the underlying cell array mutably
    pub fn cells_mut(&mut self) -> &mut [C] {
        &mut self.cells
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

    /// Get a mutable reference to the cell at the given position. Returns `None` if the position is out of bounds.
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
    pub fn set_line(&mut self, start: Pos2, end: Pos2, width: f32, cell: C) {
        let delta = end - start;
        let length = delta.length();
        for step in 0..=length as usize {
            let t = step as f32 / length;
            let pos = start + delta * t;
            self.set_circle(pos, width, cell.clone());
        }
    }

    /// Draw a circle of cells on the grid
    pub fn set_circle(&mut self, center: Pos2, radius: f32, cell: C) {
        self.iter_circle(center, radius).for_each(|(x, y)| {
            self.set(x, y, cell.clone());
        });
    }

    /// Set a cone of cells on the grid
    pub fn set_cone(&mut self, center: Pos2, radius: f32, angle: f32, fov: f32, cell: C) {
        self.iter_cone(center, radius, angle, fov)
            .for_each(|(x, y)| {
                self.set(x, y, cell.clone());
            });
    }

    /// Iterate over coordinates within a cone
    pub fn iter_cone(
        &self,
        center: Pos2,
        radius: f32,
        angle: f32,
        fov: f32,
    ) -> impl Iterator<Item = (usize, usize)> {
        let angle = normalized_angle(angle);
        self.iter_circle(center, radius).filter(move |(x, y)| {
            let pos = Pos2 {
                x: *x as f32,
                y: *y as f32,
            };
            let dir = pos - center;
            let angle = dir.angle() - angle;
            let angle = normalized_angle(angle);
            angle.abs() < fov / 2.0
        })
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

    /// Iterate over coordinates within a line
    pub fn iter_line(&self, start: Pos2, end: Pos2) -> impl Iterator<Item = (usize, usize)> + '_ {
        let delta = end - start;
        let dir = delta.normalized();
        let length = delta.length() as usize;
        (0..=length).filter_map(move |n| {
            let cur = start + dir * n as f32;
            if cur.x < 0.0
                || cur.x >= self.width as f32
                || cur.y < 0.0
                || cur.y >= self.height as f32
            {
                None
            } else {
                Some((cur.x as usize, cur.y as usize))
            }
        })
    }
}

impl<C: Clone + Default> Debug for Grid<C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Grid({}x{})", self.width, self.height)
    }
}
