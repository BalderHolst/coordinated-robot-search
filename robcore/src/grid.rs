use std::fmt::Debug;

use emath::Pos2;

/// A 2D grid of cells
#[derive(Clone)]
pub struct Grid<C: GridCell> {
    cells: Vec<C>,
    width: usize,
    height: usize,
}

impl<C: GridCell> Default for Grid<C> {
    fn default() -> Self {
        Self {
            cells: vec![],
            width: 0,
            height: 0,
        }
    }
}

impl<C: GridCell> Grid<C> {
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

    /// Get the cell at the given position. Returns `None` if the position is out of bounds.
    pub fn get(&self, x: usize, y: usize) -> Option<C> {
        if x >= self.width || y >= self.height {
            return None;
        }
        self.cells.get(y * self.width + x).cloned()
    }

    /// Set the cell at the given position
    pub fn set(&mut self, x: usize, y: usize, cell: C) {
        if let Some(inner) = self.cells.get_mut(y * self.width + x) {
            *inner = cell;
        }
    }

    /// Iterate over all cells in the grid
    pub fn iter(&self) -> impl Iterator<Item = (usize, usize, C)> + '_ {
        self.cells.iter().enumerate().map(|(i, &cell)| {
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

    /// Draw a line of cells on the grid
    pub fn line(&mut self, start: Pos2, end: Pos2, width: f32, cell: C) {
        let delta = end - start;
        let length = delta.length();
        for step in 0..=length as usize {
            let t = step as f32 / length;
            let pos = start + delta * t;
            self.circle(pos, width, cell);
        }
    }

    /// Draw a circle of cells on the grid
    pub fn circle(&mut self, center: Pos2, radius: f32, cell: C) {
        iter_circle(center, radius).for_each(|(x, y)| {
            self.set(x, y, cell);
        });
    }
}

/// Iterate over coordinates within a circle
pub fn iter_circle(center: Pos2, radius: f32) -> impl Iterator<Item = (usize, usize)> {
    let radius2 = radius * radius;
    ((center.y - radius).ceil() as usize..=(center.y + radius).floor() as usize).flat_map(
        move |y| {
            ((center.x - radius).ceil() as usize..=(center.x + radius).floor() as usize)
                .filter(move |x| {
                    let pos = Pos2 {
                        x: *x as f32,
                        y: y as f32,
                    };
                    (pos - center).length().powf(2.0) <= radius2
                })
                .map(move |x| (x, y))
        },
    )
}

impl<C: GridCell> Debug for Grid<C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Grid({}x{})", self.width, self.height)
    }
}

pub trait GridCell: Clone + Copy + Default {
    // Empty
}

macro_rules! impl_grid_cell {
    ($($t:ty),*) => { $(impl GridCell for $t {})* };
}

impl_grid_cell![
    (),
    bool,
    u8,
    u16,
    u32,
    u64,
    u128,
    i8,
    i16,
    i32,
    i64,
    i128,
    f32,
    f64
];
