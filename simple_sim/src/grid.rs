use std::default;

#[derive(Clone, Copy, Default, PartialEq)]
pub enum Cell {
    #[default]
    Empty,
    Wall,
    Object,
    OutOfBounds,
}

impl Cell {
    pub fn is_empty(&self) -> bool {
        matches!(self, Self::Empty)
    }
}

pub struct Grid {
    cells: Vec<Cell>,
    width: usize,
    height: usize,
}

impl Grid {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            cells: vec![Cell::default(); width * height],
            width,
            height,
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn get(&self, x: usize, y: usize) -> Cell {
        self.cells.get(y * self.width + x).copied().unwrap_or(Cell::OutOfBounds)
    }

    pub fn set(&mut self, x: usize, y: usize, cell: Cell) {
        self.cells[y * self.width + x] = cell;
    }

    pub fn is_empty(&self, x: usize, y: usize) -> bool {
        self.get(x, y).is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = (usize, usize, Cell)> + '_ {
        self.cells.iter().enumerate().map(move |(i, &cell)| {
            let x = i % self.width;
            let y = i / self.width;
            (x, y, cell)
        })
    }
}
