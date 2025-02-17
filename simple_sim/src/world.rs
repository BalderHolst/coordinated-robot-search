use eframe::{
    egui::{Color32, Pos2, Vec2},
    epaint::Hsva,
};

use robcore::Grid;

pub const CELLS_PR_METER: f32 = 50.0;

#[derive(Clone, Copy, Default, PartialEq)]
pub enum Cell {
    #[default]
    Empty,
    Wall,
    SearchItem,
    OutOfBounds,
}

impl Cell {
    pub fn is_empty(&self) -> bool {
        matches!(self, Self::Empty)
    }

    pub fn color(&self) -> Color32 {
        match self {
            Self::Empty => Color32::TRANSPARENT,
            Self::Wall => Hsva::new(0.6, 0.7, 0.5, 1.0).into(),
            Self::SearchItem => Hsva::new(0.22, 0.8, 0.8, 1.0).into(),
            Self::OutOfBounds => Color32::TRANSPARENT,
        }
    }
}

#[derive(Clone)]
pub struct World {
    grid: Grid<Cell>,
    width: f32,
    height: f32,
}

impl World {
    pub fn new(width: f32, height: f32) -> Self {
        let grid_width = (width * CELLS_PR_METER).ceil() as usize;
        let grid_height = (height * CELLS_PR_METER).ceil() as usize;

        Self {
            grid: Grid::new(grid_width, grid_height),
            width,
            height,
        }
    }

    pub fn size(&self) -> Vec2 {
        Vec2::new(self.width, self.height)
    }

    pub fn bounds(&self) -> (Pos2, Pos2) {
        let min = Pos2 {
            x: -self.width / 2.0,
            y: -self.height / 2.0,
        };
        let max = Pos2 {
            x: self.width / 2.0,
            y: self.height / 2.0,
        };
        (min, max)
    }

    pub fn world_to_grid(&self, pos: Pos2) -> Pos2 {
        ((pos + self.size() / 2.0) * CELLS_PR_METER).floor()
    }

    pub fn grid_to_world(&self, pos: Pos2) -> Pos2 {
        (pos + Vec2::splat(0.5)) / CELLS_PR_METER - self.size() / 2.0
    }

    pub fn get_cell(&self, pos: Pos2) -> Cell {
        if pos.x < self.width / -2.0
            || pos.x > self.width / 2.0
            || pos.y < self.height / -2.0
            || pos.y > self.height / 2.0
        {
            return Cell::OutOfBounds;
        }
        let pos = self.world_to_grid(pos);
        debug_assert!(pos.x >= 0.0, "x: {}", pos.x);
        debug_assert!(pos.y >= 0.0, "y: {}", pos.y);
        let x = pos.x as usize;
        let y = pos.y as usize;
        debug_assert!(
            x <= self.grid.width(),
            "x: {}, width: {}",
            x,
            self.grid.width()
        );
        debug_assert!(
            y <= self.grid.height(),
            "y: {}, height: {}",
            y,
            self.grid.height()
        );
        self.grid.get(x as i64, y as i64)
    }

    pub fn width(&self) -> f32 {
        self.width
    }

    pub fn height(&self) -> f32 {
        self.height
    }

    pub fn grid(&self) -> &Grid<Cell> {
        &self.grid
    }

    pub fn line(&mut self, start: Pos2, end: Pos2, width: f32, cell: Cell) {
        let width = width * CELLS_PR_METER;
        let start = self.world_to_grid(start);
        let end = self.world_to_grid(end);
        self.grid.line(start, end, width, cell);
    }

    pub fn circle(&mut self, center: Pos2, radius: f32, cell: Cell) {
        let radius = radius * CELLS_PR_METER;
        let center = self.world_to_grid(center);
        self.grid.circle(center, radius, cell);
    }
}
