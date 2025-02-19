use emath::{Pos2, Vec2};

use crate::grid::GridCell;

use super::Grid;

#[derive(Clone)]
pub struct ScaledGrid<C: GridCell> {
    grid: Grid<C>,
    width: f32,
    height: f32,
    cell_size: f32,
}

impl<C: GridCell> ScaledGrid<C> {
    pub fn new(width: f32, height: f32, cell_size: f32) -> Self {
        let grid_width = (width / cell_size).ceil() as usize;
        let grid_height = (height / cell_size).ceil() as usize;

        Self {
            grid: Grid::new(grid_width, grid_height),
            width,
            height,
            cell_size,
        }
    }

    pub fn width(&self) -> f32 {
        self.width
    }

    pub fn height(&self) -> f32 {
        self.height
    }

    pub fn scale(&self) -> f32 {
        self.cell_size
    }

    pub fn grid(&self) -> &Grid<C> {
        &self.grid
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
        ((pos + self.size() / 2.0) / self.cell_size).floor()
    }

    pub fn grid_to_world(&self, pos: Pos2) -> Pos2 {
        (pos + Vec2::splat(0.5)) * self.cell_size - self.size() / 2.0
    }

    pub fn get_cell(&self, pos: Pos2) -> C {
        if pos.x < self.width / -2.0
            || pos.x > self.width / 2.0
            || pos.y < self.height / -2.0
            || pos.y > self.height / 2.0
        {
            return C::out_of_bounds();
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
        self.grid.get(x, y)
    }

    pub fn line(&mut self, start: Pos2, end: Pos2, width: f32, cell: C) {
        let width = width / self.cell_size;
        let start = self.world_to_grid(start);
        let end = self.world_to_grid(end);
        self.grid.line(start, end, width, cell);
    }

    pub fn circle(&mut self, center: Pos2, radius: f32, cell: C) {
        let radius = radius / self.cell_size;
        let center = self.world_to_grid(center);
        self.grid.circle(center, radius, cell);
    }
}
