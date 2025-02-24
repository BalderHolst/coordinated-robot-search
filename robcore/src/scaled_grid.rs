use std::fmt::Debug;

use emath::{Pos2, Vec2};

use crate::{
    grid::{iter_circle, Grid, GridCell},
    shapes::{Circle, Line, Shape, WideLine},
};

#[derive(Clone)]
pub struct ScaledGrid<C: GridCell> {
    grid: Grid<C>,
    width: f32,
    height: f32,
    cell_size: f32,
}

impl<C: GridCell> Default for ScaledGrid<C> {
    fn default() -> Self {
        Self {
            grid: Grid::default(),
            width: 0.0,
            height: 0.0,
            cell_size: 1.0,
        }
    }
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

    pub fn from_grid(grid: Grid<C>, cell_size: f32) -> Self {
        let width = grid.width() as f32 * cell_size;
        let height = grid.height() as f32 * cell_size;
        Self {
            grid,
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

    pub fn get(&self, pos: Pos2) -> C {
        if pos.x < self.width / -2.0
            || pos.x > self.width / 2.0
            || pos.y < self.height / -2.0
            || pos.y > self.height / 2.0
        {
            return C::out_of_bounds();
        }
        let pos = self.world_to_grid(pos);
        debug_assert!(pos.x >= 0.0);
        debug_assert!(pos.y >= 0.0);
        let x = pos.x as usize;
        let y = pos.y as usize;
        debug_assert!(x <= self.grid.width());
        debug_assert!(y <= self.grid.height());
        self.grid.get(x, y)
    }

    pub fn set(&mut self, pos: Pos2, cell: C) {
        if pos.x < self.width / -2.0
            || pos.x > self.width / 2.0
            || pos.y < self.height / -2.0
            || pos.y > self.height / 2.0
        {
            return;
        }
        let pos = self.world_to_grid(pos);
        debug_assert!(pos.x >= 0.0);
        debug_assert!(pos.y >= 0.0);
        let x = pos.x as usize;
        let y = pos.y as usize;
        debug_assert!(x <= self.grid.width());
        debug_assert!(y <= self.grid.height());
        if x < self.grid.width() && y < self.grid.height() {
            self.grid.set(x, y, cell);
        }
    }

    pub fn shape(&mut self, shape: &Shape, cell: C) {
        match shape {
            Shape::Circle(Circle { center, radius }) => self.circle(*center, *radius, cell),
            Shape::Line(Line { start, end }) => self.line(*start, *end, self.cell_size, cell),
            Shape::WideLine(WideLine { start, end, width }) => {
                self.line(*start, *end, *width, cell)
            }
            Shape::Cone(_cone) => todo!(),
        }
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

    pub fn iter_circle(&self, center: Pos2, radius: f32) -> impl Iterator<Item = (Pos2, C)> + '_ {
        let radius = radius / self.cell_size;
        let center = self.world_to_grid(center);
        iter_circle(center, radius).map(move |(x, y)| {
            let pos = self.grid_to_world(Pos2::new(x as f32, y as f32));
            (pos, self.grid.get(x, y))
        })
    }
}

impl<C: GridCell> Debug for ScaledGrid<C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "ScaledGrid({}x{} at {})",
            self.width, self.height, self.cell_size
        )
    }
}
