use std::fmt::Debug;

use emath::{Pos2, Vec2};

use crate::{
    grid::{iter_circle, Grid},
    shapes::{Circle, Cone, Line, Shape, WideLine},
    utils,
};

/// A 2D grid of cells with a scale indexed by floating point values
#[derive(Clone)]
pub struct ScaledGrid<C: Clone + Default> {
    grid: Grid<C>,
    width: f32,
    height: f32,
    cell_size: f32,
}

impl<C: Clone + Default> Default for ScaledGrid<C> {
    fn default() -> Self {
        Self {
            grid: Grid::default(),
            width: 0.0,
            height: 0.0,
            cell_size: 1.0,
        }
    }
}

impl<C: Clone + Default> ScaledGrid<C> {
    /// Create a new grid with the given width, height, and the size of each cell
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

    /// Create a new grid from a grid and the size of each cell
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

    /// Width of the grid
    pub fn width(&self) -> f32 {
        self.width
    }

    /// Height of the grid
    pub fn height(&self) -> f32 {
        self.height
    }

    /// Size of each cell
    pub fn scale(&self) -> f32 {
        self.cell_size
    }

    /// Size of the grid
    pub fn size(&self) -> Vec2 {
        Vec2::new(self.width, self.height)
    }

    /// Bounds of the grid
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

    /// The underlying grid
    pub fn grid(&self) -> &Grid<C> {
        &self.grid
    }

    /// Convert a world position to the underlying grid position
    pub fn world_to_grid(&self, pos: Pos2) -> Pos2 {
        ((pos + self.size() / 2.0) / self.cell_size).floor()
    }

    /// Convert an underlying grid position to a world position
    pub fn grid_to_world(&self, pos: Pos2) -> Pos2 {
        (pos + Vec2::splat(0.5)) * self.cell_size - self.size() / 2.0
    }

    /// Get the cell at the given position. Returns `None` if the position is out of bounds.
    pub fn get(&self, pos: Pos2) -> Option<&C> {
        if pos.x < self.width / -2.0
            || pos.x > self.width / 2.0
            || pos.y < self.height / -2.0
            || pos.y > self.height / 2.0
        {
            return None;
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

    /// Set the cell at the given position
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

    /// Set the cells in a [Shape] to a value
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

    /// Set the cells in a line to a value
    pub fn line(&mut self, start: Pos2, end: Pos2, width: f32, cell: C) {
        let width = width / self.cell_size;
        let start = self.world_to_grid(start);
        let end = self.world_to_grid(end);
        self.grid.line(start, end, width, cell);
    }

    /// Set the cells in a circle to a value
    pub fn circle(&mut self, center: Pos2, radius: f32, cell: C) {
        let radius = radius / self.cell_size;
        let center = self.world_to_grid(center);
        self.grid.circle(center, radius, cell);
    }

    /// Iterate over cells within a [Circle]
    pub fn iter_circle(&self, circle: &Circle) -> impl Iterator<Item = (Pos2, Option<&C>)> + '_ {
        let Circle { center, radius } = circle;
        let radius = radius / self.cell_size;
        let center = self.world_to_grid(*center);
        iter_circle(center, radius).map(move |(x, y)| {
            let pos = self.grid_to_world(Pos2::new(x as f32, y as f32));
            let cell = self.grid.get(x, y);
            (pos, cell)
        })
    }

    /// Iterate over cells within a [Cone]
    pub fn iter_cone(&self, cone: &Cone) -> impl Iterator<Item = (Pos2, Option<&C>)> + '_ {
        let Cone {
            center,
            radius,
            angle,
            fov,
        } = cone.clone();
        self.iter_circle(&Circle { center, radius })
            .filter(move |(point, _cell)| {
                let offset = *point - center;
                let angle = offset.angle() - angle;
                let angle = utils::normalize_angle(angle);
                angle.abs() < fov / 2.0
            })
    }
}

impl<C: Clone + Default> Debug for ScaledGrid<C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "ScaledGrid({}x{} at {})",
            self.width, self.height, self.cell_size
        )
    }
}
