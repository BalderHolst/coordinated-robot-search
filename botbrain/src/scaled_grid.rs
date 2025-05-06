//! A 2D grid of cells with a scale indexed by floating point values

use std::fmt::Debug;

use emath::{Pos2, Vec2};

use crate::{
    grid::Grid,
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
        Self::empty()
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

    /// Create an empty grid of zero size
    pub fn empty() -> Self {
        Self {
            grid: Grid::empty(),
            width: 0.0,
            height: 0.0,
            cell_size: 1.0,
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

    /// The underlying grid mutable
    pub fn grid_mut(&mut self) -> &mut Grid<C> {
        &mut self.grid
    }

    /// Convert a scaled grid position to the underlying grid position
    pub fn pos_to_grid(&self, pos: Pos2) -> Pos2 {
        ((pos + self.size() / 2.0) / self.cell_size).floor()
    }

    /// Convert an underlying grid position to a scaled grid position
    pub fn grid_to_pos(&self, pos: Pos2) -> Pos2 {
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
        let pos = self.pos_to_grid(pos);
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
        let pos = self.pos_to_grid(pos);
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

    /// Iterate over the cells in the grid
    pub fn iter(&self) -> impl Iterator<Item = (f32, f32, &C)> {
        self.grid.iter().map(move |(x, y, cell)| {
            let x = x as f32 * self.cell_size - self.width / 2.0;
            let y = y as f32 * self.cell_size - self.height / 2.0;
            (x, y, cell)
        })
    }

    /// Iterate over the cells in the grid *mutably*
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (f32, f32, &mut C)> + '_ {
        self.grid.iter_mut().map(|(x, y, cell)| {
            let x = x as f32 * self.cell_size - self.width / 2.0;
            let y = y as f32 * self.cell_size - self.height / 2.0;
            (x, y, cell)
        })
    }

    /// Fill the grid with a value
    pub fn fill(&mut self, cell: C) {
        self.grid.fill(cell);
    }

    /// Set the cells in a [Shape] to a value
    pub fn set_shape(&mut self, shape: &Shape, cell: C) {
        match shape {
            Shape::Circle(Circle { center, radius }) => self.set_circle(*center, *radius, cell),
            Shape::Line(Line { start, end }) => self.set_line(*start, *end, self.cell_size, cell),
            Shape::WideLine(WideLine {
                line: Line { start, end },
                width,
            }) => self.set_line(*start, *end, *width, cell),
            Shape::Cone(cone) => self.set_cone(cone, cell),
        }
    }

    /// Set the cells in a line to a value
    pub fn set_line(&mut self, start: Pos2, end: Pos2, width: f32, cell: C) {
        let width = width / self.cell_size;
        let start = self.pos_to_grid(start);
        let end = self.pos_to_grid(end);
        self.grid.set_line(start, end, width, cell);
    }

    /// Set the cells in a circle to a value
    pub fn set_circle(&mut self, center: Pos2, radius: f32, cell: C) {
        let radius = radius / self.cell_size;
        let center = self.pos_to_grid(center);
        self.grid.set_circle(center, radius, cell);
    }

    /// Set a cone of cells to a value
    pub fn set_cone(&mut self, cone: &Cone, cell: C) {
        let Cone {
            center,
            radius,
            angle,
            fov,
        } = *cone;
        let radius = radius / self.cell_size;
        let center = self.pos_to_grid(center);
        self.grid.set_cone(center, radius, angle, fov, cell);
    }

    /// Iterate over cells within a [Circle]
    pub fn iter_circle(&self, circle: &Circle) -> impl Iterator<Item = Pos2> + '_ {
        let Circle { center, radius } = circle;
        let radius = radius / self.cell_size;
        let center = self.pos_to_grid(*center);
        self.grid
            .iter_circle(center, radius)
            .map(move |(x, y)| self.grid_to_pos(Pos2::new(x as f32, y as f32)))
    }

    /// Iterate over cells within a [Cone]
    pub fn iter_cone(&self, cone: &Cone) -> impl Iterator<Item = Pos2> + '_ {
        let Cone {
            center,
            radius,
            angle,
            fov,
        } = *cone;
        self.iter_circle(&Circle { center, radius })
            .filter(move |point| {
                let offset = *point - center;
                let angle = offset.angle() - angle;
                let angle = utils::normalize_angle(angle);
                angle.abs() < fov / 2.0
            })
    }

    /// Iterate over cells within a [Line]
    pub fn iter_line(&self, line: &Line) -> impl Iterator<Item = Pos2> + '_ {
        let Line { start, end } = line;
        let start = self.pos_to_grid(*start);
        let end = self.pos_to_grid(*end);
        self.grid
            .iter_line(start, end)
            .map(move |(x, y)| self.grid_to_pos(Pos2::new(x as f32, y as f32)))
    }

    /// Maskes out the ScaledGrid cells
    /// If mask returns true, the cell is kept
    /// If not the cell is set to default
    pub fn mask(&mut self, keep: impl Fn(Pos2) -> bool) {
        self.iter_mut().for_each(|(x, y, cell)| {
            if !keep(Pos2 { x, y }) {
                *cell = C::default()
            }
        });
    }

    /// Transform the grid to a new grid with a different type
    pub fn transform<T: Clone + Default>(self, f: impl Fn(C) -> T) -> ScaledGrid<T> {
        let mut new_grid = Grid::with_size(self.grid().size());
        let old_grid = self.grid;

        for (old, new) in old_grid
            .into_cells()
            .into_iter()
            .zip(new_grid.cells_mut().iter_mut())
        {
            *new = f(old);
        }

        ScaledGrid {
            grid: new_grid,
            width: self.width,
            height: self.height,
            cell_size: self.cell_size,
        }
    }

    /// Cast a ray from a position in a given direction
    pub fn cast_ray(
        &self,
        from: Pos2,
        angle: f32,
        max_range: f32,
        should_stop: impl Fn(&C) -> bool,
    ) -> RayCastResult<C> {
        let step_size = self.scale() * 0.5;
        let direction = Vec2::angled(angle);

        let mut distance = 0.0;

        while distance < max_range {
            let pos = from + direction * distance;

            let cell = self.get(pos);

            // Check for collisions with the grid
            if let Some(cell) = cell {
                if should_stop(cell) {
                    return RayCastResult::Hit(distance, cell);
                }
            } else {
                return RayCastResult::OutOfBounds(distance);
            }

            distance += step_size;
        }

        RayCastResult::OutOfRange(max_range)
    }
}

/// The result of a ray cast
pub enum RayCastResult<'a, C> {
    /// The ray hit an object
    Hit(f32, &'a C),

    /// The ray reached the maximum range
    OutOfRange(f32),

    /// The ray reached outside the bounds of the grid
    OutOfBounds(f32),
}

impl<C> RayCastResult<'_, C> {
    /// Get the distance of the ray cast
    pub fn distance(&self) -> f32 {
        match self {
            RayCastResult::Hit(distance, _)
            | RayCastResult::OutOfRange(distance)
            | RayCastResult::OutOfBounds(distance) => *distance,
        }
    }
}

fn linspace(start: f32, end: f32, n: usize) -> impl Iterator<Item = f32> {
    let step = (end - start) / (n as f32 - 1.0);
    (0..n).map(move |i| start + step * i as f32)
}

impl ScaledGrid<f32> {
    /// Add another grid to this grid by sampling the other grid at each cell in this grid
    pub fn add_sampled_grid(&mut self, other: &ScaledGrid<f32>, weight: f32) {
        let grid = &mut self.grid;
        for (grid_y, y) in
            linspace(-self.height / 2.0, self.height / 2.0, grid.height()).enumerate()
        {
            for (grid_x, x) in
                linspace(-self.width / 2.0, self.width / 2.0, grid.width()).enumerate()
            {
                let Some(other_value) = other.get(Pos2::new(x, y)) else {
                    continue;
                };
                if let Some(value) = grid.get_mut(grid_x, grid_y) {
                    *value += other_value * weight;
                }
            }
        }
    }

    /// Create a new grid from a stack of grids
    pub fn from_stack(stack: &[&Self], width: f32, height: f32, cell_size: f32) -> Self {
        let mut grid = Self::new(width, height, cell_size);
        stack.iter().for_each(|&other| {
            grid.add_sampled_grid(other, 1.0);
        });
        grid
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_position_conversions() {
        let grid = ScaledGrid::<f32>::new(10.0, 10.0, 1.0);
        let world_pos = Pos2::new(1.5, 1.5);
        let grid_pos = grid.pos_to_grid(world_pos);
        let world_pos2 = grid.grid_to_pos(grid_pos);

        assert_eq!(grid_pos, Pos2::new(6.0, 6.0));
        assert_eq!(world_pos, world_pos2);
    }

    #[test]
    fn test_grid_addition() {
        let mut grid1 = ScaledGrid::new(10.0, 10.0, 1.0);
        grid1.fill(1.0);

        let grid2 = grid1.clone();

        grid1.add_sampled_grid(&grid2, 1.0);

        println!("{:?}", grid1.grid().get(2, 2));
        println!("{:?}", grid2.grid().get(2, 2));

        assert_eq!(grid1.grid().get(0, 0), Some(&2.0));
    }
}
