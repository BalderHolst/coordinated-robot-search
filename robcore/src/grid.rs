use std::fmt::Debug;

use emath::Pos2;

type Coord = i64;

#[derive(Clone)]
pub struct Grid<C> {
    offset: (usize, usize),
    cells: Vec<C>,
    width: usize,
    height: usize,
}

impl<C: Clone + Copy + Default> Grid<C> {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            offset: (0, 0),
            cells: vec![C::default(); width * height],
            width,
            height,
        }
    }

    fn with_offset(width: usize, height: usize, offset_x: usize, offset_y: usize) -> Self {
        Self {
            offset: (offset_x, offset_y),
            cells: vec![C::default(); width * height],
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

    pub fn bounds(&self) -> (Coord, Coord, Coord, Coord) {
        (
            -(self.offset.0 as Coord),
            -(self.offset.1 as Coord),
            -(self.offset.0 as Coord) + self.width as Coord,
            -(self.offset.1 as Coord) + self.height as Coord,
        )
    }

    fn get_index(&self, x: Coord, y: Coord) -> Option<usize> {
        let (xo, yo) = self.offset;

        if x < -(xo as Coord) || y < -(yo as Coord) {
            return None;
        }

        let xi = (x + xo as Coord) as usize;
        let yi = (y + yo as Coord) as usize;

        if xi >= self.width || yi >= self.height {
            return None;
        }

        let idx = yi * self.width + xi;

        debug_assert!(idx < self.cells.len());

        Some(idx)
    }

    pub fn get(&self, x: Coord, y: Coord) -> C {
        let Some(idx) = self.get_index(x, y) else {
            return C::default();
        };

        self.cells.get(idx).cloned().unwrap_or_default()
    }

    pub fn set(&mut self, x: Coord, y: Coord, cell: C) {
        let (min_x, min_y, max_x, max_y) = self.bounds();

        if x < min_x || y < min_y {
            let (xo, yo) = self.offset;
            let x_diff = min_x - x;
            let y_diff = min_y - y;

            debug_assert!(x_diff >= 0);
            debug_assert!(y_diff >= 0);

            let mut new_grid = Self::with_offset(
                self.width + x_diff as usize,
                self.height + y_diff as usize,
                xo + x_diff as usize,
                yo + y_diff as usize,
            );
            for (x, y, cell) in self.iter() {
                new_grid.set(x, y, cell);
            }
            *self = new_grid;
        }

        if x >= max_x {
            let diff = (x - max_x + 1) as usize;
            let (xo, yo) = self.offset;
            let mut new_grid = Self::with_offset(self.width + diff as usize, self.height, xo, yo);
            for (x, y, cell) in self.iter() {
                new_grid.set(x, y, cell);
            }
            *self = new_grid;
        }

        if y >= max_y {
            let diff = (y - max_y + 1) as usize;
            self.height += diff;
            self.cells.extend(vec![C::default(); self.width * diff]);
        }

        let idx = self
            .get_index(x, y)
            .expect("Grid should have grown to include the cell");

        self.cells[idx] = cell;
    }

    pub fn iter(&self) -> impl Iterator<Item = (Coord, Coord, C)> + '_ {
        self.cells.iter().enumerate().map(|(i, &cell)| {
            let (xo, yo) = self.offset;
            let x = (i % self.width) as Coord - xo as Coord;
            let y = (i / self.width) as Coord - yo as Coord;
            (x, y, cell)
        })
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (usize, usize, &mut C)> + '_ {
        self.cells.iter_mut().enumerate().map(|(i, cell)| {
            let x = i % self.width;
            let y = i / self.width;
            (x, y, cell)
        })
    }

    pub fn line(&mut self, start: Pos2, end: Pos2, width: f32, cell: C) {
        let delta = end - start;
        let length = delta.length();
        for step in 0..=length as usize {
            let t = step as f32 / length;
            let pos = start + delta * t;
            self.circle(pos, width, cell);
        }
    }

    pub fn circle_iter(&self, center: Pos2, radius: f32) -> impl Iterator<Item = (Coord, Coord)> {
        let radius2 = radius * radius;
        let (xo, yo) = self.offset;
        ((center.y - radius).ceil() as Coord..=(center.y + radius).floor() as Coord).flat_map(
            move |y| {
                ((center.x - radius).ceil() as Coord..=(center.x + radius).floor() as Coord)
                    .filter(move |x| {
                        let pos = Pos2 {
                            x: *x as f32,
                            y: y as f32,
                        };
                        (pos - center).length().powf(2.0) <= radius2
                    })
                    .map(move |x| (x - (xo as Coord), y - (yo as Coord)))
            },
        )
    }

    pub fn circle(&mut self, center: Pos2, radius: f32, cell: C) {
        self.circle_iter(center, radius).for_each(|(x, y)| {
            self.set(x, y, cell);
        });
    }
}

macro_rules! impl_debug_for {
    ($ty:ident; $w:expr => $fmt:expr) => {
        impl Debug for Grid<$ty> {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                let mut s = String::new();
                let (xo, yo) = self.offset;
                for y in 0..self.height {
                    for x in 0..self.width {
                        let cell = self.get(x as Coord - xo as Coord, y as Coord - yo as Coord);
                        s.extend($fmt(cell).to_string().chars());
                    }
                    s.push('\n');
                }
                write!(f, "{}", s)
            }
        }
    };
}

macro_rules! impl_debug_for_numbers {
    [$($ty:ident)+] => {
        $(impl_debug_for!($ty; 3 => |cell| format!("{:3}", cell));)+
    }
}

impl_debug_for_numbers![u8 i8 u16 i16 u32 i32 u64 i64 usize isize];
impl_debug_for!(bool; 2 => |cell| if cell { "##" } else { "~~" });

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_grid() {
        let mut grid = Grid::new(10, 10);

        assert_eq!(grid.get(5, 5), false);

        grid.set(0, 0, true);

        assert_eq!(grid.get(0, 0), true);

        println!("{:?}", grid);

        grid.set(-5, 0, true);
        assert_eq!(grid.get(-5, 0), true);

        println!("{:?}", grid);

        grid.set(0, 11, true);
        assert_eq!(grid.get(0, 11), true);

        println!("{:?}", grid);

        grid.set(15, 11, true);
        assert_eq!(grid.get(15, 11), true);

        println!("{:?}", grid);

        assert_eq!(grid.get(0, 0), true);
    }
}
