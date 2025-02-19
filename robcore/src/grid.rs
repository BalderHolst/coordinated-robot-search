use std::fmt::Debug;

use emath::Pos2;

#[derive(Clone)]
pub struct Grid<C> {
    cells: Vec<C>,
    width: usize,
    height: usize,
}

impl<C> Default for Grid<C> {
    fn default() -> Self {
        Self {
            cells: vec![],
            width: 0,
            height: 0,
        }
    }
}

impl<C: Clone + Copy + Default> Grid<C> {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
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

    pub fn get(&self, x: usize, y: usize) -> C {
        self.cells
            .get(y * self.width + x)
            .cloned()
            .unwrap_or_default()
    }

    pub fn set(&mut self, x: usize, y: usize, cell: C) {
        if let Some(inner) = self.cells.get_mut(y * self.width + x) {
            *inner = cell;
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = (usize, usize, C)> + '_ {
        self.cells.iter().enumerate().map(|(i, &cell)| {
            let x = i % self.width;
            let y = i / self.width;
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

    pub fn circle_iter(&self, center: Pos2, radius: f32) -> impl Iterator<Item = (usize, usize)> {
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

    pub fn circle(&mut self, center: Pos2, radius: f32, cell: C) {
        self.circle_iter(center, radius).for_each(|(x, y)| {
            self.set(x, y, cell);
        });
    }
}

impl<C> Debug for Grid<C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Grid({}x{})", self.width, self.height)
    }
}
