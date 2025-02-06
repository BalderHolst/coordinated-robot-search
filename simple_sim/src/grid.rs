use eframe::{
    egui::{Color32, ColorImage, Pos2, Rgba},
    epaint::Hsva,
};

#[derive(Clone, Copy, Default, PartialEq)]
pub enum Cell {
    #[default]
    Empty,
    Wall,
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
            Self::OutOfBounds => Color32::TRANSPARENT,
        }
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
        self.cells
            .get(y * self.width + x)
            .copied()
            .unwrap_or(Cell::OutOfBounds)
    }

    pub fn set(&mut self, x: usize, y: usize, cell: Cell) {
        if let Some(inner) = self.cells.get_mut(y * self.width + x) {
            *inner = cell;
        }
    }

    pub fn is_empty(&self, x: usize, y: usize) -> bool {
        self.get(x, y).is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = (usize, usize, Cell)> + '_ {
        self.cells.iter().enumerate().map(|(i, &cell)| {
            let x = i % self.width;
            let y = i / self.width;
            (x, y, cell)
        })
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (usize, usize, &mut Cell)> + '_ {
        self.cells.iter_mut().enumerate().map(|(i, cell)| {
            let x = i % self.width;
            let y = i / self.width;
            (x, y, cell)
        })
    }

    pub fn line(&mut self, start: Pos2, end: Pos2, width: f32, cell: Cell) {
        let delta = end - start;
        let length = delta.length();
        for step in 0..=length as usize {
            let t = step as f32 / length;
            let pos = start + delta * t;
            self.circle(pos, width, cell);
        }
    }

    pub fn circle_iter(&self, center: Pos2, radius: f32) -> impl Iterator<Item = (usize, usize)>  {
        let radius2 = radius * radius;
        ((center.y - radius).ceil() as usize..=(center.y + radius).floor() as usize)
            .flat_map(move |y| {
                ((center.x - radius).ceil() as usize..=(center.x + radius).floor() as usize)
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

    pub fn circle(&mut self, center: Pos2, radius: f32, cell: Cell) {
        self.circle_iter(center, radius).for_each(|(x, y)| {
            self.set(x, y, cell);
        });
    }

    pub fn get_image(&self) -> ColorImage {
        let mut image = ColorImage::new([self.width, self.height], Cell::Empty.color());
        image.pixels.iter_mut().enumerate().for_each(|(i, pixel)| {
            let (x, y) = (i % self.width, i / self.width);
            *pixel = self.get(x, y).color();
        });
        image
    }
}
