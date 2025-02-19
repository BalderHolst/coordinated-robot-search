use eframe::{egui::Color32, epaint::Hsva};

use robcore::{grid::GridCell, scaled_grid::ScaledGrid};

pub const CELLS_PR_METER: f32 = 50.0;

#[derive(Clone, Copy, Default, PartialEq, Debug)]
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

impl GridCell for Cell {
    fn out_of_bounds() -> Self {
        Self::OutOfBounds
    }
}

pub type World = ScaledGrid<Cell>;
