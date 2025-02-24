mod description;
mod pgm;

use std::{path::PathBuf, process::exit};

use description::{BitmapDescription, ObjectDescription, WorldDescription};
use eframe::{egui::Color32, epaint::Hsva};

use robcore::{grid::GridCell, scaled_grid::ScaledGrid};

pub type World = ScaledGrid<Cell>;

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

pub fn world_from_path(path: &PathBuf) -> World {
    println!("Loading world from {:?}", path);

    let desc = match path.extension().and_then(|ext| ext.to_str()) {
        Some("ron") => {
            let Ok(contents) = std::fs::read_to_string(path) else {
                eprintln!("Failed to read file {}", path.display());
                exit(1);
            };
            match ron::from_str::<ObjectDescription>(&contents) {
                Ok(obj_desc) => WorldDescription::Objs(obj_desc),
                Err(e) => {
                    eprintln!("Failed to parse RON file {}: {}", path.display(), e);
                    exit(1);
                }
            }
        }
        Some("yaml") => {
            let Ok(contents) = std::fs::read_to_string(path) else {
                eprintln!("Failed to read file {}", path.display());
                exit(1);
            };

            let mut bitmap_desc = match serde_yml::from_str::<BitmapDescription>(&contents) {
                Ok(bitmap_desc) => bitmap_desc,
                Err(e) => {
                    eprintln!("Failed to parse YAML file {}: {}", path.display(), e);
                    exit(1);
                }
            };

            let image_path = path.with_file_name(&bitmap_desc.image);

            let Ok(image_bytes) = std::fs::read(&image_path) else {
                eprintln!("Failed to read image file {}", image_path.display());
                exit(1);
            };

            bitmap_desc.bitmap = pgm::Parser::parse(image_bytes);

            WorldDescription::Bitmap(bitmap_desc)
        }
        _ => {
            eprintln!("Unknown file type for {:?}", path);
            exit(1);
        }
    };

    desc.create()
}
