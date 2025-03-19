mod description;
mod pgm;

use std::path::PathBuf;

use description::{BitmapDescription, ObjectDescription, WorldDescription};
use eframe::{egui::Color32, epaint::Hsva};

use botbrain::scaled_grid::ScaledGrid;

pub type World = ScaledGrid<Cell>;

#[derive(Clone, Copy, Default, PartialEq, Debug)]
pub enum Cell {
    #[default]
    Empty,
    Wall,
    SearchItem,
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
        }
    }
}

pub fn world_from_path(path: &PathBuf) -> Result<World, String> {
    println!("Loading world from {:?}", path);

    let desc = match path.extension().and_then(|ext| ext.to_str()) {
        Some("ron") => {
            let contents = std::fs::read_to_string(path)
                .map_err(|e| format!("Failed to read file '{}': {}", path.display(), e))?;

            let obj_desc = ron::from_str::<ObjectDescription>(&contents)
                .map_err(|e| format!("Failed to parse RON file {}: {}", path.display(), e))?;

            WorldDescription::Objs(obj_desc)
        }
        Some("yaml") => {
            let contents = std::fs::read_to_string(path)
                .map_err(|e| format!("Failed to read file '{}': {}", path.display(), e))?;

            let mut bitmap_desc = serde_yml::from_str::<BitmapDescription>(&contents)
                .map_err(|e| format!("Failed to parse YAML file {}: {}", path.display(), e))?;

            let image_path = path.with_file_name(&bitmap_desc.image);

            let image_bytes = std::fs::read(&image_path).map_err(|e| {
                format!("Failed to read image file {}: {}", image_path.display(), e)
            })?;

            bitmap_desc.bitmap = pgm::Parser::parse(image_bytes);

            WorldDescription::Bitmap(bitmap_desc)
        }
        _ => Err(format!("Unknown file type for {:?}", path))?,
    };

    Ok(desc.create())
}
