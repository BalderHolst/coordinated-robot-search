pub mod description;
mod pgm;

use std::path::PathBuf;

use description::{BitmapDescription, ObjectDescription, WorldDescription};
use eframe::{
    egui::{Color32, ColorImage},
    epaint::Hsva,
};

use botbrain::{scaled_grid::ScaledGrid, Pos2};

use crate::{gui::Theme, utils};

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

    pub fn color(&self, theme: Theme) -> Color32 {
        match self {
            Self::Empty => theme.world_background(),
            Self::Wall => Hsva::new(0.6, 0.7, 0.5, 1.0).into(),
            Self::SearchItem => Hsva::new(0.22, 0.8, 0.8, 1.0).into(),
        }
    }
}

impl From<Cell> for botbrain::MapCell {
    fn from(value: Cell) -> Self {
        match value {
            Cell::Empty => botbrain::MapCell::Free,
            Cell::Wall => botbrain::MapCell::Obstacle,
            Cell::SearchItem => botbrain::MapCell::Free,
        }
    }
}

pub fn convert_to_botbrain_map(world: &World) -> botbrain::Map {
    let mut map =
        ScaledGrid::<botbrain::MapCell>::new(world.width(), world.height(), world.scale());
    for (x, y, &cell) in world.iter() {
        map.set(Pos2::new(x, y), cell.into());
    }
    map
}

pub fn desc_from_path(path: &PathBuf) -> Result<WorldDescription, String> {
    println!("Loading world from {:?}", path);

    let contents = || {
        std::fs::read_to_string(path)
            .map_err(|e| format!("Failed to read file '{}': {}", path.display(), e))
    };

    match path.extension().and_then(|ext| ext.to_str()) {
        Some("ron") => {
            let obj_desc = ron::from_str::<ObjectDescription>(&contents()?)
                .map_err(|e| format!("Failed to parse RON file {}: {}", path.display(), e))?;
            Ok(WorldDescription::Objs(obj_desc))
        }
        Some("json") => {
            let obj_desc = serde_json::from_str::<ObjectDescription>(&contents()?)
                .map_err(|e| format!("Failed to parse JSON file {}: {}", path.display(), e))?;
            Ok(WorldDescription::Objs(obj_desc))
        }
        Some("yaml") => {
            let mut bitmap_desc = serde_yml::from_str::<BitmapDescription>(&contents()?)
                .map_err(|e| format!("Failed to parse YAML file {}: {}", path.display(), e))?;

            let image_path = path.with_file_name(&bitmap_desc.image);

            let image_bytes = std::fs::read(&image_path).map_err(|e| {
                format!("Failed to read image file {}: {}", image_path.display(), e)
            })?;

            bitmap_desc.bitmap = pgm::Parser::parse(image_bytes);

            Ok(WorldDescription::Bitmap(bitmap_desc))
        }
        _ => Err(format!("Unknown file type for {:?}", path))?,
    }
}

pub fn world_from_path(path: &PathBuf) -> Result<World, String> {
    let desc = desc_from_path(path)?;
    Ok(desc.create())
}

pub fn world_to_image(world: &World, theme: Theme) -> ColorImage {
    utils::grid_to_image(world.grid(), |c| c.color(theme))
}
