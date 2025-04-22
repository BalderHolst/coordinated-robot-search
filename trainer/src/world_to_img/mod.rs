use std::{fs, path::PathBuf};

use crate::cli::WorldToImgArgs;

const DEFAULT_IMG_EXT: &str = "png";

pub fn world_to_img(args: WorldToImgArgs) -> Result<(), String> {
    let input = args.input;
    let output = args.output;

    if let Ok(entries) = input.read_dir() {
        if output.is_file() {
            return Err("Output path is a file, but input is a directory.".into());
        }

        _ = fs::create_dir_all(&output);

        assert!(output.is_dir());

        for file in entries {
            let Ok(file) = file.map_err(|e| format!("Failed to read directory entry: {}", e))
            else {
                continue;
            };
            let input_file_path = file.path();
            let output_file_path = output
                .join(file.file_name())
                .with_extension(DEFAULT_IMG_EXT);
            world_to_img_file(&input_file_path, output_file_path, args.force, args.theme)?;
        }
    } else if input.is_file() {
        if output.is_dir() {
            return Err(format!(
                "Output path is a directory, but input is a file: {}.",
                input.display()
            ));
        }
        world_to_img_file(&input, output, args.force, args.theme)?;
    } else {
        return Err(format!(
            "Input file is not a valid directory or file: {}",
            input.display()
        ));
    }

    Ok(())
}

fn world_to_img_file(
    input: &PathBuf,
    output: PathBuf,
    force: bool,
    theme: simple_sim::gui::Theme,
) -> Result<(), String> {
    if !input.exists() {
        return Err(format!("Input file not found: {}", input.display()));
    }

    if output.exists() && !force {
        return Err(format!(
            "File '{}' already exists. Use --force to overwrite.",
            output.display()
        ));
    }

    if let Some(dir) = output.parent() {
        _ = fs::create_dir_all(dir)
    }

    let world = simple_sim::world::world_from_path(input)?;
    let img = render_world(&world, theme);

    img.save(&output)
        .map_err(|e| format!("Failed to save image to '{}': {}", output.display(), e))?;

    Ok(())
}

fn render_world(
    world: &simple_sim::world::World,
    theme: simple_sim::gui::Theme,
) -> image::RgbaImage {
    let grid = world.grid().clone();

    let w = grid.width() as u32;
    let h = grid.height() as u32;

    let mut img = image::RgbaImage::new(w, h);

    for (i, pixel) in img.pixels_mut().enumerate() {
        let img_x = i % w as usize;
        let img_y = i / w as usize;

        let grid_x = img_x;
        let grid_y = img_y;

        let mut color = grid
            .get(grid_x, grid_y)
            .map(|cell| {
                let cell_color = cell.color(theme);
                cell_color.to_array()
            })
            .unwrap_or([u8::MAX, 0, 0, u8::MAX]);

        color[3] = u8::MAX;

        *pixel = image::Rgba(color);
    }

    img
}
