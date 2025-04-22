use std::{fs, ops::Range};

use botbrain::{
    shapes::{Circle, Line, Shape, WideLine},
    Pos2, Vec2,
};
use rand::Rng;
use simple_sim::world::description::ObjectDescription;

use crate::cli::{WorldGenArgs, WorldToImgArgs};

pub fn world_gen(args: WorldGenArgs) -> Result<(), String> {
    let out_dir = args.output;

    // Check for existing files in the output directory
    if let Ok(entries) = out_dir.read_dir() {
        if entries.count() > 0 && !args.force {
            return Err("Output directory is not empty. Use --force files".into());
        }
    }

    // Create the output directory if it doesn't exist
    fs::create_dir_all(&out_dir).map_err(|e| {
        format!(
            "Failed to create output directory '{}': {}",
            out_dir.display(),
            e
        )
    })?;

    for i in 0..args.n {
        // Generate the world description
        let desc = generate_desc(args.size, args.scale);

        let path = out_dir.join(format!("world_{}.ron", i));

        // Serialize to RON format
        let Ok(contents) = ron::ser::to_string_pretty(&desc, Default::default()) else {
            return Err(format!(
                "Failed to serialize world description: {}",
                path.display()
            ));
        };

        // Write to file
        fs::write(&path, contents)
            .map_err(|e| format!("Failed to write to file '{}': {}", path.display(), e))?;

        print!("[{}/{}]", i + 1, args.n);
        print!("\t Generated {}x{}", desc.width, desc.height,);
        println!("\t => {}", path.display());
    }

    // Render the worlds to images
    if let Some(render_dir) = args.render {
        let render_args = WorldToImgArgs {
            input: out_dir,
            output: render_dir,
            force: args.force,
            theme: simple_sim::gui::Theme::Light,
        };
        crate::world_to_img::world_to_img(render_args)?;
    }

    Ok(())
}

const OBSTACLE_OCCUPANCY: Range<f32> = 0.01..0.08;

const CIRCLE_RADIUS: Range<f32> = 0.5..2.0;
const LINE_WIDTH: Range<f32> = 0.3..1.0;

fn point_within_range(point: &Pos2, distance: f32) -> Pos2 {
    let p1 = point;
    let mut rng = rand::rng();
    loop {
        let p2 = *p1
            + Vec2::new(
                rng.random_range(-distance..distance),
                rng.random_range(-distance..distance),
            );
        if p1.distance(p2) <= distance {
            return p2;
        }
    }
}

const MAX_LINE_LENGTH: f32 = 20.0;
fn random_obstacle(world_size: Vec2) -> Shape {
    let n = rand::random();

    let mut rng = rand::rng();
    let mut rand_pos = || {
        Pos2::new(
            rng.random_range(-world_size.x / 2.0..world_size.x / 2.0),
            rng.random_range(-world_size.y / 2.0..world_size.y / 2.0),
        )
    };

    match n {
        ..0.5 => {
            let center = rand_pos();
            let radius = rng.random_range(CIRCLE_RADIUS);
            Shape::Circle(Circle { center, radius })
        }
        _ => {
            let start = rand_pos();
            let end = point_within_range(&start, MAX_LINE_LENGTH);
            let width = rng.random_range(LINE_WIDTH);
            Shape::WideLine(WideLine {
                line: Line { start, end },
                width,
            })
        }
    }
}

fn generate_desc(world_size: Vec2, scale: f32) -> ObjectDescription {
    let width = world_size.x.ceil();
    let height = world_size.y.ceil();

    let mut desc = ObjectDescription {
        width,
        height,
        scale,
        obstacles: vec![],
        search_items: vec![],
    };

    let area = width * height;
    let mut obstacle_area = 0.0;

    let target_occupancy = rand::rng().random_range(OBSTACLE_OCCUPANCY);

    while obstacle_area / area < target_occupancy {
        let obstacle = random_obstacle(world_size);
        obstacle_area += obstacle.area();
        desc.obstacles.push(obstacle);
    }

    desc
}
