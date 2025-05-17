use std::f32::consts::PI;

use botbrain::{params, Pos2, RobotPose, Vec2};
use rand::SeedableRng;
use simple_sim::world::{world_from_path, World};

use crate::cli;

const SPAWN_MARGIN: f32 = params::DIAMETER;

const ROBOT_WIGGLE_ROOM: f32 = params::DIAMETER;
const ROBOT_MIN_SPACE: f32 = 0.2;

const ROBOT_SPACE: f32 = params::DIAMETER + ROBOT_MIN_SPACE * 2.0 + ROBOT_WIGGLE_ROOM * 2.0;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PlacementResult {
    pub poses: Vec<RobotPose>,
    pub square_size: f32,
    pub square_center: Pos2,
    pub attempts: usize,
}

pub fn place_robots(args: &cli::PlaceRobotsArgs) -> Result<(), String> {
    let mut rng = match args.seed {
        Some(seed) => rand::rngs::StdRng::seed_from_u64(seed),
        None => rand::rngs::StdRng::from_os_rng(),
    };

    let world = world_from_path(&args.world)?;

    let positions = place(&world, args.n_robots, &mut rng);

    println!(
        "{}",
        serde_json::to_string(&positions)
            .map_err(|e| format!("Failed to serialize positions: {}", e))?
    );

    Ok(())
}

pub fn place(world: &World, n_robots: usize, rng: &mut impl rand::Rng) -> PlacementResult {
    let dim = (n_robots as f32).sqrt().ceil() as usize;

    let spawn_square_size = dim as f32 * ROBOT_SPACE + SPAWN_MARGIN * 2.0;

    let mut attempts = 0;
    let mut spawn_square_center;
    loop {
        attempts += 1;

        // Get a random placement of the spawn square
        spawn_square_center = Pos2::new(
            rng.random_range(
                -world.height() / 2.0 + spawn_square_size / 2.0
                    ..world.width() / 2.0 - spawn_square_size / 2.0,
            ),
            rng.random_range(
                -world.height() / 2.0 + spawn_square_size / 2.0
                    ..world.height() / 2.0 - spawn_square_size / 2.0,
            ),
        );

        eprintln!(
            "Trying to place robots in square of size {} at center {}",
            spawn_square_size, spawn_square_center
        );

        // Validate the placement
        {
            let mut valid = true;

            for pos in world.iter_square(spawn_square_center, spawn_square_size) {
                match world.get(pos) {
                    Some(cell) if cell.is_empty() => {}
                    _ => {
                        valid = false;
                        break;
                    }
                }
            }

            if valid {
                break;
            }
        }
    }

    // Assign slots
    let mut free = (0..dim * dim).collect::<Vec<_>>();
    let mut slots_indexes = vec![];

    for _ in 0..n_robots {
        let r = rng.random_range(0..free.len());
        slots_indexes.push(free.remove(r));
    }

    // Assign positions
    let slot_pose = |slot: usize| {
        let x_idx = slot % dim;
        let y_idx = slot / dim;

        let origin = spawn_square_center
            + Vec2::splat(-spawn_square_size / 2.0 + SPAWN_MARGIN + ROBOT_SPACE / 2.0);

        let pos = origin
            + Vec2 {
                x: x_idx as f32 * ROBOT_SPACE
                    + rng.random_range(-ROBOT_WIGGLE_ROOM..ROBOT_WIGGLE_ROOM),
                y: y_idx as f32 * ROBOT_SPACE
                    + rng.random_range(-ROBOT_WIGGLE_ROOM..ROBOT_WIGGLE_ROOM),
            };

        let angle = rng.random_range(-PI..PI);
        RobotPose { pos, angle }
    };

    let positions = slots_indexes.into_iter().map(slot_pose).collect();

    PlacementResult {
        poses: positions,
        square_size: spawn_square_size,
        square_center: spawn_square_center,
        attempts,
    }
}
