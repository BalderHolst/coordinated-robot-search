use std::f32::consts::PI;

use app::App;
use eframe::egui::{self, pos2};

use crate::{
    cli::Args,
    sim::{Robot, Simulator},
    world::{Cell, World},
};

mod app;
mod bind_key;
mod camera;

/// Target frames per second for the gui
const TARGET_FPS: f32 = 60.0;

/// Target simulation steps per second
const TARGET_SPS: f32 = 60.0;

pub fn run(args: Args) {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([320.0, 240.0]),
        ..Default::default()
    };

    if let Err(e) = eframe::run_native(
        concat!(env!("CARGO_PKG_NAME"), " ", env!("CARGO_PKG_VERSION")),
        options,
        Box::new(|cc| {
            // Set up world
            let mut world = World::new(10.0, 10.0);
            world.line(pos2(-2.0, 0.0), pos2(1.5, 0.0), 0.2, Cell::Wall);
            world.line(pos2(-2.0, 0.0), pos2(-2.0, -2.0), 0.2, Cell::Wall);
            world.circle(pos2(0.0, 3.0), 0.08, Cell::SearchItem);

            // Set up simulator
            let mut sim = Simulator::new(world, TARGET_SPS, args.behavior.get_fn());
            sim.add_robot(Robot::new_at(-2.0, -2.0, 1.0));
            sim.add_robot(Robot::new_at(-1.0, 1.0, 0.0));
            sim.add_robot(Robot::new_at(2.2, 1.1, PI));

            // Create app
            let app = App::new(sim, cc);
            Ok(Box::new(app))
        }),
    ) {
        let m = match e {
            eframe::Error::AppCreation(_) => todo!(),
            eframe::Error::Winit(_) => todo!(),
            eframe::Error::WinitEventLoop(e) => todo!("{e:?}"),
            eframe::Error::Glutin(_) => todo!(),
            eframe::Error::NoGlutinConfigs(_, _) => {
                format!("{e}.\n\nIs the system library in sync with the dynamically linked one?")
            }
            other => format!("{}", other),
        };
        eprintln!("Error running GUI: {m}");
    }
}
