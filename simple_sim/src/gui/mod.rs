use std::f32::consts::PI;

use app::App;
use eframe::egui;

use crate::{
    cli::Args,
    sim::{Agent, Simulator},
    world::world_from_path,
};

mod app;
mod bind_key;
mod camera;

pub fn run(args: Args) {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([320.0, 240.0]),
        ..Default::default()
    };

    if let Err(e) = eframe::run_native(
        concat!(env!("CARGO_PKG_NAME"), " ", env!("CARGO_PKG_VERSION")),
        options,
        Box::new(|cc| {
            let world = world_from_path(&args.world);

            // Set up simulator
            let mut sim = Simulator::new(world, args.target_sps, args.behavior.get_fn());
            sim.add_robot(Agent::new_at(-2.0, -2.0, 1.0));
            sim.add_robot(Agent::new_at(2.2, 1.1, PI));
            sim.add_robot(Agent::new_at(-1.0, 1.0, 0.0));

            // Create app
            let app = App::new(sim, args, cc);
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
