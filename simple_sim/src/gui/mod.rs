use app::App;
use eframe::egui;
use robcore::behaviors;

use crate::sim::{Robot, Simulator, World};

mod camera;
mod app;

pub fn run() {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([320.0, 240.0]),
        ..Default::default()
    };

    let world = World::new(10.0, 10.0);
    let mut sim = Simulator::new(world, behaviors::circle);
    sim.add_robot(Robot::new_at(-1.0, 0.0));
    sim.add_robot(Robot::new_at(1.0, -0.5));

    let app = App::new(sim);

    if let Err(e) = eframe::run_native(
        env!("CARGO_PKG_NAME"),
        options,
        Box::new(|_cc| Ok(Box::new(app))),
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
