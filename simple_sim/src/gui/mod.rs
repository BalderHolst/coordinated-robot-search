use app::App;
use eframe::egui::{self, Pos2, Rgba, Vec2};
use robcore::behaviors;

use crate::sim::{Robot, Simulator};

mod camera;
mod app;

pub fn run() {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([320.0, 240.0]),
        ..Default::default()
    };

    let mut sim = Simulator::new(behaviors::circle);
    sim.add_robot(Robot {
        pos: Pos2::new(-1.0, 0.0),
        vel: 0.0,
        angle: 0.0,
        avel: 0.0,
        color: Rgba::GREEN,
    });
    sim.add_robot(Robot {
        pos: Pos2::new(-0.5, 1.0),
        vel: 0.0,
        angle: 0.0,
        avel: 0.0,
        color: Rgba::BLUE,
    });

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
