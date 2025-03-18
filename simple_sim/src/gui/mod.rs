use app::{App, AppArgs};
use eframe::egui;

use crate::{cli::{RunArgs, ScenarioArgs}, scenario::Scenario, sim::{SimArgs, Simulator}, world::{world_from_path, World}};

mod app;
mod bind_key;
mod camera;

pub fn run_interactive(args: RunArgs) -> Result<(), String> {
    let sim_args = SimArgs {
        world: world_from_path(&args.world)?,
        behavior: args.behavior.clone(),
        threads: args.threads,
    };
    let app_args = args.into();
    run(sim_args, app_args)
}

pub fn run_scenario(args: ScenarioArgs, world: World, scenario: Scenario) -> Result<(), String> {
    let sim_args = SimArgs {
        world,
        behavior: scenario.behavior,
        threads: args.threads,
    };

    let app_args = AppArgs {
        paused: false,
        target_fps: args.target_fps,
        target_sps: args.target_sps,
    };

    run(sim_args, app_args)
}

fn run(sim_args: SimArgs, app_args: AppArgs) -> Result<(), String> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([320.0, 240.0]),
        ..Default::default()
    };

    eframe::run_native(
        concat!(env!("CARGO_PKG_NAME"), " ", env!("CARGO_PKG_VERSION")),
        options,
        Box::new(|cc| {

            // Create simulator
            let sim = Simulator::new(sim_args);

            // Create app
            let app = App::new(sim, app_args, cc);

            Ok(Box::new(app))
        }),
    )
    .map_err(|e| match e {
        eframe::Error::NoGlutinConfigs(_, _) => {
            format!("{e}.\n\nIs the system library in sync with the dynamically linked one?")
        }
        _ => format!("{}", e),
    })
}
