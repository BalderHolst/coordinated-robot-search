use app::{App, AppArgs};
use arrow_array::RecordBatch;
use arrow_schema::Schema;
use eframe::egui::{self, Style, Visuals};

use crate::{
    cli::{GlobArgs, RunArgs},
    scenario::Scenario,
    sim::{SimArgs, Simulator},
    world::world_from_path,
};

mod app;
mod bind_key;
mod camera;

pub use app::Theme;

pub fn run_interactive(args: GlobArgs, run_args: RunArgs) -> Result<(), String> {
    let sim_args = SimArgs {
        world: world_from_path(&run_args.world)?,
        behavior: run_args.behavior.clone(),
        #[cfg(not(feature = "single-thread"))]
        threads: args.threads,
        #[cfg(not(feature = "single-thread"))]
        no_debug_soup: args.no_debug_soup,
    };
    let app_args = args.into();
    let sim = Simulator::new(sim_args);
    run(sim, app_args)
}

pub fn run_scenario(
    sim: Simulator,
    scenario: Scenario,
    args: GlobArgs,
) -> Result<RecordBatch, String> {
    let app_args = AppArgs {
        pause_at: Some(scenario.duration),
        ..AppArgs::from(args)
    };
    run(sim, app_args)?;

    // TODO: Actually collect data
    Ok(RecordBatch::new_empty(Schema::empty().into()))
}

fn run(sim: Simulator, app_args: AppArgs) -> Result<(), String> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([320.0, 240.0]),
        ..Default::default()
    };

    eframe::run_native(
        concat!(env!("CARGO_PKG_NAME"), " ", env!("CARGO_PKG_VERSION")),
        options,
        Box::new(|cc| {
            let style = Style {
                visuals: match app_args.theme {
                    app::Theme::Light => Visuals::light(),
                    app::Theme::Dark => Visuals::dark(),
                    app::Theme::Grayscale => Visuals::light(),
                },
                ..Style::default()
            };
            cc.egui_ctx.set_style(style);
            cc.egui_ctx.set_pixels_per_point(app_args.ui_scale);

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
