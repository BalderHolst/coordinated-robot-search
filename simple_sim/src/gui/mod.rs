mod camera;

use camera::Camera;
use eframe::{
    self,
    egui::{self, Frame, Margin, Pos2, Rgba, Sense},
    epaint::Hsva,
};

struct App {
    cam: Camera,
}

impl App {
    fn new() -> Self {
        Self {
            cam: Camera::default(),
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("top-bar")
            .frame(Frame {
                inner_margin: Margin::symmetric(4.0, 4.0),
                fill: Hsva::new(0.0, 0.0, 0.005, 1.0).into(),
                ..Default::default()
            })
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.heading(env!("CARGO_PKG_NAME"));
                    ui.button("Back").clicked().then(|| {
                        println!("Back button clicked");
                    });
                });
            });

        // egui::SidePanel::right("right-panel")
        //     .resizable(true)
        //     .show_animated(ctx, true, |ui| {
        //     });

        egui::CentralPanel::default()
            .frame(Frame {
                fill: Hsva::new(0.0, 0.0, 0.01, 1.0).into(),
                ..Default::default()
            })
            .show(ctx, |ui| {
                let size = ui.available_size_before_wrap();
                let (resp, painter) = ui.allocate_painter(size, Sense::click_and_drag());

                let viewport = ui.ctx().input(|i| i.screen_rect());
                painter.rect_filled(viewport, 0.0, Rgba::from_white_alpha(0.01));
                self.cam.set_viewport(viewport);
                self.cam.update(ui, &resp);
                painter.rect_filled(painter.clip_rect(), 0.0, Rgba::from_white_alpha(0.01));

                painter.circle_filled(
                    self.cam.world_to_viewport(Pos2::ZERO),
                    self.cam.scaled(1.0),
                    Rgba::RED,
                );
            });
    }
}

pub fn run() {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([320.0, 240.0]),
        ..Default::default()
    };

    if let Err(e) = eframe::run_native(
        env!("CARGO_PKG_NAME"),
        options,
        Box::new(|_cc| Ok(Box::new(App::new()))),
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
