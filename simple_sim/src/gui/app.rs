use super::camera::Camera;
use eframe::{
    self,
    egui::{self, Frame, Margin, Pos2, Rgba, Sense},
    epaint::Hsva,
};

pub struct App {
    cam: Camera,
}

impl App {
    pub fn new() -> Self {
        Self {
            cam: Camera::new(Pos2::ZERO),
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
