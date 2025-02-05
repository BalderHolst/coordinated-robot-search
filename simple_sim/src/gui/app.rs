use crate::sim::Simulator;

use super::camera::Camera;
use eframe::{
    self,
    egui::{self, Frame, Margin, Painter, Pos2, Rect, Rgba, Sense, Shape, Vec2},
    epaint::{Hsva, PathShape, PathStroke},
};
use robcore::Robot;

const FPS: f32 = 60.0;

pub struct App {
    pub cam: Camera,
    pub sim: Simulator,
}

impl App {
    pub fn new(sim: Simulator) -> Self {
        Self {
            cam: Camera::new(Pos2::ZERO),
            sim,
        }
    }

    fn draw_robots(&mut self, painter: &Painter) {
        for robot in self.sim.robots.iter() {
            let pos = self.cam.world_to_viewport(robot.pos);
            let vel = Vec2::angled(robot.angle) * robot.vel;
            let end = pos + self.cam.scaled(vel);

            for point in &robot.get_lidar_data().0 {
                let end =
                    pos + Vec2::angled(robot.angle + point.angle) * self.cam.scaled(point.distance);
                painter.line(
                    vec![pos, end],
                    PathStroke::new(
                        self.cam.scaled(0.01),
                        Hsva::new(point.distance / self.sim.world.width() * 2.0, 0.8, 0.8, 1.0),
                    ),
                );
            }

            painter.line_segment(
                [pos, end],
                PathStroke::new(self.cam.scaled(0.10), robot.color),
            );

            painter.circle_filled(pos, self.cam.scaled(0.2), robot.color);
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint_after_secs(1.0 / FPS);

        egui::TopBottomPanel::top("top-bar")
            .frame(Frame {
                inner_margin: Margin::symmetric(4.0, 4.0),
                fill: Hsva::new(0.0, 0.0, 0.005, 1.0).into(),
                ..Default::default()
            })
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.heading(concat!(
                        env!("CARGO_PKG_NAME"),
                        " ",
                        env!("CARGO_PKG_VERSION")
                    ));
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
                self.cam.set_viewport(viewport);

                // Draw world area
                let (min, max) = &self.sim.world.bounds();
                let world_rect = Rect::from_points(&[
                    self.cam.world_to_viewport(*min),
                    self.cam.world_to_viewport(*max),
                ]);
                painter.rect_filled(world_rect, 0.0, Rgba::from_white_alpha(0.01));

                self.cam.update(ui, &resp);

                ui.input(|i| {
                    self.sim.step(i.unstable_dt);
                });

                self.draw_robots(&painter);
            });
    }
}
