use crate::{bind_down, sim::Simulator};

use super::camera::Camera;
use eframe::{
    self,
    egui::{self, Frame, Key, Margin, Painter, Pos2, Rect, Rgba, Sense, Shape, Vec2},
    epaint::{Hsva, PathShape, PathStroke},
};
use robcore::Robot;

const FPS: f32 = 60.0;

pub struct App {
    pub cam: Camera,
    pub sim: Simulator,
    focused: Option<usize>,
}

impl App {
    pub fn new(sim: Simulator) -> Self {
        Self {
            cam: Camera::new(Pos2::ZERO),
            sim,
            focused: None,
        }
    }

    fn draw_robots(&mut self, painter: &Painter) {
        for (n, robot) in self.sim.robots.iter().enumerate() {
            let pos = self.cam.world_to_viewport(robot.pos);

            if self.focused == Some(n) {
                for point in &robot.get_lidar_data().0 {
                    let end = pos
                        + Vec2::angled(robot.angle + point.angle) * self.cam.scaled(point.distance);
                    painter.line(
                        vec![pos, end],
                        PathStroke::new(
                            self.cam.scaled(0.01),
                            Hsva::new(point.distance / self.sim.world.width() * 2.0, 0.8, 0.8, 1.0),
                        ),
                    );
                }
            }

            let vel = Vec2::angled(robot.angle) * robot.vel;
            let end = pos + self.cam.scaled(vel);
            painter.line_segment(
                [pos, end],
                PathStroke::new(self.cam.scaled(0.05), robot.color),
            );

            painter.circle_filled(pos, self.cam.scaled(self.sim.robot_size), robot.color);
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
                    ui.heading(concat!(
                        env!("CARGO_PKG_NAME"),
                        " ",
                        env!("CARGO_PKG_VERSION")
                    ));

                    ui.button("Home").clicked().then(|| {
                        self.focused = None;
                        self.cam.home();
                    });

                    ui.button("Unfocus").clicked().then(|| {
                        self.focused = None;
                    });

                    for (n, _robot) in self.sim.robots.iter().enumerate() {
                        ui.button(format!("Robot [{n}]")).clicked().then(|| {
                            self.focused = Some(n);
                        });
                    }
                });
            });

        egui::SidePanel::right("right-panel")
            .resizable(true)
            .show_animated(ctx, self.focused.is_some(), |ui| {
                let n = self.focused.unwrap_or(0);
                let robot = &self.sim.robots[n];
                ui.heading(format!("Robot [{n}]"));
                ui.label(format!("Speed: {:.3}", robot.vel));
                ui.label(format!("Angle: {:.3}", robot.angle));
                ui.label(format!("Position: {:?}", robot.pos));
            });

        egui::CentralPanel::default()
            .frame(Frame {
                fill: Hsva::new(0.0, 0.0, 0.01, 1.0).into(),
                ..Default::default()
            })
            .show(ctx, |ui| {
                ui.ctx().request_repaint_after_secs(1.0 / FPS);

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

                // Step simulation
                ui.input(|i| {
                    self.sim.step(i.unstable_dt);
                });

                // Handle Keys
                ui.input(|i| {
                    bind_down!(i; Key::Escape => self.focused = None);
                });

                self.draw_robots(&painter);
            });
    }
}
