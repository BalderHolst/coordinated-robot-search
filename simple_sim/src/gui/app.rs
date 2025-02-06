use crate::{bind_down, sim::Simulator};

use super::camera::Camera;
use eframe::{
    self,
    egui::{
        self, pos2, Align, Align2, Color32, FontFamily, FontId, Frame, Key, Margin, Painter, Pos2,
        Rect, Rgba, Sense, Shape, Stroke, Style, TextureHandle, TextureOptions, Vec2,
    },
    epaint::{Hsva, PathShape, PathStroke},
    CreationContext,
};
use robcore::Robot;

const FPS: f32 = 60.0;
const ROBOT_COLOR: Hsva = Hsva {
    h: 1.2,
    s: 0.2,
    v: 0.8,
    a: 1.0,
};

struct VisOpts {
    show_lidar: bool,
    show_velocity: bool,
}

impl Default for VisOpts {
    fn default() -> Self {
        Self {
            show_lidar: true,
            show_velocity: true,
        }
    }
}

pub struct App {
    pub cam: Camera,
    pub sim: Simulator,
    focused: Option<usize>,
    follow: Option<usize>,
    world_texture: TextureHandle,
    vis_opts: VisOpts,
}

impl App {
    pub fn new(sim: Simulator, cc: &CreationContext) -> Self {
        let world_image = sim.world.grid().get_image();
        let world_texture = cc.egui_ctx.load_texture(
            "world-grid-image",
            world_image,
            TextureOptions {
                magnification: egui::TextureFilter::Nearest,
                ..Default::default()
            },
        );
        Self {
            cam: Camera::new(Pos2::ZERO),
            sim,
            focused: None,
            follow: None,
            world_texture,
            vis_opts: VisOpts::default(),
        }
    }

    fn draw_robots(&mut self, painter: &Painter) {
        for (n, robot) in self.sim.robots.iter().enumerate() {
            let pos = self.cam.world_to_viewport(robot.pos);

            if self.vis_opts.show_lidar && self.focused == Some(n) {
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

            // Draw robot (as a circle)
            let stroke = match self.focused {
                Some(f) if f == n => Stroke::new(self.cam.scaled(0.04), Rgba::WHITE),
                _ => Stroke::NONE,
            };
            painter.circle(
                pos,
                self.cam.scaled(self.sim.robot_radius),
                ROBOT_COLOR,
                stroke,
            );

            // Draw velocity vector (arrow)
            if self.vis_opts.show_velocity {
                let vel = Vec2::angled(robot.angle) * robot.vel;
                let end = pos + self.cam.scaled(vel);
                let stroke_width = self.cam.scaled(0.05);
                let stroke = Stroke::new(stroke_width, ROBOT_COLOR);
                painter.circle_filled(end, stroke_width / 2.0, ROBOT_COLOR);
                painter.arrow(pos, end - pos, stroke);
            }

            // Draw id of robot
            let font_id = FontId {
                size: self.cam.scaled(0.3),
                family: FontFamily::Monospace,
            };
            painter.text(
                pos,
                Align2::CENTER_CENTER,
                n.to_string(),
                font_id,
                Hsva::new(0.0, 0.0, 0.02, 1.0).into(),
            );
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

                    ui.with_layout(egui::Layout::right_to_left(Align::Center), |ui| {
                        ui.label(format!("FPS: {:.1}", 1.0 / ctx.input(|i| i.stable_dt)));
                    });
                });
            });

        egui::TopBottomPanel::bottom("bottom-bar")
            .frame(Frame {
                inner_margin: Margin::symmetric(4.0, 4.0),
                fill: Hsva::new(0.0, 0.0, 0.005, 1.0).into(),
                ..Default::default()
            })
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Global Visualization Options:");
                    ui.toggle_value(&mut self.vis_opts.show_velocity, "Show Velocities");
                });
            });

        egui::SidePanel::right("right-panel")
            .resizable(true)
            .show_animated(ctx, self.focused.is_some(), |ui| {
                let n = self.focused.unwrap_or(0);
                let robot = &self.sim.robots[n];

                ui.horizontal(|ui| {
                    ui.heading(format!("Robot [{n}]"));
                    ui.with_layout(egui::Layout::right_to_left(Align::Center), |ui| match self
                        .follow
                    {
                        Some(_) => ui.button("Unfollow").clicked().then(|| {
                            self.follow = None;
                        }),
                        None => ui.button("Follow").clicked().then(|| {
                            self.follow = Some(n);
                        }),
                    });
                });

                ui.separator();
                ui.label(format!("Speed: {:.3}", robot.vel));
                ui.label(format!("Angle: {:.3}", robot.angle));
                ui.label(format!("Position: {:?}", robot.pos));
                ui.separator();

                ui.label("Visualization Options");
                ui.toggle_value(&mut self.vis_opts.show_lidar, "Show Lidar");

                // Keep new size for next frame
                ui.allocate_space(ui.available_size());
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

                let moved = self.cam.update(ui, &resp);

                if moved && self.follow.is_some() {
                    self.follow = None;
                }

                if let Some(f) = self.follow {
                    self.cam.pos = self.sim.robots[f].pos;
                }

                painter.rect_filled(world_rect, 0.0, Rgba::from_white_alpha(0.01));
                let uv = Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0));
                painter.image(self.world_texture.id(), world_rect, uv, Color32::WHITE);

                // Step simulation
                ui.input(|i| {
                    self.sim.step(i.stable_dt);
                });

                // Handle Keys
                ui.input(|i| {
                    bind_down!(i; Key::Escape => {
                        self.focused = None;
                        self.follow = None;
                    });
                });

                // Handle clicks
                resp.clicked().then(|| {
                    let Some(pos) = resp.interact_pointer_pos() else {
                        return;
                    };
                    let pos = self.cam.canvas_to_world(pos);

                    // Check if we clicked on a robot
                    let mut found = None;
                    for (n, robot) in self.sim.robots.iter().enumerate() {
                        if (robot.pos - pos).length() < self.sim.robot_radius * 1.5 {
                            self.focused = Some(n);
                            found = Some(n);
                            break;
                        }
                    }
                    self.focused = found;

                    // If we were following a robot, follow the clicked one
                    if self.follow.is_some() {
                        if let Some(f) = self.focused {
                            self.follow = Some(f);
                        }
                    }
                });

                self.draw_robots(&painter);
            });
    }
}
