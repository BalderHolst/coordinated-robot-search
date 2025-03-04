use std::{
    collections::HashSet,
    f32::consts::PI,
    hash::{self, Hash, Hasher},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
};

use crate::{
    bind_down, bind_pressed,
    cli::Args,
    sim::{Agent, Simulator, SimulatorState},
    world::Cell,
};

use super::camera::Camera;
use eframe::{
    self,
    egui::{
        self, pos2, Align, Align2, Color32, ColorImage, FontFamily, FontId, Frame, ImageData, Key,
        Margin, Painter, Pos2, Rect, Rgba, Sense, Stroke, TextureFilter, TextureId, TextureOptions,
        Vec2,
    },
    epaint::{Hsva, ImageDelta, PathStroke},
    CreationContext,
};
use robcore::debug::DebugType;

const ROBOT_COLOR: Hsva = Hsva {
    h: 1.2,
    s: 0.2,
    v: 0.8,
    a: 1.0,
};

fn string_color(s: &str) -> Color32 {
    let mut hasher = hash::DefaultHasher::new();
    s.hash(&mut hasher);
    let hash = hasher.finish();
    let hue = hash as f32 / u64::MAX as f32;
    Hsva::new(hue, 0.8, 0.8, 1.0).into()
}

/// Textures used by the app
struct AppTextures {
    world: TextureId,
    search_grid: Option<TextureId>,
}

pub struct GlobalOptions {
    paused: Arc<AtomicBool>,
    focused: Option<usize>,
    follow: Option<usize>,
}

impl Default for GlobalOptions {
    fn default() -> Self {
        Self {
            paused: Arc::new(AtomicBool::new(false)),
            focused: None,
            follow: None,
        }
    }
}

#[derive(Clone)]
pub struct RobotOptions {
    show_lidar: bool,
    show_cam_range: bool,
    show_search_grid: bool,
    debug_items: HashSet<String>,
    show_velocity: bool,
}

#[allow(clippy::derivable_impls)]
impl Default for RobotOptions {
    fn default() -> Self {
        Self {
            show_lidar: false,
            show_cam_range: false,
            show_search_grid: false,
            debug_items: Default::default(),
            show_velocity: true,
        }
    }
}

#[derive(Clone, Copy, Default)]
enum CursorState {
    #[default]
    Picking,
    SpawnRobot,
}

pub struct App {
    sim_bg: Arc<Mutex<Simulator>>,
    actual_sps_bg: Arc<Mutex<f32>>,
    target_sps_bg: Arc<Mutex<f32>>,
    target_sps: usize,
    target_fps: f32,
    pub cam: Camera,
    pub sim_state: SimulatorState,
    global_opts: GlobalOptions,
    robot_opts: Vec<RobotOptions>,
    textures: AppTextures,
    cursor_state: CursorState,
}

fn grid_to_image<C: Clone + Default>(
    grid: &robcore::grid::Grid<C>,
    color: impl Fn(C) -> Color32,
) -> ColorImage {
    let mut image = ColorImage::new([grid.width(), grid.height()], Cell::Empty.color());
    image.pixels.iter_mut().enumerate().for_each(|(i, pixel)| {
        let (x, y) = (i % grid.width(), i / grid.width());
        let cell = grid.get(x, y).cloned().unwrap_or_default();
        *pixel = color(cell);
    });
    image
}

impl App {
    pub fn new(mut sim: Simulator, args: Args, cc: &CreationContext) -> Self {
        // Step once to get the initial state
        sim.step();

        let sim_bg = Arc::new(Mutex::new(sim));
        let sim_state = sim_bg.lock().unwrap().state.clone();

        let actual_sps_bg = Arc::new(Mutex::new(args.target_sps));
        let target_sps_bg = Arc::new(Mutex::new(args.target_sps));
        let paused = Arc::new(AtomicBool::new(args.paused));

        {
            let sim_bg = sim_bg.clone();
            let actual_sps = actual_sps_bg.clone();
            let target_sps = target_sps_bg.clone();
            let paused = paused.clone();
            thread::spawn(move || loop {
                let target_sps = *target_sps.lock().unwrap();
                let paused = paused.load(Ordering::Relaxed);
                if paused || target_sps == 0.0 {
                    thread::sleep(std::time::Duration::from_secs_f32(1.0 / args.target_fps));
                    continue;
                }

                let frame_start = std::time::Instant::now();
                if let Ok(mut sim) = sim_bg.lock() {
                    sim.step();
                }

                let step_time = frame_start.elapsed();

                let target_time = std::time::Duration::from_secs_f32(1.0 / target_sps);
                let remaining = target_time.checked_sub(step_time).unwrap_or_default();
                thread::sleep(remaining);

                if let Ok(mut actual_sps) = actual_sps.try_lock() {
                    *actual_sps = 1.0 / frame_start.elapsed().as_secs_f32();
                }
            });
        }

        let world_image = grid_to_image(sim_state.world.grid(), |c| c.color());
        let world_image = ImageData::from(world_image);

        let texture_manager = &cc.egui_ctx.tex_manager();
        let mut texture_manager = texture_manager.write();

        let world_texture = texture_manager.alloc(
            "world-grid-image".to_string(),
            world_image,
            TextureOptions {
                magnification: TextureFilter::Nearest,
                ..Default::default()
            },
        );

        let global_opts = GlobalOptions {
            paused,
            ..Default::default()
        };

        let robot_opts = vec![RobotOptions::default(); sim_state.agents.len()];

        Self {
            cam: Camera::new(Pos2::ZERO, 100.0),
            sim_state,
            sim_bg,
            target_sps: args.target_sps as usize,
            actual_sps_bg,
            target_sps_bg,
            target_fps: args.target_fps,
            global_opts,
            robot_opts,
            textures: AppTextures {
                world: world_texture,
                search_grid: None,
            },
            cursor_state: CursorState::default(),
        }
    }

    fn draw_robots(&mut self, painter: &Painter) {
        for (n, agent) in self.sim_state.agents.iter().enumerate() {
            let robot = &agent.robot;
            let robot_opts = &self.robot_opts[n];

            let pos = self.cam.world_to_viewport(robot.pos);

            // Draw lidar rays
            if robot_opts.show_lidar {
                for point in robot.lidar.points() {
                    let end = pos
                        + Vec2::angled(robot.angle + point.angle) * self.cam.scaled(point.distance);
                    painter.line(
                        vec![pos, end],
                        PathStroke::new(
                            self.cam.scaled(0.01),
                            Hsva::new(
                                point.distance / self.sim_state.world.width() * 2.0,
                                0.8,
                                0.8,
                                0.5,
                            ),
                        ),
                    );
                }
            }

            // Draw Cam Range
            {
                const CAM_RANGE_STEPS: usize = 20;
                let stroke = PathStroke::new(self.cam.scaled(0.04), Hsva::new(0.0, 0.8, 0.8, 0.8));
                if robot_opts.show_cam_range {
                    let step_size = robot.cam_fov / CAM_RANGE_STEPS as f32;
                    let points = (0..CAM_RANGE_STEPS)
                        .map(|i| {
                            let angle = i as f32 * step_size - robot.cam_fov / 2.0;
                            let dist = robot.lidar.interpolate(angle).min(robot.cam_range);
                            let pos = robot.pos + Vec2::angled(angle + robot.angle) * dist;
                            self.cam.world_to_viewport(pos)
                        })
                        .collect();
                    painter.line(points, stroke);
                }
            }

            // Draw camera rays
            for point in &robot.cam.0 {
                let width = self.cam.scaled(0.20) * point.propability;
                let color = Hsva::new(0.0 * 2.0, 0.8, 0.8, point.propability);
                let end = pos + Vec2::angled(robot.angle + point.angle) * self.cam.scaled(1.0);
                painter.line(vec![pos, end], PathStroke::new(width, color));
            }

            // Draw FOV
            const FOV_INIDICATOR_LEN: f32 = 0.1;
            const FOV_INIDICATOR_WIDTH: f32 = 0.02;
            {
                let left = Vec2::angled(robot.angle - robot.cam_fov / 2.0);
                painter.line_segment(
                    [
                        pos,
                        pos + left * self.cam.scaled(robot.diameter / 2.0 + FOV_INIDICATOR_LEN),
                    ],
                    PathStroke::new(self.cam.scaled(FOV_INIDICATOR_WIDTH), ROBOT_COLOR),
                );
                let right = Vec2::angled(robot.angle + robot.cam_fov / 2.0);
                painter.line_segment(
                    [
                        pos,
                        pos + right * self.cam.scaled(robot.diameter / 2.0 + FOV_INIDICATOR_LEN),
                    ],
                    PathStroke::new(self.cam.scaled(FOV_INIDICATOR_WIDTH), ROBOT_COLOR),
                );
            }

            // Draw robot (as a circle)
            {
                let stroke = match self.global_opts.focused {
                    Some(f) if f == n => Stroke::new(self.cam.scaled(0.04), Rgba::WHITE),
                    _ => Stroke::NONE,
                };
                painter.circle(
                    pos,
                    self.cam.scaled(robot.diameter / 2.0),
                    ROBOT_COLOR,
                    stroke,
                );
            }

            // Draw velocity vector (arrow)
            if robot_opts.show_velocity {
                let vel = Vec2::angled(robot.angle) * (robot.vel + robot.diameter * 0.5);
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
                robot.id.as_u32().to_string(),
                font_id,
                Hsva::new(0.0, 0.0, 0.02, 1.0).into(),
            );

            // Draw search grid
            if robot_opts.show_search_grid {
                let (min, max) = self.sim_state.world.bounds();
                let min = self.cam.world_to_viewport(min);
                let max = self.cam.world_to_viewport(max);
                let canvas = Rect::from_min_max(min, max);

                let mut min: f32 = 0.0;
                let mut max: f32 = 0.0;
                for (_, _, cell) in robot.search_grid.grid().iter() {
                    min = min.min(*cell);
                    max = max.max(*cell);
                }

                const COOL_COLOR: Color32 = Color32::BLUE;
                const UNCERTAIN_COLOR: Color32 = Color32::WHITE;
                const WARM_COLOR: Color32 = Color32::YELLOW;
                const HOT_COLOR: Color32 = Color32::RED;

                const THRESHOLD: f32 = 10.0;

                let image = grid_to_image(robot.search_grid.grid(), |c| {
                    match c >= 0.0 {
                        true => WARM_COLOR.lerp_to_gamma(HOT_COLOR, c / max),
                        false => UNCERTAIN_COLOR.lerp_to_gamma(COOL_COLOR, c / min),
                    }
                    .gamma_multiply((c.abs() / THRESHOLD).min(1.0))
                });

                let texture_manager = &painter.ctx().tex_manager();
                let mut texture_manager = texture_manager.write();

                let id = match self.textures.search_grid {
                    Some(id) => {
                        texture_manager.set(id, ImageDelta::full(image, TextureOptions::default()));
                        id
                    }
                    None => texture_manager.alloc(
                        "search-grid-image".to_string(),
                        ImageData::from(image),
                        TextureOptions {
                            magnification: TextureFilter::Nearest,
                            ..Default::default()
                        },
                    ),
                };

                let uv = Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0));
                painter.image(id, canvas, uv, Color32::from_white_alpha(128));
            }

            // Draw debug soup
            let mut numbers = 0;
            if let Some(soup) = &robot.debug_soup {
                for (name, data) in soup.iter() {
                    if !robot_opts.debug_items.contains(name) {
                        continue;
                    }
                    let color = string_color(name);
                    match data {
                        DebugType::Vector(vec) => {
                            let end = pos + self.cam.scaled(*vec);
                            painter.arrow(
                                pos,
                                end - pos,
                                Stroke::new(self.cam.scaled(0.03), color),
                            );
                        }
                        DebugType::Vectors(vecs) => {
                            for vec in vecs {
                                let end = pos + self.cam.scaled(*vec);
                                painter.arrow(
                                    pos,
                                    end - pos,
                                    Stroke::new(self.cam.scaled(0.01), color.gamma_multiply(0.3)),
                                );
                            }
                        }
                        DebugType::WeightedVectors(vecs) => {
                            let max_weight = vecs.iter().map(|(_, w)| w.abs()).sum::<f32>();
                            for (vec, weight) in vecs {
                                let alpha = *weight / max_weight;
                                let color = color.gamma_multiply(1.0 - alpha);
                                let end = pos + self.cam.scaled(*vec);
                                painter.arrow(
                                    pos,
                                    end - pos,
                                    Stroke::new(self.cam.scaled(0.01), color),
                                );
                            }
                        }
                        DebugType::Radius(radius) => {
                            painter.circle_stroke(
                                pos,
                                self.cam.scaled(*radius),
                                Stroke::new(self.cam.scaled(0.02), color),
                            );
                        }
                        DebugType::Point(p) => {
                            let p = self.cam.world_to_viewport(*p);
                            painter.circle_filled(p, self.cam.scaled(0.05), color);
                        }
                        DebugType::Points(ps) => {
                            for p in ps {
                                let p = self.cam.world_to_viewport(*p);
                                painter.circle_filled(p, self.cam.scaled(0.03), color);
                            }
                        }
                        DebugType::WeightedPoints(ps) => {
                            let max_weight =
                                ps.iter()
                                    .map(|(_, w)| w)
                                    .fold(0.0, |acc, w| match *w > acc {
                                        true => *w,
                                        false => acc,
                                    });
                            let min_weight =
                                ps.iter()
                                    .map(|(_, w)| w)
                                    .fold(0.0, |acc, w| match *w < acc {
                                        true => *w,
                                        false => acc,
                                    });
                            for (p, w) in ps {
                                let p = self.cam.world_to_viewport(*p);
                                let alpha = (*w - min_weight) / (max_weight - min_weight);
                                let color = color.gamma_multiply(alpha);
                                painter.circle_filled(p, self.cam.scaled(0.03), color);
                            }
                        }
                        DebugType::NumberPoints(ps) => {
                            let font_id = FontId {
                                size: self.cam.scaled(0.08),
                                family: FontFamily::Monospace,
                            };
                            for (p, w) in ps {
                                let p = self.cam.world_to_viewport(*p);
                                painter.text(
                                    p,
                                    Align2::CENTER_CENTER,
                                    format!("{:.1}", w),
                                    font_id.clone(),
                                    color,
                                );
                            }
                        }
                        DebugType::Number(n) => {
                            let font_id = FontId {
                                size: self.cam.scaled(0.2),
                                family: FontFamily::Monospace,
                            };
                            painter.text(
                                pos + Vec2::new(
                                    0.0,
                                    -self.cam.scaled(0.5) - numbers as f32 * self.cam.scaled(0.2),
                                ),
                                Align2::CENTER_CENTER,
                                format!("{name}: {n:.3}"),
                                font_id,
                                color,
                            );
                            numbers += 1;
                        }
                    }
                }
            }
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
                        self.global_opts.focused = None;
                        self.cam.home();
                    });

                    ui.button("Unfocus").clicked().then(|| {
                        self.global_opts.focused = None;
                    });

                    for (n, _) in self.sim_state.agents.iter().enumerate() {
                        ui.button(format!("Robot [{n}]")).clicked().then(|| {
                            self.global_opts.focused = Some(n);
                        });
                    }

                    ui.with_layout(egui::Layout::right_to_left(Align::Center), |ui| {
                        ui.label(format!("FPS: {:.0}", 1.0 / ctx.input(|i| i.unstable_dt)));
                        if let Ok(sps) = self.actual_sps_bg.lock() {
                            ui.label(format!("SPS: {:.0}", sps));
                        }
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
                    ui.label("Simulation Options");

                    // SPS Slider
                    let prev_target_sps = self.target_sps;
                    ui.label("SPS:");
                    ui.add(
                        egui::Slider::new(&mut self.target_sps, 0..=240)
                            .clamping(egui::SliderClamping::Never),
                    );
                    if prev_target_sps != self.target_sps {
                        *self.target_sps_bg.lock().unwrap() = self.target_sps as f32;
                    }

                    // Pause/Resume Button
                    let paused = self.global_opts.paused.load(Ordering::Relaxed);
                    ui.button(if paused { "Resume" } else { "Pause" })
                        .clicked()
                        .then(|| {
                            self.global_opts.paused.store(!paused, Ordering::Relaxed);
                        });

                    //Spawn Robot Button
                    ui.selectable_label(
                        matches!(self.cursor_state, CursorState::SpawnRobot),
                        "Spawn Robots",
                    )
                    .clicked()
                    .then(|| match self.cursor_state {
                        CursorState::SpawnRobot => self.cursor_state = CursorState::default(),
                        _ => self.cursor_state = CursorState::SpawnRobot,
                    });
                });
            });

        egui::SidePanel::right("right-panel")
            .resizable(true)
            .show_animated(ctx, self.global_opts.focused.is_some(), |ui| {
                let n = self.global_opts.focused.unwrap_or(0);
                let robot = &self.sim_state.agents[n].robot;
                let robot_opts = &mut self.robot_opts[n];

                ui.horizontal(|ui| {
                    ui.heading(format!("Robot [{n}]"));
                    ui.with_layout(egui::Layout::right_to_left(Align::Center), |ui| match self
                        .global_opts
                        .follow
                    {
                        Some(_) => ui.button("Unfollow").clicked().then(|| {
                            self.global_opts.follow = None;
                        }),
                        None => ui.button("Follow").clicked().then(|| {
                            self.global_opts.follow = Some(n);
                        }),
                    });
                });

                ui.separator();
                ui.label(format!("Speed: {:.3}", robot.vel));
                ui.label(format!("Angle: {:.3}", robot.angle));
                ui.label(format!("Position: {:?}", robot.pos));
                ui.separator();

                ui.label("Visualization Options");
                ui.toggle_value(&mut robot_opts.show_velocity, "Show Velocity");
                ui.toggle_value(&mut robot_opts.show_lidar, "Show Lidar");
                ui.toggle_value(&mut robot_opts.show_cam_range, "Show Cam Range");

                ui.separator();
                ui.heading("Grids");
                ui.toggle_value(&mut robot_opts.show_search_grid, "Show Search Grid");

                ui.separator();
                ui.heading("Debug Items");
                if let Some(soup) = &robot.debug_soup {
                    let mut names: Vec<_> = soup.keys().collect();
                    names.sort();
                    for name in names {
                        ui.horizontal(|ui| {
                            let color = string_color(name);

                            let (_, painter) = ui.allocate_painter(
                                Vec2::splat(10.0),
                                Sense {
                                    click: false,
                                    drag: false,
                                    focusable: false,
                                },
                            );

                            painter.rect_filled(painter.clip_rect(), 0.0, color);

                            let mut shown = robot_opts.debug_items.contains(name);
                            let resp = ui.toggle_value(&mut shown, name);
                            if resp.changed() {
                                match shown {
                                    true => robot_opts.debug_items.insert(name.to_string()),
                                    false => robot_opts.debug_items.remove(name),
                                };
                            }
                        });
                    }
                }

                // Keep new size for next frame
                ui.allocate_space(ui.available_size());
            });

        egui::CentralPanel::default()
            .frame(Frame {
                fill: Hsva::new(0.0, 0.0, 0.01, 1.0).into(),
                ..Default::default()
            })
            .show(ctx, |ui| {
                ui.ctx().request_repaint_after_secs(1.0 / self.target_fps);

                let size = ui.available_size_before_wrap();
                let (resp, painter) = ui.allocate_painter(size, Sense::click_and_drag());

                let viewport = ui.ctx().input(|i| i.screen_rect());
                self.cam.set_viewport(viewport);

                // Draw world area
                let (min, max) = &self.sim_state.world.bounds();
                let world_rect = Rect::from_points(&[
                    self.cam.world_to_viewport(*min),
                    self.cam.world_to_viewport(*max),
                ]);

                let moved = self.cam.update(ui, &resp);

                if moved && self.global_opts.follow.is_some() {
                    self.global_opts.follow = None;
                }

                if let Some(f) = self.global_opts.follow {
                    let agent_pos = self.sim_state.agents[f].pos();
                    self.cam.set_pos(agent_pos);
                }

                painter.rect_filled(world_rect, 0.0, Rgba::from_white_alpha(0.01));
                let uv = Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0));
                painter.image(self.textures.world, world_rect, uv, Color32::WHITE);

                // Get simulation
                if let Ok(sim) = self.sim_bg.lock() {
                    self.sim_state = sim.state.clone();
                }

                // Handle Keys
                ui.input(|i| {
                    bind_down!(i; Key::Escape => {
                        self.global_opts.focused = None;
                        self.global_opts.follow = None;
                    });

                    bind_pressed!(i; Key::Space => {
                        let paused = self.global_opts.paused.load(Ordering::Relaxed);
                        self.global_opts.paused.store(!paused, Ordering::Relaxed);
                    });
                });

                // Handle clicks
                resp.clicked().then(|| {
                    let Some(pos) = resp.interact_pointer_pos() else {
                        return;
                    };
                    let pos = self.cam.canvas_to_world(pos);

                    match self.cursor_state {
                        CursorState::SpawnRobot => {
                            let mut sim = self.sim_bg.lock().unwrap();
                            let agent =
                                Agent::new_at(pos, PI / 2.0 * self.sim_state.agents.len() as f32);
                            sim.add_robot(agent);
                            self.robot_opts.push(RobotOptions::default());
                        }
                        CursorState::Picking => {
                            // Check if we clicked on a robot
                            let mut found = None;
                            for (n, agent) in self.sim_state.agents.iter().enumerate() {
                                let robot = &agent.robot;
                                if (robot.pos - pos).length() < robot.diameter * 0.75 {
                                    self.global_opts.focused = Some(n);
                                    found = Some(n);
                                    break;
                                }
                            }
                            self.global_opts.focused = found;

                            // If we were following a robot, follow the clicked one
                            if self.global_opts.follow.is_some() {
                                if let Some(f) = self.global_opts.focused {
                                    self.global_opts.follow = Some(f);
                                }
                            }
                        }
                    }
                });

                self.draw_robots(&painter);
            });
    }
}
