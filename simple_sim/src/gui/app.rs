use std::{
    collections::{HashMap, HashSet},
    f32::consts::PI,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
};

use crate::{
    bind_down, bind_pressed,
    cli::RunArgs,
    sim::{Simulator, SimulatorState},
    world::Cell,
};

use super::camera::Camera;
use botbrain::{debug::DebugType, RobotParameters};
use eframe::{
    self,
    egui::{
        self, pos2, Align, Align2, Color32, ColorImage, FontFamily, FontId, Frame, ImageData, Key,
        Margin, Painter, Pos2, Rect, Rgba, Sense, Separator, Stroke, TextureFilter, TextureId,
        TextureOptions, Vec2,
    },
    epaint::{Hsva, ImageDelta, PathStroke},
    CreationContext,
};

const ROBOT_COLOR: Hsva = Hsva {
    h: 1.2,
    s: 0.1,
    v: 0.6,
    a: 1.0,
};

/// Textures used by the app
struct AppTextures {
    world: TextureId,
    others: HashMap<String, TextureId>,
}

pub struct GlobalOptions {
    paused: Arc<AtomicBool>,
    focused: Option<usize>,
    follow: Option<usize>,
    show_only: Option<usize>,
}

impl Default for GlobalOptions {
    fn default() -> Self {
        Self {
            paused: Arc::new(AtomicBool::new(false)),
            focused: None,
            follow: None,
            show_only: None,
        }
    }
}

#[derive(Clone)]
pub struct RobotOptions {
    debug_items: HashSet<(&'static str, &'static str)>,
}

#[allow(clippy::derivable_impls)]
impl Default for RobotOptions {
    fn default() -> Self {
        Self {
            debug_items: Default::default(),
        }
    }
}

#[derive(Clone, Copy, Default)]
enum CursorState {
    #[default]
    Picking,
    SpawnRobot,
    SpawnManyRobots,
}

pub struct App {
    sim_bg: Arc<Mutex<Simulator>>,
    actual_sps_bg: Arc<Mutex<f32>>,
    target_sps_bg: Arc<Mutex<f32>>,
    target_sps: usize,
    target_fps: f32,
    actual_sps: f32,
    pub cam: Camera,
    pub sim_state: SimulatorState,
    global_opts: GlobalOptions,
    robot_opts: Vec<RobotOptions>,
    textures: AppTextures,
    cursor_state: CursorState,
}

fn grid_to_image<C: Clone + Default>(
    grid: &botbrain::grid::Grid<C>,
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

pub struct AppArgs {
    pub target_sps: f32,
    pub target_fps: f32,
    pub paused: bool,
}

impl From<RunArgs> for AppArgs {
    fn from(args: RunArgs) -> Self {
        Self {
            target_sps: args.target_sps,
            target_fps: args.target_fps,
            paused: args.paused,
        }
    }
}

impl App {
    pub fn new(mut sim: Simulator, args: AppArgs, cc: &CreationContext) -> Self {
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
                    // println!("p2: {:?}", sim.state.agents.iter().map(|a| a.state.pos).collect::<Vec<_>>());
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
            actual_sps: args.target_sps as f32,
            global_opts,
            robot_opts,
            textures: AppTextures {
                world: world_texture,
                others: Default::default(),
            },
            cursor_state: CursorState::default(),
        }
    }

    fn draw_robots(&mut self, painter: &Painter) {
        for (n, agent) in self.sim_state.agents.iter().enumerate() {
            let robot_state = &agent.state;
            let robot_opts = &self.robot_opts[n];

            let RobotParameters {
                diameter, cam_fov, ..
            } = agent.robot.params();

            let robot_pos = self.cam.world_to_viewport(agent.state.pos);
            let robot_angle = agent.state.angle;

            if self.global_opts.show_only.is_none_or(|f| f == n) {
                // Draw robot (as a circle)
                {
                    let stroke = match self.global_opts.focused {
                        Some(f) if f == n => Stroke::new(self.cam.scaled(0.04), Rgba::WHITE),
                        _ => Stroke::NONE,
                    };
                    painter.circle(
                        robot_pos,
                        self.cam.scaled(diameter / 2.0),
                        ROBOT_COLOR,
                        stroke,
                    );
                }

                // Draw FOV
                const FOV_INIDICATOR_LEN: f32 = 0.1;
                const FOV_INIDICATOR_WIDTH: f32 = 0.02;
                {
                    let left = Vec2::angled(robot_angle - cam_fov / 2.0);
                    painter.line_segment(
                        [
                            robot_pos,
                            robot_pos + left * self.cam.scaled(diameter / 2.0 + FOV_INIDICATOR_LEN),
                        ],
                        PathStroke::new(self.cam.scaled(FOV_INIDICATOR_WIDTH), ROBOT_COLOR),
                    );
                    let right = Vec2::angled(robot_angle + cam_fov / 2.0);
                    painter.line_segment(
                        [
                            robot_pos,
                            robot_pos
                                + right * self.cam.scaled(diameter / 2.0 + FOV_INIDICATOR_LEN),
                        ],
                        PathStroke::new(self.cam.scaled(FOV_INIDICATOR_WIDTH), ROBOT_COLOR),
                    );
                }

                // Draw velocity vector (arrow)
                let vel = Vec2::angled(robot_angle) * (robot_state.vel + diameter * 0.5);
                let end = robot_pos + self.cam.scaled(vel);
                let stroke_width = self.cam.scaled(0.05);
                let stroke = Stroke::new(stroke_width, ROBOT_COLOR);
                painter.circle_filled(end, stroke_width / 2.0, ROBOT_COLOR);
                painter.arrow(robot_pos, end - robot_pos, stroke);

                // Draw id of robot
                let font_id = FontId {
                    size: self.cam.scaled(0.3),
                    family: FontFamily::Monospace,
                };
                painter.text(
                    robot_pos,
                    Align2::CENTER_CENTER,
                    agent.robot.id().as_u32().to_string(),
                    font_id,
                    Hsva::new(0.0, 0.0, 0.02, 1.0).into(),
                );
            }

            let mut numbers = 0;
            for (n, (category, name, data)) in agent.robot.get_debug_soup().iter().enumerate() {
                if !robot_opts.debug_items.contains(&(category, name)) {
                    continue;
                }
                let color = get_color(n);
                match data {
                    DebugType::Vector(vec) => {
                        let end = robot_pos + self.cam.scaled(*vec);
                        painter.arrow(
                            robot_pos,
                            end - robot_pos,
                            Stroke::new(self.cam.scaled(0.03), color),
                        );
                    }
                    DebugType::Vectors(vecs) => {
                        for vec in vecs {
                            let end = robot_pos + self.cam.scaled(*vec);
                            painter.arrow(
                                robot_pos,
                                end - robot_pos,
                                Stroke::new(self.cam.scaled(0.01), color.gamma_multiply(0.3)),
                            );
                        }
                    }
                    DebugType::WeightedVectors(vecs) => {
                        let max_weight = vecs.iter().map(|(_, w)| w.abs()).sum::<f32>();
                        for (vec, weight) in vecs {
                            let alpha = *weight / max_weight;
                            let color = color.gamma_multiply(1.0 - alpha);
                            let end = robot_pos + self.cam.scaled(*vec);
                            painter.arrow(
                                robot_pos,
                                end - robot_pos,
                                Stroke::new(self.cam.scaled(0.01), color),
                            );
                        }
                    }
                    DebugType::VectorField(field) => {
                        for (pos, vec) in field {
                            let pos = self.cam.world_to_viewport(*pos);
                            let end = pos + self.cam.scaled(*vec);
                            painter.arrow(
                                pos,
                                end - pos,
                                Stroke::new(self.cam.scaled(0.05), color),
                            );
                        }
                    }
                    DebugType::Radius(radius) => {
                        painter.circle_stroke(
                            robot_pos,
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
                            robot_pos
                                + Vec2::new(
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
                    DebugType::RobotRays(rays) => {
                        for (angle, distance) in rays {
                            let color = Hsva::new(
                                distance / self.sim_state.world.width() * 2.0,
                                0.8,
                                0.8,
                                0.5,
                            );
                            let dir = Vec2::angled(robot_angle + *angle);
                            let start = robot_pos + dir * self.cam.scaled(*diameter / 2.0);
                            let end = robot_pos + dir * self.cam.scaled(*distance);
                            painter.line(
                                vec![start, end],
                                PathStroke::new(self.cam.scaled(0.01), color),
                            );
                        }
                    }
                    DebugType::RobotLine(points) => {
                        let points = points
                            .iter()
                            .map(|p| {
                                let len = p.length();
                                let angle = robot_angle + p.angle();
                                robot_pos + Vec2::angled(angle) * self.cam.scaled(len)
                            })
                            .collect();
                        painter.line(points, PathStroke::new(self.cam.scaled(0.01), color));
                    }
                    DebugType::Grid(grid) => {
                        let (min, max) = self.sim_state.world.bounds();
                        let min = self.cam.world_to_viewport(min);
                        let max = self.cam.world_to_viewport(max);
                        let canvas = Rect::from_min_max(min, max);

                        let mut min: f32 = 0.0;
                        let mut max: f32 = 0.0;
                        for (_, _, cell) in grid.grid().iter() {
                            min = min.min(*cell);
                            max = max.max(*cell);
                        }

                        const COOL_COLOR: Color32 = Color32::LIGHT_BLUE;
                        const COLD_COLOR: Color32 = Color32::BLUE;
                        const UNCERTAIN_COLOR: Color32 = Color32::WHITE;
                        const WARM_COLOR: Color32 = Color32::YELLOW;
                        const HOT_COLOR: Color32 = Color32::RED;

                        const THRESHOLD: f32 = 10.0;

                        fn ease_out_quart(c: f32) -> f32 {
                            1. - f32::powi(1. - c, 4)
                        }

                        let image = grid_to_image(grid.grid(), |c| {
                            match c {
                                c if c > 0.0 => {
                                    WARM_COLOR.lerp_to_gamma(HOT_COLOR, ease_out_quart(c / max))
                                }
                                c if c < 0.0 => {
                                    COOL_COLOR.lerp_to_gamma(COLD_COLOR, ease_out_quart(c / min))
                                }
                                _ => UNCERTAIN_COLOR,
                            }
                            .gamma_multiply((c.abs() / THRESHOLD).min(1.0))
                        });

                        let texture_manager = &painter.ctx().tex_manager();
                        let mut texture_manager = texture_manager.write();

                        let texture_options = TextureOptions {
                            magnification: TextureFilter::Nearest,
                            ..Default::default()
                        };

                        let maybe_id = self.textures.others.get(name).cloned();
                        let id = match maybe_id {
                            Some(id) => {
                                texture_manager.set(id, ImageDelta::full(image, texture_options));
                                id
                            }
                            None => {
                                let id = texture_manager.alloc(
                                    format!("{}-grid-image", name),
                                    ImageData::from(image),
                                    texture_options,
                                );
                                // Add to cache
                                println!("[INFO] Allocated new texture for {} (id={:?})", name, id);
                                self.textures.others.insert(name.to_string(), id);
                                id
                            }
                        };

                        let uv = Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0));
                        painter.image(id, canvas, uv, Color32::from_white_alpha(128));
                    }
                }
            }
        }
    }

    fn spawn_robot(&mut self, pos: Pos2) {
        let mut sim = self.sim_bg.lock().unwrap();
        let angle = PI / 2.0 * self.sim_state.agents.len() as f32;
        sim.add_robot(pos, angle);
        self.robot_opts.push(RobotOptions::default());
        self.global_opts.focused = Some(self.robot_opts.len() - 1);
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

                    ui.add(Separator::default().vertical());

                    if let Ok(actual_sps) = self.actual_sps_bg.lock() {
                        self.actual_sps = *actual_sps;
                    }
                    ui.with_layout(egui::Layout::right_to_left(Align::Center), |ui| {
                        ui.label(format!("FPS: {:>03.0}", 1.0 / ctx.input(|i| i.unstable_dt)));
                        ui.label(format!("SPS: {:>03.0}", self.actual_sps));

                        ui.add(Separator::default().vertical());

                        ui.label(format!("Time: {:.1}s", self.sim_state.time.as_secs_f64()));
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
                        matches!(self.cursor_state, CursorState::SpawnManyRobots),
                        "Spawn Robots",
                    )
                    .clicked()
                    .then(|| match self.cursor_state {
                        CursorState::SpawnManyRobots => self.cursor_state = CursorState::default(),
                        _ => self.cursor_state = CursorState::SpawnManyRobots,
                    });
                });
            });

        egui::SidePanel::right("right-panel")
            .resizable(true)
            .show_animated(ctx, self.global_opts.focused.is_some(), |ui| {
                let n = self.global_opts.focused.unwrap_or(0);

                let Some(agent) = &self.sim_state.agents.get(n) else {
                    return;
                };

                let robot_state = &agent.state;
                let robot_opts = &mut self.robot_opts[n];

                ui.horizontal(|ui| {
                    ui.heading(format!("Robot [{n}]"));
                });

                ui.separator();
                ui.label(format!("Speed: {:.3}", robot_state.vel));
                ui.label(format!("Angle: {:.3}", robot_state.angle));
                ui.label(format!("Position: {:?}", robot_state.pos));
                ui.separator();

                match self.global_opts.follow {
                    Some(_) => ui.button("Unfollow").clicked().then(|| {
                        self.global_opts.follow = None;
                    }),
                    None => ui.button("Follow").clicked().then(|| {
                        self.global_opts.follow = Some(n);
                    }),
                };

                match self.global_opts.show_only {
                    Some(_) => ui.button("Show All").clicked().then(|| {
                        self.global_opts.show_only = None;
                    }),
                    None => ui.button("Show Only").clicked().then(|| {
                        self.global_opts.show_only = Some(n);
                    }),
                };

                ui.separator();

                ui.heading("Debug Items");

                let mut current_category = "";
                for (n, (category, name, _items)) in agent.robot.get_debug_soup().iter().enumerate()
                {
                    if category != current_category {
                        ui.label(category);
                        current_category = category;
                    }
                    draw_debug_item_label(ui, get_color(n), category, name, robot_opts);
                }

                // if let DebugSoup(Some(soup)) = agent.robot.get_debug_soup() {
                //     let mut n = 0;

                //     if let Some(global_items) = soup.get("") {
                //         for name in global_items.keys() {
                //             draw_debug_item_label(ui, get_color(n), "", *name, robot_opts);
                //             n += 1;
                //         }
                //     }

                //     for (category, items) in soup.iter() {
                //         if category.is_empty() {
                //             continue;
                //         }
                //         ui.label(*category);
                //         for name in items.keys() {
                //             draw_debug_item_label(ui, get_color(n), *category, *name, robot_opts);
                //             n += 1;
                //         }
                //     }
                // }

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
                    let agent_pos = self.sim_state.agents[f].state.pos;
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
                        self.cursor_state = CursorState::default();
                    });

                    bind_pressed!(i; Key::N => {
                        match self.cursor_state {
                            CursorState::SpawnRobot => {
                                self.cursor_state = CursorState::default();
                            }
                            _ => {
                                self.cursor_state = CursorState::SpawnRobot;
                            }
                        }
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
                            self.spawn_robot(pos);
                            self.cursor_state = CursorState::Picking;
                        }
                        CursorState::SpawnManyRobots => {
                            self.spawn_robot(pos);
                        }
                        CursorState::Picking => {
                            // Check if we clicked on a robot
                            let mut found = None;
                            for (n, agent) in self.sim_state.agents.iter().enumerate() {
                                let robot_diameter = agent.robot.params().diameter;
                                if (agent.state.pos - pos).length() < robot_diameter * 0.75 {
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

fn draw_debug_item_label(
    ui: &mut egui::Ui,
    color: Color32,
    category: &'static str,
    name: &'static str,
    robot_opts: &mut RobotOptions,
) {
    ui.horizontal(|ui| {
        let (_, painter) = ui.allocate_painter(
            Vec2::splat(10.0),
            Sense {
                click: false,
                drag: false,
                focusable: false,
            },
        );

        painter.rect_filled(painter.clip_rect(), 0.0, color);

        let mut shown = robot_opts.debug_items.contains(&(category, name));
        let resp = ui.toggle_value(&mut shown, name);
        if resp.changed() {
            match shown {
                true => robot_opts.debug_items.insert((category, name)),
                false => robot_opts.debug_items.remove(&(category, name)),
            };
        }
    });
}

fn get_color(n: usize) -> Color32 {
    let hue = n as f32 * PI / 10.0;
    Hsva::new(hue, 0.8, 0.8, 1.0).into()
}
