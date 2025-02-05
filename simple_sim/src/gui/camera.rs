use eframe::egui::{Key, Pos2, Rect, Response, Ui, Vec2};

#[derive(Debug)]
pub struct Camera {
    home: Pos2,
    pos: Pos2,
    scale: f32,
    viewport: Rect,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            home: Pos2::ZERO,
            pos: Pos2::ZERO,
            viewport: Rect::ZERO,
            scale: 100.0,
        }
    }
}

impl Camera {
    pub fn new(pos: Pos2) -> Self {
        Self {
            home: pos,
            pos,
            ..Default::default()
        }
    }

    pub fn home(&mut self) {
        self.pos = self.home;
    }

    pub fn origin(&self) -> Pos2 {
        self.world_to_viewport(Pos2::ZERO)
    }

    pub fn scaled<X>(&self, x: X) -> X
    where
        X: std::ops::Mul<f32, Output = X>,
    {
        x * self.scale
    }

    pub fn world_to_viewport(&self, world_pos: Pos2) -> Pos2 {
        let rel_pos = world_pos - self.pos;
        self.viewport.center() + rel_pos * self.scale
    }

    pub fn canvas_to_world(&self, canvas_pos: Pos2) -> Pos2 {
        let rel_pos = (canvas_pos - self.viewport.center()) / self.scale;

        self.pos + rel_pos
    }

    pub fn canvas_to_rel(&self, canvas_pos: Vec2) -> Vec2 {
        canvas_pos / self.scale
    }

    pub fn set_viewport(&mut self, viewport: Rect) {
        self.viewport = viewport;
    }

    pub fn update(&mut self, ui: &Ui, resp: &Response) {
        // Zoom
        if resp.hovered() {
            ui.input(|i| {
                let delta_scale = i.smooth_scroll_delta.y;
                let delta_scale = 0.01 * delta_scale * self.scale;
                self.scale += delta_scale;
                self.scale = self.scale.clamp(10.0, 500.0);
            });
        }

        // Drag view
        if resp.dragged() {
            let delta = self.canvas_to_rel(resp.drag_delta());
            self.pos -= delta;
        }

        // Keys
        ui.input(|i| {
            macro_rules! bind {
                ($bind:tt, $($key:expr),+ => $e:expr) => {{
                    if $(i.$bind($key))|+ {
                        $e;
                    }
                }}
            }
            macro_rules! bind_down {
                ($($key:expr),+ => $e:expr) => {bind!(key_down, $($key),+ => $e)}
            }
            macro_rules! bind_pressed {
                ($($key:expr),+ => $e:expr) => {bind!(key_pressed, $($key),+ => $e)}
            }

            let mut velocity = Vec2::ZERO;
            bind_down!(Key::ArrowRight, Key::D => velocity.x += 1.0);
            bind_down!(Key::ArrowLeft,  Key::A => velocity.x -= 1.0);
            bind_down!(Key::ArrowUp,    Key::W => velocity.y -= 1.0);
            bind_down!(Key::ArrowDown,  Key::S => velocity.y += 1.0);

            bind_pressed!(Key::H => self.home());

            if velocity != Vec2::ZERO {
                self.pos += velocity * 0.001 * self.scale;
            }
        });
    }
}
