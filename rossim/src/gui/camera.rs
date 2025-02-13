use eframe::egui::{Key, Pos2, Rect, Response, Ui, Vec2};

use crate::{bind_down, bind_pressed};

#[derive(Debug)]
pub struct Camera {
    pub pos: Pos2,
    home: Pos2,
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

    pub fn update(&mut self, ui: &Ui, resp: &Response) -> bool {
        let mut moved = false;

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
            moved = true;
            let delta = self.canvas_to_rel(resp.drag_delta());
            self.pos -= delta;
        }

        // Keys
        ui.input(|i| {
            let mut velocity = Vec2::ZERO;
            bind_down!(i; Key::ArrowRight, Key::D => velocity.x += 1.0);
            bind_down!(i; Key::ArrowLeft,  Key::A => velocity.x -= 1.0);
            bind_down!(i; Key::ArrowUp,    Key::W => velocity.y -= 1.0);
            bind_down!(i; Key::ArrowDown,  Key::S => velocity.y += 1.0);

            bind_pressed!(i; Key::H => {
                moved = true;
                self.home()
            });

            if velocity != Vec2::ZERO {
                moved = true;
                self.pos += velocity * 0.001 * self.scale;
            }
        });

        moved
    }
}
