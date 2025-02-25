use eframe::egui::{Key, Pos2, Rect, Response, Ui, Vec2};

use crate::{bind_down, bind_pressed};

#[derive(Clone, Debug)]
struct CameraView {
    pub pos: Pos2,
    pub scale: f32,
}

impl Default for CameraView {
    fn default() -> Self {
        Self {
            pos: Pos2::ZERO,
            scale: 100.0,
        }
    }
}

#[derive(Debug)]
pub struct Camera {
    view: CameraView,
    home: CameraView,
    viewport: Rect,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            view: CameraView::default(),
            home: CameraView::default(),
            viewport: Rect::ZERO,
        }
    }
}

impl Camera {
    pub fn new(pos: Pos2, scale: f32) -> Self {
        let view = CameraView { pos, scale };
        Self {
            view: view.clone(),
            home: view,
            ..Default::default()
        }
    }

    pub fn pos(&self) -> Pos2 {
        self.view.pos
    }

    pub fn set_pos(&mut self, pos: Pos2) {
        self.view.pos = pos;
    }

    pub fn home(&mut self) {
        self.view = self.home.clone();
    }

    pub fn origin(&self) -> Pos2 {
        self.world_to_viewport(Pos2::ZERO)
    }

    pub fn scaled<X>(&self, x: X) -> X
    where
        X: std::ops::Mul<f32, Output = X>,
    {
        x * self.view.scale
    }

    pub fn world_to_viewport(&self, world_pos: Pos2) -> Pos2 {
        let rel_pos = world_pos - self.view.pos;
        self.viewport.center() + rel_pos * self.view.scale
    }

    pub fn canvas_to_world(&self, canvas_pos: Pos2) -> Pos2 {
        let rel_pos = (canvas_pos - self.viewport.center()) / self.view.scale;

        self.view.pos + rel_pos
    }

    pub fn canvas_to_rel(&self, canvas_pos: Vec2) -> Vec2 {
        canvas_pos / self.view.scale
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
                let delta_scale = 0.01 * delta_scale * self.view.scale;
                self.view.scale += delta_scale;
            });
        }

        // Drag view
        if resp.dragged() {
            moved = true;
            let delta = self.canvas_to_rel(resp.drag_delta());
            self.view.pos -= delta;
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
                self.view.pos += velocity * 0.001 * self.view.scale;
            }
        });

        moved
    }
}
