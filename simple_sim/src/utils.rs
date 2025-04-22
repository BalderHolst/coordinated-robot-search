use eframe::egui::{Color32, ColorImage};

pub fn grid_to_image<C: Clone + Default>(
    grid: &botbrain::grid::Grid<C>,
    color: impl Fn(C) -> Color32,
) -> ColorImage {
    let mut image = ColorImage::new([grid.width(), grid.height()], color(C::default()));
    image.pixels.iter_mut().enumerate().for_each(|(i, pixel)| {
        let (x, y) = (i % grid.width(), i / grid.width());
        let cell = grid.get(x, y).cloned().unwrap_or_default();
        *pixel = color(cell);
    });
    image
}
