#![allow(dead_code)]

/// Struct for storing camera info
pub struct CameraInfo {
    /// Horizontal field of view in radians
    _fov_h: f32,
    /// Vertical field of view in radians
    _fov_v: f32,
    /// Pixels in horizontal direction
    /// Typically, this is the width of the video
    pixels_h: i32,
    /// Pixels in vertical direction
    /// Typically, this is the height of the video
    pixels_v: i32,
    /// Constant for horizontal angle calculation
    b_h: f32,
    /// Constant for vertical angle calculation
    b_v: f32,
}

impl CameraInfo {
    pub fn new(pixels_h: i32, pixels_v: i32, fov_h: f32, fov_v: f32) -> Self {
        let b_h = ((pixels_h as f32) / 2.0) / f32::tan(fov_h / 2.0);
        let b_v = ((pixels_v as f32) / 2.0) / f32::tan(fov_v / 2.0);

        Self {
            _fov_h: fov_h,
            _fov_v: fov_v,
            pixels_v,
            pixels_h,
            b_h,
            b_v,
        }
    }

    /// Return the horizontal angle in radians
    pub fn get_angle_h(&self, pixel_h: i32) -> f32 {
        let pixels_to_h_center = pixel_h - (self.pixels_h / 2);
        f32::atan(pixels_to_h_center as f32 / self.b_h)
    }

    /// Return the vertical angle in radians
    pub fn get_angle_v(&self, pixel_v: i32) -> f32 {
        let pixels_to_v_center = pixel_v - (self.pixels_v / 2);
        f32::atan(pixels_to_v_center as f32 / self.b_v)
    }

    /// Return the angles in radians (horizontal, vertical)
    pub fn get_angles(&self, pixel_h: i32, pixel_v: i32) -> (f32, f32) {
        (self.get_angle_h(pixel_h), self.get_angle_v(pixel_v))
    }

    pub fn get_pixels_h(&self) -> i32 {
        self.pixels_h
    }

    pub fn get_pixels_v(&self) -> i32 {
        self.pixels_v
    }
}
