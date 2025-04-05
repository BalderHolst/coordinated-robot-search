use opencv::{
    core::{CV_8UC1, CV_8UC3, Point2f, Point2i, Scalar, Vec3f, Vec4d, Vector},
    highgui,
    imgproc::{self, LineTypes},
    prelude::*,
};

use crate::vision::camera_info::CameraInfo;

/// Target color of the search object
const TARGET_COLOR: [f32; 3] = [30.0, 255.0, 255.0];
/// The weights for the weighted distance calculation (must sum to 1.0)
const TARGET_COLOR_WEIGHTS: [f32; 3] = [0.65, 0.25, 0.1];
/// Tolerances for the hue probability
const HUE_TOLERANCE: f32 = 5.0;
/// Tolerances for the saturation probability
const SATURATION_TOLERANCE: f32 = 100.0;
/// Tolerances for the value probability
const VALUE_TOLERANCE: f32 = 150.0;

/// Helper struct for storing intermediate results of vision
struct Circle {
    radius: i32,
    center: Point2i,
    color: Vec3f,
}

pub struct ObjectDetection {
    camera: CameraInfo,

    // For continous use (No reallocation)
    mask: Mat,
    hsv: Mat,
    masked_circles: Mat,
}

impl ObjectDetection {
    /// Contructer that uses Camera to initialize the vision Mat's
    pub fn new(camera: CameraInfo) -> Self {
        let mask = Mat::new_rows_cols_with_default(
            camera.get_pixels_v(),
            camera.get_pixels_h(),
            CV_8UC1,
            Scalar::all(0.0),
        )
        .expect("Mat failed");

        let hsv = Mat::new_rows_cols_with_default(
            camera.get_pixels_v(),
            camera.get_pixels_h(),
            CV_8UC3,
            Scalar::all(0.0),
        )
        .expect("Mat failed");

        // NOTE: Remove (only for debugging)
        let masked_circles = Mat::new_rows_cols_with_default(
            camera.get_pixels_v(),
            camera.get_pixels_h(),
            CV_8UC3,
            Scalar::all(0.0),
        )
        .expect("Mat failed");
        Self {
            camera,
            mask,
            hsv,
            masked_circles,
        }
    }

    pub fn find_search_objects_probability(
        &mut self,
        image: &Mat,
    ) -> Result<botbrain::CamData, String> {
        Self::apply_yellow_mask(self, image)?;

        let contours = self
            .find_contours()
            .map_err(|e| format!("find_contours failed: {}", e))?;

        let search_objects = self
            .find_likely_objects(contours)
            .map_err(|e| format!("find_likely_objects failed: {}", e))?;

        Ok(search_objects)
    }

    /// Stores mask result in self.mask
    fn apply_yellow_mask(&mut self, img: &Mat) -> Result<(), String> {
        // Convert image to HSV for easier color extraction
        imgproc::cvt_color(&img, &mut self.hsv, imgproc::COLOR_RGB2HSV, 0)
            .map_err(|e| format!("cvt_color failed: {}", e))?;

        // Define lower and upper bounds for yellow color
        // Hue is from 0 to 180
        // Yellow is normally 60 degrees of 360 degrees, therefore ~30 degrees here
        let lower_yellow = Scalar::new(28.0, 180.0, 180.0, 0.0);
        let upper_yellow = Scalar::new(32.0, 255.0, 255.0, 0.0);

        // Extract yellow color into a mask
        opencv::core::in_range(&self.hsv, &lower_yellow, &upper_yellow, &mut self.mask)
            .map_err(|e| format!("in_range failed: {}", e))?;

        Ok(())
    }

    fn find_contours(&self) -> Result<Vector<Vector<Point2i>>, String> {
        // Find contours
        let mut contours = Vector::<Vector<Point2i>>::new();
        imgproc::find_contours(
            &self.mask,
            &mut contours,
            imgproc::RETR_EXTERNAL,
            imgproc::CHAIN_APPROX_SIMPLE,
            Point2i::new(0, 0),
        )
        .map_err(|e| format!("find_contours failed: {}", e))?;
        Ok(contours)
    }

    /// Extract the circles and their mean color from the contours
    fn find_likely_objects(
        &mut self,
        contours: Vector<Vector<Point2i>>,
    ) -> Result<botbrain::CamData, String> {
        if contours.is_empty() {
            return Err("No contours found".to_string());
        }
        // INFO: Only for debug
        self.hsv
            .copy_to(&mut self.masked_circles)
            .expect("copy_to failed");
        // Find the largest
        let circles = contours
            .into_iter()
            .map(|c| {
                let mut center = Point2f::new(0.0, 0.0);
                let mut radius = 0.0;
                imgproc::min_enclosing_circle(&c, &mut center, &mut radius)
                    .expect("min_enclosing_circle failed");
                Circle {
                    center: Point2i::new(center.x as i32, center.y as i32),
                    radius: radius as i32,
                    color: Vec3f::default(),
                }
            })
            .filter(|circle| circle.radius > 10)
            .map(|circle| {
                // Reset mask
                self.mask
                    .set_scalar(Scalar::all(0.0))
                    .expect("set_scalar failed");

                // Mask out the circle
                imgproc::circle(
                    &mut self.mask,
                    circle.center,
                    circle.radius,
                    Scalar::all(255.0),
                    -1,
                    LineTypes::LINE_8 as i32,
                    0,
                )
                .expect("circle failed");

                // Find the mean color
                let mean_color: Vec4d =
                    opencv::core::mean(&self.hsv, &self.mask).expect("mean failed");
                Circle {
                    color: Vec3f::from_array([
                        mean_color[0] as f32,
                        mean_color[1] as f32,
                        mean_color[2] as f32,
                    ]),
                    ..circle
                }
            })
            // INFO: Only for debug
            .inspect(|circle| {
                imgproc::circle(
                    &mut self.masked_circles,
                    circle.center,
                    circle.radius + 5,
                    Scalar::from_array([
                        circle.color[0] as f64,
                        circle.color[1] as f64,
                        circle.color[2] as f64,
                        255.0,
                    ]),
                    1,
                    LineTypes::LINE_8 as i32,
                    0,
                )
                .expect("circle failed");
            })
            .collect::<Vec<_>>();

        // INFO: Only for debug
        let mut temp_img = Mat::default();
        imgproc::cvt_color(
            &self.masked_circles,
            &mut temp_img,
            imgproc::COLOR_HSV2BGR,
            0,
        )
        .expect("cvt_color failed");
        highgui::imshow("circles", &temp_img).unwrap();

        if circles.is_empty() {
            Err("No circles found".to_string())
        } else {
            self.convert_to_cam_data(circles)
        }
    }

    fn compute_probability(diff: f32, tolerance: f32, weight: f32) -> f32 {
        if diff.abs() > tolerance {
            0.0
        } else {
            weight * (1.0 - diff.abs() / tolerance)
        }
    }

    fn convert_to_cam_data(&self, circles: Vec<Circle>) -> Result<botbrain::CamData, String> {
        let cam_points: Vec<botbrain::CamPoint> = circles
            .into_iter()
            .flat_map(|circle| {
                let hue_probability = Self::compute_probability(
                    circle.color[0] - TARGET_COLOR[0],
                    HUE_TOLERANCE,
                    TARGET_COLOR_WEIGHTS[0],
                );

                let saturation_probability = Self::compute_probability(
                    circle.color[1] - TARGET_COLOR[1],
                    SATURATION_TOLERANCE,
                    TARGET_COLOR_WEIGHTS[1],
                );

                let value_probability = Self::compute_probability(
                    circle.color[2] - TARGET_COLOR[2],
                    VALUE_TOLERANCE,
                    TARGET_COLOR_WEIGHTS[2],
                );

                let probability = hue_probability + saturation_probability + value_probability;

                let mut cam_points: Vec<botbrain::CamPoint> = vec![];

                // HACK: Update to use cone for object of interest
                for i in (-circle.radius / 2)..(circle.radius / 2) {
                    let angle = -self.camera.get_angle_h(circle.center.x + i);
                    cam_points.push(botbrain::CamPoint { probability, angle });
                }

                cam_points
            })
            .collect();

        Ok(botbrain::CamData::Points(cam_points))
    }
}
