#[allow(dead_code)]
use crate::camera_info::CameraInfo;
use opencv::{
    core::{CV_8UC1, CV_8UC3, Point2f, Point2i, Scalar, Vec3b, Vec4d, Vector},
    highgui,
    imgproc::{self, LineTypes},
    prelude::*,
};

/// Helper struct for storing intermediate results of vision
struct Circle {
    radius: i32,
    center: Point2i,
}

pub struct Vision {
    camera: CameraInfo,

    // For continous use (No reallocation)
    mask: Mat,
    hsv: Mat,
    masked_circles: Mat,
}

impl Vision {
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

    /// Util function to convert a r2r::sensor_msgs::msg::Image to an opencv::Mat
    pub fn sensor_image_to_opencv_image(
        image: &mut r2r::sensor_msgs::msg::Image,
    ) -> Result<Mat, Box<dyn std::error::Error>> {
        let mat = unsafe {
            Mat::new_rows_cols_with_data_unsafe(
                image.height as i32,
                image.width as i32,
                CV_8UC3,
                image.data.as_mut_ptr() as *mut std::ffi::c_void,
                image.step as usize,
            )
        };
        mat.map_err(|e| e.into())
    }

    pub fn find_search_objects_propability(
        &mut self,
        image: &Mat,
    ) -> Result<botbrain::CamData, String> {
        Self::apply_yellow_mask(self, image);
        if let Ok(contours) = self.find_contours() {
            if let Ok(_seach_objects) = self.find_most_likely_object(contours) {
                // TODO: Return CamData
                Ok(botbrain::CamData::default())
            } else {
                Err("No object found".to_string())
            }
        } else {
            Err("No contours found".to_string())
        }
    }

    /// Stores mask result in self.mask
    fn apply_yellow_mask(&mut self, img: &Mat) {
        // TODO: Return result?
        // Convert image to HSV for easier color extraction
        imgproc::cvt_color(&img, &mut self.hsv, imgproc::COLOR_RGB2HSV, 0)
            .expect("cvt_color failed");

        // Define lower and upper bounds for yellow color
        // Hue is from 0 to 180
        // Yellow is normally 60 degrees of 360 degrees, therefore ~30 degrees here
        let lower_yellow = Scalar::new(28.0, 180.0, 180.0, 0.0);
        let upper_yellow = Scalar::new(32.0, 255.0, 255.0, 0.0);

        // Extract yellow color into a mask
        opencv::core::in_range(&self.hsv, &lower_yellow, &upper_yellow, &mut self.mask)
            .expect("in_range failed");
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
        .expect("find_contours failed");
        Ok(contours)
    }

    fn find_most_likely_object(
        &mut self,
        contours: Vector<Vector<Point2i>>,
    ) -> Result<Circle, String> {
        // TODO: Return CamData
        if contours.is_empty() {
            return Err("No contours found".to_string());
        }
        self.hsv
            .copy_to(&mut self.masked_circles)
            .expect("copy_to failed");
        // Find the largest
        let best_circle = contours
            .iter()
            .map(|c| {
                let mut center = Point2f::new(0.0, 0.0);
                let mut radius = 0.0;
                imgproc::min_enclosing_circle(&c, &mut center, &mut radius)
                    .expect("min_enclosing_circle failed");
                (center, radius)
            })
            .filter(|(_center, radius)| *radius > 10.0)
            .map(|(center, radius)| {
                // Reset mask
                self.mask
                    .set_scalar(Scalar::all(0.0))
                    .expect("set_scalar failed");

                // Mask out the circle
                imgproc::circle(
                    &mut self.mask,
                    Point2i::new(center.x as i32, center.y as i32),
                    radius as i32,
                    Scalar::all(255.0),
                    -1,
                    LineTypes::LINE_8 as i32,
                    0,
                )
                .expect("circle failed");

                // Find the mean color
                let mean_color: Vec4d =
                    opencv::core::mean(&self.hsv, &self.mask).expect("mean failed");
                (center, radius, mean_color)
            })
            .inspect(|(center, radius, color)| {
                imgproc::circle(
                    &mut self.masked_circles,
                    Point2i::new(center.x as i32, center.y as i32),
                    *radius as i32 + 5,
                    Scalar::from_array([color[0], color[1], color[2], 255.0]),
                    1,
                    LineTypes::LINE_8 as i32,
                    0,
                )
                .expect("circle failed");
                println!("Color: {:?}", color);
            })
            .min_by_key(|(_center, _radius, color)| {
                let target_color = Vec3b::from_array([30, 255, 255]); // HSV color
                target_color
                    .iter()
                    .zip(color.iter())
                    .map(|(&a, &b)| (a as i16 - b as i16).abs())
                    .sum::<i16>()
            });
        let mut temp_img = Mat::default();
        imgproc::cvt_color(
            &self.masked_circles,
            &mut temp_img,
            imgproc::COLOR_HSV2BGR,
            0,
        )
        .expect("cvt_color failed");

        highgui::imshow("circles", &temp_img).unwrap();

        // Return the best circle
        if let Some((center, radius, _color)) = best_circle {
            Ok(Circle {
                center: Point2i::new(center.x as i32, center.y as i32),
                radius: radius as i32,
            })
        } else {
            Err("No circle found".to_string())
        }
    }
}
