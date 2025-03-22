use opencv::{
    core::{CV_8UC1, CV_8UC3, Point2i, Scalar, Vec3b, VecN, Vector},
    imgproc::{self, LineTypes},
    prelude::*,
};

pub struct Camera {
    /// Horizontal field of view in radians
    _fov_h: f32,
    /// Vertical field of view in radians
    _fov_v: f32,
    /// Pixels in vertical direction
    /// Typically, this is the height of the video
    pixels_v: i32,
    /// Pixels in horizontal direction
    /// Typically, this is the width of the video
    pixels_h: i32,
    /// Constant for horizontal angle calculation
    b_h: f32,
    /// Constant for vertical angle calculation
    b_v: f32,
}

impl Camera {
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
}

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

pub fn find_search_objects(image: &Mat) -> Result<botbrain::CamData, String> {
    let masked_img = apply_yellow_mask(image);
    if let Ok(contours) = find_contours(&masked_img) {
        if let Ok(seach_objects) = find_most_likely_object(contours, &masked_img) {
            // TODO: Return CamData
        }
    } else {
        return Err("No contours found".to_string());
    }
    todo!();
}

fn apply_yellow_mask(img: &Mat) -> Mat {
    // Convert to HSV
    let mut hsv = Mat::default();
    imgproc::cvt_color(&img, &mut hsv, imgproc::COLOR_RGB2HSV, 0).expect("cvt_color failed");

    // Define lower and upper bounds for yellow color
    let lower_yellow = Scalar::new(28.0, 180.0, 200.0, 0.0);
    let upper_yellow = Scalar::new(32.0, 255.0, 255.0, 0.0);

    // Create a mask
    let mut mask =
        Mat::new_rows_cols_with_default(img.rows(), img.cols(), CV_8UC1, Scalar::all(0.0))
            .expect("Mat failed");
    opencv::core::in_range(&hsv, &lower_yellow, &upper_yellow, &mut mask).expect("in_range failed");

    // Extract yellow balls using bitwise operation
    let mut result =
        Mat::new_rows_cols_with_default(img.rows(), img.cols(), CV_8UC3, Scalar::all(0.0))
            .expect("Mat failed");
    opencv::core::bitwise_and(&img, &img, &mut result, &mask).expect("bitwise_and failed");
    mask
}

pub fn find_contours(img_with_mask: &Mat) -> Result<Vector<Vector<Point2i>>, String> {
    // Find contours
    let mut contours = Vector::<Vector<Point2i>>::new();
    imgproc::find_contours(
        &img_with_mask,
        &mut contours,
        imgproc::RETR_EXTERNAL,
        imgproc::CHAIN_APPROX_SIMPLE,
        opencv::core::Point::new(0, 0),
    )
    .expect("find_contours failed");
    Ok(contours)
}

struct Circle {
    radius: i32,
    center: Point2i,
}

fn find_most_likely_object(contours: Vector<Vector<Point2i>>, img: &Mat) -> Result<Circle, String> {
    // TODO: Return CamData
    if contours.is_empty() {
        return Err("No contours found".to_string());
    }
    // Find the largest
    let best_circle = contours
        .iter()
        .map(|c| {
            let mut center = opencv::core::Point2f::new(0.0, 0.0);
            let mut radius = 0.0;
            imgproc::min_enclosing_circle(&c, &mut center, &mut radius)
                .expect("min_enclosing_circle failed");
            (center, radius)
        })
        .map(|(center, radius)| {
            // Make a mask
            let mut mask = opencv::core::Mat::new_rows_cols_with_default(
                img.rows(),
                img.cols(),
                CV_8UC1,
                Scalar::all(0.0),
            )
            .expect("Mat failed");

            // Mask out the circle
            imgproc::circle(
                &mut mask,
                Point2i::new(center.x as i32, center.y as i32),
                radius as i32,
                Scalar::all(255.0),
                -1,
                LineTypes::LINE_8 as i32,
                0,
            )
            .expect("circle failed");

            // Find the mean color
            let mean_color: VecN<f64, 4> = opencv::core::mean(&img, &mask).expect("mean failed");
            // let mean_color: VecN<u8, 4> = mean_color.to().unwrap_or_default();
            let color = Mat::new_rows_cols_with_default(
                1,
                1,
                CV_8UC3,
                Scalar::from_array([mean_color[0], mean_color[1], mean_color[2], 255.0]),
            )
            .expect("Mat failed");

            let mut final_color = Mat::new_rows_cols_with_default(
                1,
                1,
                CV_8UC3,
                Scalar::from_array([0.0, 0.0, 0.0, 255.0]),
            )
            .expect("Mat failed");

            imgproc::cvt_color(&color, &mut final_color, imgproc::COLOR_BGR2HSV, 0)
                .expect("cvt_color failed");
            let final_mean_color = final_color.at_2d::<Vec3b>(0, 0).expect("at failed");
            (center, radius, final_mean_color.to_owned())
        })
        .min_by_key(|(_center, _radius, color)| {
            let target_color = Vec3b::from_array([30, 255, 255]); // HSV color
            target_color
                .iter()
                .zip(color.iter())
                .map(|(&a, &b)| (a as i16 - b as i16).abs())
                .sum::<i16>()
        });

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
