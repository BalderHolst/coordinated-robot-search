use camera::Camera;
use opencv::{
    core::{
        AlgorithmHint, CV_8UC1, CV_8UC3, Mat, MatTraitConst, Point2i, Scalar, Vec3b, Vec3d, Vec3f,
        Vec3i, Vec4f, VecN, Vector,
    },
    gapi::Circle,
    highgui::{self, WindowFlags},
    imgproc::{self, FONT_HERSHEY_SIMPLEX, LineTypes},
};
use rand::{Rng, distr::Uniform};

mod camera;

const WIDTH: usize = 800;
const HEIGHT: usize = 600;

// TODO: Make conversion function from ros2 to opencv (use impl ToInputArray from opencv)
// sensor_msgs/msg/Image
// sensor_msgs/msg/CameraInfo - This is more relevant for non-simulation cameras

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Setup windows
    highgui::named_window("Original", WindowFlags::WINDOW_NORMAL as i32).expect("imshow failed");
    highgui::named_window("Masked", WindowFlags::WINDOW_NORMAL as i32).expect("imshow failed");
    highgui::move_window("Masked", WIDTH as i32, 0).expect("imshow failed");

    // Pixels to angles conversion example
    let cam = Camera::new(
        WIDTH as i32,
        HEIGHT as i32,
        f32::to_radians(70.0),
        f32::to_radians(40.0),
    );

    loop {
        let img = make_test_img();

        // Search for the object
        let mask = make_yellow_mask(&img);

        // Extract yellow balls using bitwise operation
        let mut result =
            Mat::new_rows_cols_with_default(img.rows(), img.cols(), CV_8UC3, Scalar::all(0.0))
                .expect("Mat failed");
        opencv::core::bitwise_and(&img, &img, &mut result, &mask).expect("bitwise_and failed");

        // Find contours
        let mut contours = Vector::<Vector<Point2i>>::new();
        imgproc::find_contours(
            &mask,
            &mut contours,
            imgproc::RETR_EXTERNAL,
            imgproc::CHAIN_APPROX_SIMPLE,
            opencv::core::Point::new(0, 0),
        )
        .expect("find_contours failed");
        // Draw bounding circle if any contours found
        if let Ok(circle) = find_most_likely_object(contours, &result) {
            imgproc::circle(
                &mut result,
                circle.center,
                circle.radius + 5,
                Scalar::new(255.0, 0.0, 0.0, 0.0),
                2,
                imgproc::LINE_AA,
                0,
            )
            .expect("circle failed");

            imgproc::put_text(
                &mut result,
                &format!("Ball center: ({},{})", circle.center.x, circle.center.y),
                Point2i::new(20, 50),
                FONT_HERSHEY_SIMPLEX,
                0.8,
                Scalar::all(255.0),
                1,
                imgproc::LineTypes::LINE_8 as i32,
                false,
            )
            .expect("put_text failed");
            let angles = cam.get_angles(circle.center.x, circle.center.y);
            imgproc::put_text(
                &mut result,
                &format!(
                    "Ball angle: ({},{})",
                    f32::to_degrees(angles.0),
                    f32::to_degrees(angles.1)
                ),
                Point2i::new(20, 100),
                FONT_HERSHEY_SIMPLEX,
                0.8,
                Scalar::all(255.0),
                1,
                imgproc::LineTypes::LINE_8 as i32,
                false,
            )
            .expect("put_text failed");

            // Show images
            highgui::imshow("Masked", &result).expect("imshow failed");
        }
        highgui::imshow("Original", &img).expect("imshow failed");

        // Must be called to show images
        if highgui::wait_key(0).expect("wait_key failed") == ('q' as i32) {
            break;
        }
    }
    Ok(())
}

fn find_most_likely_object(
    contours: Vector<Vector<Point2i>>,
    img: &Mat,
) -> Result<opencv::gapi::Circle, String> {
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

            imgproc::cvt_color(
                &color,
                &mut final_color,
                imgproc::COLOR_BGR2HSV,
                0,
                AlgorithmHint::ALGO_HINT_DEFAULT,
            )
            .expect("cvt_color failed");
            let final_mean_color = final_color.at_2d::<Vec3b>(0, 0).expect("at failed");
            (center, radius, final_mean_color.to_owned())
        })
        .min_by_key(|(_center, _radius, color)| {
            let target_color = Vec3b::from_array([30, 255, 255]); // HSV color
            target_color
                .into_iter()
                .zip(color.into_iter())
                .map(|(a, b)| (a as i16 - b as i16).abs())
                .sum::<i16>()
        });

    // Return the best circle
    if let Some((center, radius, _color)) = best_circle {
        Ok(Circle::new_def(
            Point2i::new(center.x as i32, center.y as i32),
            radius as i32,
            Scalar::default(),
        )
        .expect("Circle failed"))
    } else {
        Err("No circle found".to_string())
    }
}

fn make_yellow_mask(img: &Mat) -> Mat {
    // Convert to HSV
    let mut hsv = Mat::default();
    imgproc::cvt_color(
        &img,
        &mut hsv,
        imgproc::COLOR_BGR2HSV,
        0,
        AlgorithmHint::ALGO_HINT_DEFAULT,
    )
    .expect("cvt_color failed");

    // Define lower and upper bounds for yellow color
    let lower_yellow = Scalar::new(28.0, 180.0, 200.0, 0.0);
    let upper_yellow = Scalar::new(32.0, 255.0, 255.0, 0.0);

    // Create a mask
    let mut mask =
        Mat::new_rows_cols_with_default(img.rows(), img.cols(), CV_8UC1, Scalar::all(0.0))
            .expect("Mat failed");
    opencv::core::in_range(&hsv, &lower_yellow, &upper_yellow, &mut mask).expect("in_range failed");
    mask
}

/// Create a random image with a search object
fn make_test_img() -> Mat {
    // Create a test canvas
    let mut img =
        Mat::new_rows_cols_with_default(HEIGHT as i32, WIDTH as i32, CV_8UC3, Scalar::all(60.0))
            .expect("Mat failed");

    // Create noise in the image
    let mut rng = rand::rng();
    let width_range = Uniform::try_from(0..WIDTH).expect("Uniform failed");
    let height_range = Uniform::try_from(0..HEIGHT).expect("Uniform failed");
    let radius_range = Uniform::try_from(5..50).expect("Uniform failed");
    for _ in 0..100 {
        // Generate random values
        let width = rng.sample(width_range);
        let height = rng.sample(height_range);
        let radius = rng.sample(radius_range);
        let color = Vec3d::from_array([
            rng.random::<u8>() as f64,
            rng.random::<u8>() as f64,
            rng.random::<u8>() as f64,
        ]);
        // Draw the circle
        imgproc::circle(
            &mut img,
            Point2i::new(width as i32, height as i32),
            radius,
            Scalar::from_array([color[0], color[1], color[2], 255.0]),
            -1,
            imgproc::LineTypes::LINE_8 as i32,
            0,
        )
        .expect("circle failed");
    }
    let width = rng.sample(width_range);
    let height = rng.sample(height_range);
    let radius = rng.sample(radius_range);

    // Create the search object
    imgproc::circle(
        &mut img,
        Point2i::new(width as i32, height as i32),
        radius,
        Scalar::from_array([0.0, 255.0, 255.0, 255.0]), // Yellow
        -1,
        imgproc::LineTypes::LINE_8 as i32,
        0,
    )
    .expect("circle failed");
    img
}
