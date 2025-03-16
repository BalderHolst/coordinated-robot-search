use opencv::{
    core::{AlgorithmHint, CV_8UC4, Mat, Point2i, Scalar, Vec4d},
    highgui, imgproc,
};
use rand::{Rng, distr::Uniform};

const WIDTH: usize = 800;
const HEIGHT: usize = 600;

// TODO: Make conversion function from ros2 to opencv
// sensor_msgs/msg/Image
// sensor_msgs/msg/CameraInfo - This is more relevant for non-simulation cameras

// TODO: Simple detection of e.g. tennis ball (yellow circle)

fn main() {
    let img = make_test_img();

    // Search for the object
    // Convert to HSV
    let mut hsv = Mat::default();
    imgproc::cvt_color(
        &img,
        &mut hsv,
        imgproc::COLOR_BGR2HLS,
        0,
        AlgorithmHint::ALGO_HINT_DEFAULT,
    )
    .expect("cvt_color failed");

    // Define lower and upper bounds for yellow color
    let lower_yellow = Scalar::new(20.0, 100.0, 100.0, 0.0);
    let upper_yellow = Scalar::new(30.0, 255.0, 255.0, 0.0);

    // Create a mask
    let mut mask = Mat::default();
    opencv::core::in_range(&hsv, &lower_yellow, &upper_yellow, &mut mask).expect("in_range failed");

    // Extract yellow ball using bitwise operation
    let mut result = Mat::default();
    opencv::core::bitwise_and(&img, &img, &mut result, &mask).expect("bitwise_and failed");

    // Show images
    highgui::imshow("Original", &img).expect("imshow failed");
    highgui::imshow("Masked", &result).expect("imshow failed");
    highgui::wait_key(0).expect("wait_key failed");
}

/// Create a random image with a search object
fn make_test_img() -> Mat {
    // Create a test canvas
    let mut img =
        Mat::new_rows_cols_with_default(HEIGHT as i32, WIDTH as i32, CV_8UC4, Scalar::all(60.0))
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
        let color = Vec4d::from_array([
            rng.random::<u8>() as f64,
            rng.random::<u8>() as f64,
            rng.random::<u8>() as f64,
            150.0,
        ]);
        // Draw the circle
        imgproc::circle(
            &mut img,
            Point2i::new(width as i32, height as i32),
            radius,
            Scalar::from_array([color[0], color[1], color[2], color[3]]),
            -1,
            imgproc::LineTypes::LINE_8 as i32,
            0,
        )
        .expect("circle failed");
    }

    // Create the search object
    imgproc::circle(
        &mut img,
        Point2i::new(250, 250),
        10,
        Scalar::from_array([0.0, 255.0, 255.0, 255.0]), // Yellow
        -1,
        imgproc::LineTypes::LINE_8 as i32,
        0,
    )
    .expect("circle failed");
    img
}
