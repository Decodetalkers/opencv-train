use opencv::{core, highgui, prelude::*, videoio, Result};
mod tool;
fn main() -> Result<()> {
    let window = "video capture";
    let window1 = "video capture2";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
    highgui::named_window(window1, highgui::WINDOW_AUTOSIZE)?;
    let mut cam0 = videoio::VideoCapture::new_default(4)?; // 0 is the default camera
    let mut cam1 = videoio::VideoCapture::new_default(6)?; // 0 is the default camera

    let mut vector2d = core::Vector::<core::Vector<core::Point2f>>::new();
    let mut vector2d2 = core::Vector::<core::Vector<core::Point2f>>::new();
    let mut vector3d = core::Vector::<core::Vector<core::Point3f>>::new();
    let mut vector3d2 = core::Vector::<core::Vector<core::Point3f>>::new();
    // every 1 second loop once
    let mut count1: i32 = 0;
    let mut count2: i32 = 0;
    loop {
        tool::create_vec_message(
            &mut vector2d2,
            &mut vector3d2,
            &mut cam0,
            window,
            "a",
            &mut count1,
        )?;
        tool::create_vec_message(
            &mut vector2d,
            &mut vector3d,
            &mut cam1,
            window1,
            "b",
            &mut count2,
        )?;
        let key = highgui::wait_key(10)?;
        if key > 0 && key != 255 {
            break;
        }
    }
    // 释放cam0
    cam0.release()?;
    drop(cam0);
    cam1.release()?;
    drop(cam1);

    // 清除所有窗口
    highgui::destroy_all_windows()?;
    println!("out");

    let mut camera_matrix = Mat::default();
    let mut camera_matrix2 = Mat::default();
    let mut dist_coeffs = Mat::default();
    let mut dist_coeffs2 = Mat::default();
    let newcameramtx =
        tool::newcameramtx(&vector2d, &vector3d, &mut camera_matrix, &mut dist_coeffs)?;
    let newcameramtx2 = tool::newcameramtx(
        &vector2d2,
        &vector3d2,
        &mut camera_matrix2,
        &mut dist_coeffs2,
    )?;
    let window2 = "video capture2";
    let window3 = "video capture3";
    highgui::named_window(window2, highgui::WINDOW_AUTOSIZE)?;
    highgui::named_window(window3, highgui::WINDOW_AUTOSIZE)?;
    let mut cam2 = videoio::VideoCapture::new_default(4)?; // 0 is the default camera
    let mut cam3 = videoio::VideoCapture::new_default(6)?; // 0 is the default camera
    loop {
        tool::new_camera(
            &mut cam2,
            &camera_matrix,
            &dist_coeffs,
            &newcameramtx,
            window2,
        )?;
        tool::new_camera(
            &mut cam3,
            &camera_matrix2,
            &dist_coeffs2,
            &newcameramtx2,
            window3,
        )?;
        let key = highgui::wait_key(10)?;
        if key > 0 && key != 255 {
            break;
        }
    }
    Ok(())
}
