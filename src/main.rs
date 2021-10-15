use opencv::{core, highgui, prelude::*, videoio, Result};
//use opencv::calib3d::prelude::StereoBM;
//use opencv::calib3d;

mod tool;
pub trait Average {
    fn average(&self) -> Result<Vec<Vec<f64>>>;
}
type Rvec = core::Vector<Mat>;
impl Average for Rvec {
    fn average(&self) -> Result<Vec<Vec<f64>>> {
        let temp = self.to_vec();
        let len = temp.len();
        let first = temp[0].to_vec_2d()? as Vec<Vec<f64>>;
        let leny = first.len();
        let lenx = first[0].len();
        let mut output = vec![vec![0.0;lenx];leny];
        for amat in temp {
            let tepp = amat.to_vec_2d()? as Vec<Vec<f64>>;
            for i in 0..leny {
                for j in 0..lenx {
                    output[i][j] += tepp[i][j];
                }
            }
        }
        for i in 0..leny {
            for j in 0..lenx {
                output[i][j]/=len as f64;
            }
        }
        Ok(output)
    }
}
fn main() -> Result<()> {
    let window = "video capture";
    let window1 = "video capture2";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
    highgui::named_window(window1, highgui::WINDOW_AUTOSIZE)?;
    let mut cam0 = videoio::VideoCapture::new(4, videoio::CAP_V4L)?; // 0 is the default camera
    cam0.set(videoio::CAP_PROP_FRAME_WIDTH, 400.0)?;
    cam0.set(videoio::CAP_PROP_FRAME_HEIGHT, 200.0)?;
    let mut cam1 = videoio::VideoCapture::new(6, videoio::CAP_V4L)?; // 0 is the default camera
    cam1.set(videoio::CAP_PROP_FRAME_WIDTH, 400.0)?;
    cam1.set(videoio::CAP_PROP_FRAME_HEIGHT, 200.0)?;
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

    // camera_matrix left , when is used to picture the depth, I need use the first of it
    let mut camera_matrix = Mat::default();
    // like the left one , I need the right one.
    let mut camera_matrix2 = Mat::default();

    // it is distortion. also is needed.
    let mut dist_coeffs = Mat::default();
    let mut dist_coeffs2 = Mat::default();
    // rotation
    let mut rvecs_left = core::Vector::<Mat>::new();
    // transition
    let mut tvecs_left = core::Vector::<Mat>::new();
    let mut rvecs_right = core::Vector::<Mat>::new();
    let mut tvecs_right = core::Vector::<Mat>::new();
    let newcameramtx = tool::newcameramtx(
        &vector2d,
        &vector3d,
        &mut camera_matrix,
        &mut dist_coeffs,
        &mut rvecs_left,
        &mut tvecs_left,
    )?;
    let newcameramtx2 = tool::newcameramtx(
        &vector2d2,
        &vector3d2,
        &mut camera_matrix2,
        &mut dist_coeffs2,
        &mut rvecs_right,
        &mut tvecs_right,
    )?;
    //let temp: Vec<Vec<f64>> = camera_matrix.to_vec_2d()?;
    //println!("{:?}", temp);
    //println!("{:?}",rvecs_left.to_vec()[0].to_vec_2d()? as Vec<Vec<f64>>);
    let window2 = "video capture2";
    let window3 = "video capture3";
    highgui::named_window(window2, highgui::WINDOW_AUTOSIZE)?;
    highgui::named_window(window3, highgui::WINDOW_AUTOSIZE)?;
    let mut cam2 = videoio::VideoCapture::new(4, videoio::CAP_V4L)?;
    cam2.set(videoio::CAP_PROP_FRAME_WIDTH, 400.0)?;
    cam2.set(videoio::CAP_PROP_FRAME_HEIGHT, 200.0)?;
    let mut cam3 = videoio::VideoCapture::new(6, videoio::CAP_V4L)?; // 0 is the default camera
    cam3.set(videoio::CAP_PROP_FRAME_WIDTH, 400.0)?;
    cam3.set(videoio::CAP_PROP_FRAME_HEIGHT, 200.0)?;
    let mut count3: i32 = 0;
    let mut count4: i32 = 0;
    loop {
        tool::new_camera(
            &mut cam2,
            &camera_matrix,
            &dist_coeffs,
            &newcameramtx,
            window2,
            &mut count3,
            "c",
        )?;
        tool::new_camera(
            &mut cam3,
            &camera_matrix2,
            &dist_coeffs2,
            &newcameramtx2,
            window3,
            &mut count4,
            "d",
        )?;
        let key = highgui::wait_key(10)?;
        if key > 0 && key != 255 {
            break;
        }
    }
    cam2.release()?;
    drop(cam2);
    cam3.release()?;
    drop(cam3);
    highgui::destroy_all_windows()?;
    // here export the solution
    println!(
        "dist_coeffs {:?}",
        dist_coeffs.to_vec_2d()? as Vec<Vec<f64>>
    );
    println!(
        "dist_coeffs2 {:?}",
        dist_coeffs2.to_vec_2d()? as Vec<Vec<f64>>
    );
    println!(
        "rvec_left {:?}",
        rvecs_left.average()?
    );
    println!(
        "tvec_left {:?}",
        tvecs_left.average()?
    );
    println!(
        "rvecs_right {:?}",
        rvecs_right.average()?
    );
    println!(
        "tvecs_right {:?}",
        tvecs_right.average()?
    );
    println!(
        "camera_matrix {:?}",
        camera_matrix.to_vec_2d()? as Vec<Vec<f64>>
    );
    println!(
        "camera_matrix2 {:?}",
        camera_matrix2.to_vec_2d()? as Vec<Vec<f64>>
    );
    Ok(())
}
