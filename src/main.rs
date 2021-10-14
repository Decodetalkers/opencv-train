use opencv::{core, highgui, prelude::*, videoio, Result};
//use opencv::calib3d::prelude::StereoBM;
use opencv::calib3d;
mod tool;
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
    let temp: Vec<Vec<f64>> = camera_matrix.to_vec_2d()?;
    println!("{:?}", temp);
    println!("{:?}",rvecs_left.to_vec()[0].to_vec_2d()? as Vec<Vec<f64>>);
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
    let om = (&rvecs_left.to_vec()[0]).clone();
    // om after transport 
    let mut r = Mat::default();
    let mut temp = Mat::default();
    opencv::calib3d::rodrigues(&om, &mut r, &mut temp)?;
    let t = (&tvecs_left.to_vec()[0]).clone();
    let size = core::Size2i {
        width:640,
        height: 480,
    };
    let mut r1 = Mat::default();
    let mut r2 = Mat::default();
    let mut p1 = Mat::default();
    let mut p2 = Mat::default();
    let mut q = Mat::default();
    let mut valid_pix_roi1 = core::Rect::default();
    let mut valid_pix_roi2 = core::Rect::default();
    opencv::calib3d::stereo_rectify(
        &camera_matrix,
        &dist_coeffs,
        &camera_matrix2,
        &dist_coeffs2,
        size.clone(),
        &r,
        &t,
        &mut r1,
        &mut r2,
        &mut p1,
        &mut p2,
        &mut q,
        opencv::calib3d::CALIB_ZERO_DISPARITY,
        -1.0, 
        size.clone(),
        &mut valid_pix_roi1,
        &mut valid_pix_roi2
    )?;
    let mut left_map1 = Mat::default();
    let mut left_map2 = Mat::default();
    opencv::calib3d::init_undistort_rectify_map(
        &camera_matrix,
        &dist_coeffs, 
        &r1, 
        &p1, 
        size.clone(),
        core::CV_16SC2,
        &mut left_map1,
        &mut left_map2
    )?;
    let mut right_map1 = Mat::default();
    let mut right_map2 = Mat::default();
    opencv::calib3d::init_undistort_rectify_map(
        &camera_matrix2,
        &dist_coeffs2, 
        &r2, 
        &p2, 
        size,
        core::CV_16SC2,
        &mut right_map1,
        &mut right_map2
    )?;

    // deepth
    let mut cam2 = videoio::VideoCapture::new(4, videoio::CAP_V4L)?;
    cam2.set(videoio::CAP_PROP_FRAME_WIDTH, 400.0)?;
    cam2.set(videoio::CAP_PROP_FRAME_HEIGHT, 200.0)?;
    let mut cam3 = videoio::VideoCapture::new(6, videoio::CAP_V4L)?; // 0 is the default camera
    cam3.set(videoio::CAP_PROP_FRAME_WIDTH, 400.0)?;
    cam3.set(videoio::CAP_PROP_FRAME_HEIGHT, 200.0)?;
            let window = "center";
            highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;

    loop {
        let mut frame1 = Mat::default();
        let mut frame2 = Mat::default();
        cam2.read(&mut frame1)?;
        cam3.read(&mut frame2)?;
        if frame1.size()?.width >0 && frame2.size()?.width>0 {
            let mut img1_rectified = Mat::default();
            let mut img2_rectified = Mat::default();
            //println!("test");
            opencv::imgproc::remap(
                &frame1,
                &mut img1_rectified,
                &left_map1,
                &left_map2,
                opencv::imgproc::INTER_LINEAR,
                0,
                core::Scalar::default()
            )?;
            opencv::imgproc::remap(
                &frame2,
                &mut img2_rectified,
                &right_map1,
                &right_map2,
                opencv::imgproc::INTER_LINEAR,
                0,
                core::Scalar::default()
            )?;
            
            //println!("test");
            let mut gray_img1 = Mat::default();
            let mut gray_img2 = Mat::default();
            opencv::imgproc::cvt_color(
				&img1_rectified,
				&mut gray_img1,
				opencv::imgproc::COLOR_BGR2GRAY,
				0,
			)?;
            opencv::imgproc::cvt_color(
				&img2_rectified,
				&mut gray_img2,
				opencv::imgproc::COLOR_BGR2GRAY,
				0,
			)?;
            //println!("test");
            let mut stereo = <dyn calib3d::StereoBM>::create(6,21)?;
            let mut disparity = Mat::default();
            //println!("test");
            stereo.compute(&gray_img1, &gray_img2, &mut disparity)?;
            let mut disp = Mat::default();
            let mut mask = Mat::default();
            //println!("test");
            core::normalize(
                &disparity,
                &mut disp,
                0.0,
                5.0,
                core::NORM_MINMAX,
                core::CV_8U,
                &mut mask
            )?;
            //let mut output = core::Vector::<Mat>::new();
            let mut output = Mat::default();
            //let mut output2 = Mat::default();
            calib3d::reproject_image_to_3d(
                &disp,
                &mut output,
                &q,
                false, 
                3
            )?;
            //output.convert_to(&mut output2, core::CV_16SC1, 0.0, 0.0)?;
            //let a = output.at_2d(3, 3)? as &(i32,i32,i32);
            //println!("{:?}",a);
            let show :Vec<Vec<core::Vec3s>>= output.to_vec_2d()?;
            highgui::set_mouse_callback(window, Some(Box::new(move |x,y,z,_| {
               if x == 1 {
                   println!("{},{}",y,z);
                   println!("{:?}",show[x as usize][y as usize]);
               }
            })))?;
            highgui::imshow("first", &gray_img1)?;
            highgui::imshow("second", &gray_img2)?;
            highgui::imshow(window, &disp)?;
        }
        let key = highgui::wait_key(10)?;
        if key>0 && key !=255 {
            break;
        }
    }
    Ok(())
}
