use opencv::{Result, calib3d, core, features2d, highgui, imgproc, prelude::*, videoio};

fn main() -> Result<()> {
    let window = "video capture";
    let window1 = "video capture2";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
    highgui::named_window(window1, highgui::WINDOW_AUTOSIZE)?;
    let mut cam0 = videoio::VideoCapture::new_default(4)?; // 0 is the default camera
    let mut cam1 = videoio::VideoCapture::new_default(6)?; // 0 is the default camera
    //opencv::viz::Color::gray()?;
    let mut orb = <dyn features2d::ORB>::default()?;
    let mut vector2d = core::Vector::<core::Vector<core::Point2f>>::new();
    let mut vector3d = core::Vector::<core::Vector<core::Point3f>>::new();
    // every 1 second loop once
    loop {
        // left eye
        let mut frame = Mat::default();
        cam0.read(&mut frame)?;
        if frame.size()?.width > 0 {
            //highgui::imshow(window, &mut frame)?;
            let mut gray = Mat::default();
            imgproc::cvt_color(&frame, &mut gray, imgproc::COLOR_BGR2GRAY, 0)?;
            let mut kps = opencv::types::VectorOfKeyPoint::new();
            let mask = Mat::default();
            orb.detect(&gray, &mut kps, &mask)?;
            let mut display = Mat::default();
            #[cfg(not(ocvrs_opencv_branch_4))]
            let default_draw_matches_flags = features2d::DrawMatchesFlags_DEFAULT;
            features2d::draw_keypoints(
                &gray,
                &kps,
                &mut display,
                core::Scalar::all(-1f64),
                default_draw_matches_flags,
            )?;
            highgui::imshow(window, &display)?;
        }
        let mut frame1 = Mat::default();
        // right eye
        cam1.read(&mut frame1)?;
        if frame1.size()?.width > 0 {
            let mut ventor2d = core::Vector::<core::Point2f>::new();
            //let mut display = Mat::default();
            //let find = calib3d::find_chessboard_corners(
            //    &frame1,
            //    core::Size2i {
            //        width: 12,
            //        height: 8,
            //    },
            //    // 将frame1找到的信息绘制到display上
            //    &mut display,
            //    calib3d::CALIB_CB_ADAPTIVE_THRESH
            //        + calib3d::CALIB_CB_FAST_CHECK
            //        + calib3d::CALIB_CB_FILTER_QUADS
            //        + calib3d::CALIB_CB_NORMALIZE_IMAGE,
            //)?;
            // 找出所有角点记录到ventor2d上
            //
            // 注释掉原本的，因为需要一个二维数组而不是mat
            let find = calib3d::find_chessboard_corners(
                &frame1,
                core::Size2i {
                    width: 12,
                    height: 8,
                },
                // 将frame1找到的信息绘制到display上
                &mut ventor2d,
                calib3d::CALIB_CB_ADAPTIVE_THRESH
                    + calib3d::CALIB_CB_FAST_CHECK
                    + calib3d::CALIB_CB_FILTER_QUADS
                    + calib3d::CALIB_CB_NORMALIZE_IMAGE,
            )?;

            // 找到后绘制出角点
            if find {
                //println!("{:?}",display);
                //println!("ss");
                calib3d::draw_chessboard_corners(
                    &mut frame1,
                    core::Size2i {
                        width: 12,
                        height: 8,
                    },
                    &ventor2d,
                    find,
                )?;
                highgui::imshow(window1, &frame1)?;
                if highgui::wait_key(10)? > 0{
                    let mut ventor = core::Vector::<core::Point3f>::new();
                    for i in 0..12{
                        for j in 0..8{
                            let temp = core::Point3f{
                                x: i as f32,
                                y: j as f32,
                                z: 0f32,
                            };
                            ventor.push(temp);
                        }
                    }
                    // 万万想不到，你这家伙是要数组套数组的！

                    vector2d.push(ventor2d);

                    vector3d.push(ventor);

                    //println!("{}",ventor.len());
                    //println!("ss{}",ventor2d.len());

                }
            }
            highgui::imshow(window1, &frame1)?;
        }
        let key = highgui::wait_key(10)?;
        if key > 0 && key != 255 {
            break;
        }
    }
    // 释放cam0
    cam0.release()?;
    drop(cam0);

    // 清除所有窗口
    highgui::destroy_all_windows()?;
    println!("out");

    //获取内参所有信息
    let mut camera_matrix = Mat::default();
    let mut dist_coeffs = Mat::default();
    //let mut rvecs = Mat::default();
    //let mut tvecs = Mat::default();
    let mut rvecs = core::Vector::<Mat>::new();
    let mut tvecs = core::Vector::<Mat>::new();
    let ret = calib3d::calibrate_camera(
        &vector3d, 
        &vector2d,
        core::Size2i{
            width: 12,
            height: 8,
        },
        &mut camera_matrix, 
        &mut dist_coeffs,
        &mut rvecs, 
        &mut tvecs,
        calib3d::CALIB_FIX_PRINCIPAL_POINT,
        core::TermCriteria { 
            typ: 10,
            max_count: 10,
            epsilon: 10f64,
        }
    )?;
    println!("{}",ret);
    for rv in rvecs {
        println!("{:?}",rv);
    }
    //println!("rvecs 旋转向量{:?}",rvecs);
    let mut roi = core::Rect2i::default();
    let newcameramtx = calib3d::get_optimal_new_camera_matrix(
        &camera_matrix,
        &dist_coeffs,
        core::Size2i{
            width: 12,
            height: 8,
        },
        0f64,
        core::Size2i{
            width: 12,
            height: 8,
        },
        &mut roi,
        true
    )?;

    let window2 = "video capture2";
    highgui::named_window(window2, highgui::WINDOW_AUTOSIZE)?;
    let mut cam2 = videoio::VideoCapture::new_default(4)?; // 0 is the default camera
    let mut frame = Mat::default();
    cam2.read(&mut frame)?;
    if frame.size()?.width > 0 {
        let array = Mat::default();
        let size = frame.size()?;
        let mut dst1 = Mat::default();
        println!("ssss");
        imgproc::undistort(
            &frame,
            &mut dst1,
            &camera_matrix,
            &dist_coeffs,
            &newcameramtx,
        )?;
        println!("ffff");
        let mut mapx = Mat::default();
        let mut mapy = Mat::default();
        imgproc::init_undistort_rectify_map(
            &camera_matrix,
            &dist_coeffs,
            &array,
            &newcameramtx,
            size,
            core::CV_16SC2, 
            &mut mapx,
            &mut mapy
        )?;
        println!("mmmm");
        let mut dist2 = Mat::default();
        imgproc::remap(
            &frame,
            &mut dist2,
            &mapx, 
            &mapy,
            imgproc::INTER_AREA,
            core::BORDER_TRANSPARENT,
            core::Scalar::default(),
        )?;
        highgui::imshow(window2, &dist2)?;
        highgui::wait_key(1000)?;
    }
    Ok(())
}
