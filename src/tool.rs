use opencv::{
    calib3d,
    core::{self, Point2f, Vector},
    highgui, imgcodecs,
    prelude::*,
    videoio,
    Result,
};
//fn create_store_before(dir: &str) {
//    fs::create_dir_all(dir).unwrap();
//}
pub fn create_vec_message(
    vector2d: &mut Vector<Vector<Point2f>>,
    window: &str,
    dir: &str,
) -> Result<()> {
    let mut pics: core::Vector<String> = core::Vector::<String>::new();
    core::glob(dir, &mut pics, false)?;
    for apic in pics {
        let mut frame = imgcodecs::imread(&apic, 0)?;
        if frame.size()?.width > 0 {
            let mut ventor2d = Vector::<Point2f>::new();
            let find = calib3d::find_chessboard_corners(
                &frame,
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
            if find{
            // 找到后绘制出角点
                let mut params: Vector<i32> = Vector::new();
                params.push(3);
                params.push(4);
                //println!("{:?}",display);
                //println!("ss");
                calib3d::draw_chessboard_corners(
                    &mut frame,
                    core::Size2i {
                        width: 12,
                        height: 8,
                    },
                    &ventor2d,
                    find,
                )?;
                highgui::imshow(window, &frame)?;
                //if highgui::wait_key(10)? > 0 {
                // 万万想不到，你这家伙是要数组套数组的！
                //println!("ssss");
                vector2d.push(ventor2d);
            } 
                //println!("{}",ventor.len());
                //println!("ss{}",ventor2d.len());
                //}
            highgui::imshow(window, &frame)?;
        }
    }
    Ok(())
}
//pub fn new_camera(
//    camera: &mut VideoCapture,
//    camera_matrix: &Mat,
//    dist_coeffs: &Mat,
//    newcameramtx: &Mat,
//    window: &str,
//    count: &mut i32,
//    dir: &str,
//) -> Result<()> {
//    create_store_before(dir);
//    let mut frame = Mat::default();
//    camera.read(&mut frame)?;
//    if frame.size()?.width > 0 {
//        *count += 1;
//        let array = Mat::default();
//        let size = frame.size()?;
//        let mut dst1 = Mat::default();
//        calib3d::undistort(
//            &frame,
//            &mut dst1,
//            &camera_matrix,
//            &dist_coeffs,
//            &newcameramtx,
//        )?;
//        let mut mapx = Mat::default();
//        let mut mapy = Mat::default();
//        calib3d::init_undistort_rectify_map(
//            &camera_matrix,
//            &dist_coeffs,
//            &array,
//            &newcameramtx,
//            size,
//            core::CV_16SC2,
//            &mut mapx,
//            &mut mapy,
//        )?;
//        let mut dist2 = Mat::default();
//        imgproc::remap(
//            &frame,
//            &mut dist2,
//            &mapx,
//            &mapy,
//            imgproc::INTER_AREA,
//            core::BORDER_TRANSPARENT,
//            core::Scalar::default(),
//        )?;
//        highgui::imshow(window, &dist2)?;
//        let filename = format!("{}/output-{}.png", dir, count.clone());
//        let mut params: Vector<i32> = Vector::new();
//        params.push(3);
//        params.push(4);
//        imgcodecs::imwrite(&filename, &dist2, &params)?;
//    }
//    Ok(())
//}
pub fn deep(
    camera_matrix: &Mat,
    dist_coeffs: &Mat,
    camera_matrix2: &Mat,
    dist_coeffs2: &Mat,
    om: &Mat,
    t: &Mat,
) -> Result<()> {
    //let om = Mat::from_slice_2d(&[[0.010], [-0.020], [0.0]])?;
    let mut r = Mat::default();
    let mut temp = Mat::default();
    calib3d::rodrigues(&om, &mut r, &mut temp)?;
    // error
    //let t = Mat::from_slice_2d(&[[-50.59612], [-2.60704], [18.87635]])?;
    let size = core::Size2i {
        width: 400,
        height: 200,
    };
    let mut r1 = Mat::default();
    let mut r2 = Mat::default();
    let mut p1 = Mat::default();
    let mut p2 = Mat::default();
    let mut q = Mat::default();
    let mut valid_pix_roi1 = core::Rect::default();
    let mut valid_pix_roi2 = core::Rect::default();
    // Something error
    opencv::calib3d::stereo_rectify(
        &camera_matrix,
        &dist_coeffs,
        &camera_matrix2,
        &dist_coeffs2,
        size,
        &r,
        &t,
        &mut r1,
        &mut r2,
        &mut p1,
        &mut p2,
        &mut q,
        opencv::calib3d::CALIB_ZERO_DISPARITY,
        -1.0,
        size,
        &mut valid_pix_roi1,
        &mut valid_pix_roi2,
    )?;
    let mut left_map1 = Mat::default();
    let mut left_map2 = Mat::default();
    opencv::calib3d::init_undistort_rectify_map(
        &camera_matrix,
        &dist_coeffs,
        &r1,
        &p1,
        size,
        core::CV_16SC2,
        &mut left_map1,
        &mut left_map2,
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
        &mut right_map2,
    )?;
    //println!("{:?}",right_map2.to_vec_2d()? as Vec<Vec<u16>>);
    // deepth
    let mut cam2 = videoio::VideoCapture::new(4, videoio::CAP_V4L)?;
    cam2.set(videoio::CAP_PROP_FRAME_WIDTH, 400.0)?;
    cam2.set(videoio::CAP_PROP_FRAME_HEIGHT, 200.0)?;
    let mut cam3 = videoio::VideoCapture::new(6, videoio::CAP_V4L)?; // 0 is the default camera
    cam3.set(videoio::CAP_PROP_FRAME_WIDTH, 400.0)?;
    cam3.set(videoio::CAP_PROP_FRAME_HEIGHT, 200.0)?;
    let window = "center";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
    let mut num: i32 = 0;
    let mut blocksize: i32 = 5;
    highgui::create_trackbar("num", window, Some(&mut num), 10, Some(Box::new(|_| {})))?;
    highgui::create_trackbar(
        "blockSize",
        window,
        Some(&mut blocksize),
        20,
        Some(Box::new(|_| {})),
    )?;
    loop {
        let mut frame1 = Mat::default();
        let mut frame2 = Mat::default();
        cam2.read(&mut frame1)?;
        cam3.read(&mut frame2)?;
        if frame1.size()?.width > 0 && frame2.size()?.width > 0 {
            let mut img1_rectified = Mat::default();
            let mut img2_rectified = Mat::default();
            //Here, remap has some trouble, after remap , It is really black . I don't known why.
            //It is hardly to belive.
            opencv::imgproc::remap(
                &frame1,
                &mut img1_rectified,
                &left_map1,
                &left_map2,
                opencv::imgproc::INTER_LINEAR,
                core::BORDER_CONSTANT,
                core::Scalar::default(),
            )?;
            opencv::imgproc::remap(
                &frame2,
                &mut img2_rectified,
                &right_map1,
                &right_map2,
                opencv::imgproc::INTER_LINEAR,
                core::BORDER_CONSTANT,
                core::Scalar::default(),
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
            let num_local = highgui::get_trackbar_pos("num", window)?;
            let mut blocksize_local = highgui::get_trackbar_pos("blockSize", window)?;
            if blocksize_local % 2 == 0 {
                blocksize_local += 1;
            }
            if blocksize_local < 5 {
                blocksize_local = 5;
            }
            let mut stereo = <dyn calib3d::StereoBM>::create(16 * num_local, blocksize_local)?;
            let mut disparity = Mat::default();
            //println!("test");
            stereo.compute(&gray_img1, &gray_img2, &mut disparity)?;
            let mut disp = Mat::default();
            let mask = Mat::default();
            //println!("test");
            core::normalize(
                &disparity,
                &mut disp,
                0.0,
                255.0,
                core::NORM_MINMAX,
                core::CV_8U,
                &mask,
            )?;
            let mut disp2 = Mat::default();
            core::divide(16f64, &disp, &mut disp2, -1)?;
            //println!("b {:?}",disp2.to_vec_2d()? as Vec<Vec<u8>>);
            let mut output = Mat::default();
            calib3d::reproject_image_to_3d(&disp2, &mut output, &q, true, 3)?;
            let show: Vec<Vec<core::Vec3s>> = output.to_vec_2d()?;
            highgui::set_mouse_callback(
                window,
                Some(Box::new(move |x, y, z, _| {
                    if x == 1 {
                        println!("{},{}", y, z);
                        println!("{:?}", show[z as usize][y as usize]);
                    }
                })),
            )?;
            highgui::imshow("first", &img1_rectified)?;
            highgui::imshow("second", &img2_rectified)?;
            highgui::imshow(window, &disp)?;
        }
        let key = highgui::wait_key(10)?;
        if key > 0 && key != 255 {
            break;
        }
    }

    Ok(())
}
