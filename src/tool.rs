use opencv::{
    calib3d,
    core::{self, Point2f, Point3f, Vector},
    highgui, imgcodecs, imgproc,
    prelude::*,
    videoio::VideoCapture,
    Result,
};
use std::fs;
fn create_store_before(dir: &str) {
    fs::create_dir_all(dir).unwrap();
}
pub fn create_vec_message(
    vector2d: &mut Vector<Vector<Point2f>>,
    vector3d: &mut Vector<Vector<Point3f>>,
    camera: &mut VideoCapture,
    window: &str,
    dir: &str,
    count: &mut i32,
) -> Result<()> {
    create_store_before(dir);
    let mut frame1 = Mat::default();
    camera.read(&mut frame1)?;
    if frame1.size()?.width > 0 {
        let mut ventor2d = Vector::<Point2f>::new();
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
            *count += 1;
            let filename = format!("{}/input-{}.png", dir, count.clone().to_string());
            let mut params: Vector<i32> = Vector::new();
            params.push(3);
            params.push(4);
            imgcodecs::imwrite(&filename, &frame1, &params)?;
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
            highgui::imshow(window, &frame1)?;
            if highgui::wait_key(10)? > 0 {
                let mut ventor = core::Vector::<core::Point3f>::new();
                for i in 0..12 {
                    for j in 0..8 {
                        let temp = core::Point3f {
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
        highgui::imshow(window, &frame1)?;
    }
    Ok(())
}
pub fn newcameramtx(
    vector2d: &Vector<Vector<Point2f>>,
    vector3d: &Vector<Vector<Point3f>>,
    camera_matrix: &mut Mat,
    dist_coeffs: &mut Mat,
) -> Result<Mat> {
    //获取内参所有信息
    //let mut rvecs = Mat::default();
    //let mut tvecs = Mat::default();
    // 旋转值
    let mut rvecs = Vector::<Mat>::new();
    let mut tvecs = Vector::<Mat>::new();
    let ret = calib3d::calibrate_camera(
        vector3d,
        vector2d,
        core::Size2i {
            width: 12,
            height: 8,
        },
        camera_matrix,
        dist_coeffs,
        &mut rvecs,
        &mut tvecs,
        calib3d::CALIB_FIX_PRINCIPAL_POINT,
        core::TermCriteria {
            typ: 10,
            max_count: 10,
            epsilon: 10f64,
        },
    )?;
    println!("{}", ret);
    //println!("rvecs 旋转向量{:?}",rvecs);
    let mut roi = core::Rect2i::default();
    calib3d::get_optimal_new_camera_matrix(
        camera_matrix,
        dist_coeffs,
        core::Size2i {
            width: 12,
            height: 8,
        },
        0f64,
        core::Size2i {
            width: 12,
            height: 8,
        },
        &mut roi,
        true,
    )
}
pub fn new_camera(
    camera: &mut VideoCapture,
    camera_matrix: &Mat,
    dist_coeffs: &Mat,
    newcameramtx: &Mat,
    window: &str,
    count: &mut i32,
    dir: &str,
) -> Result<()> {
    create_store_before(dir);
    let mut frame = Mat::default();
    camera.read(&mut frame)?;
    if frame.size()?.width > 0 {
        *count += 1;
        let array = Mat::default();
        let size = frame.size()?;
        let mut dst1 = Mat::default();
        calib3d::undistort(
            &frame,
            &mut dst1,
            &camera_matrix,
            &dist_coeffs,
            &newcameramtx,
        )?;
        let mut mapx = Mat::default();
        let mut mapy = Mat::default();
        calib3d::init_undistort_rectify_map(
            &camera_matrix,
            &dist_coeffs,
            &array,
            &newcameramtx,
            size,
            core::CV_16SC2,
            &mut mapx,
            &mut mapy,
        )?;
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
        highgui::imshow(window, &dist2)?;
        let filename = format!("{}/output-{}.png", dir, count.clone().to_string());
        let mut params: Vector<i32> = Vector::new();
        params.push(3);
        params.push(4);
        imgcodecs::imwrite(&filename, &dist2, &params)?;

    }
    Ok(())
}
