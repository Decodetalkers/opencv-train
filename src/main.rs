use opencv::{core, highgui, prelude::*, Result};

//use opencv::calib3d::prelude::StereoBM;
//use opencv::calib3d;

mod tool;
fn main() -> Result<()> {
    let mut image_points1 = core::Vector::<core::Vector<core::Point2f>>::new();
    let mut image_points2 = core::Vector::<core::Vector<core::Point2f>>::new();
    // every 1 second loop once
    let mut object_points_before = core::Vector::<core::Point3f>::new();
    for i in 0..12 {
        for j in 0..8 {
            let temp = core::Point3f {
                x: i as f32,
                y: j as f32,
                z: 0f32,
            };
            object_points_before.push(temp);
        }
    }
    let mut object_points = core::Vector::<core::Vector<core::Point3f>>::new();
    for _ in 0..48 {
      object_points.push(object_points_before.clone());
    }
    tool::create_vec_message(&mut image_points1, "a")?;
    tool::create_vec_message(&mut image_points2, "b")?;
    // 释放cam0


    // 清除所有窗口
    highgui::destroy_all_windows()?;
    println!("out");
    println!("{}", image_points1.len());
    println!("{}", image_points2.len());

    // camera_matrix left , when is used to picture the depth, I need use the first of it
    let mut camera_matrix = Mat::default();
    // like the left one , I need the right one.
    let mut camera_matrix2 = Mat::default();

    // it is distortion. also is needed.
    let mut dist_coeffs = Mat::default();
    let mut dist_coeffs2 = Mat::default();
    // rotation
    let mut r = Mat::default();
    let mut t = Mat::default();
    //let mut e = core::Vector::<Mat>::new();
    let mut e = Mat::default();
    //let mut f = core::Vector::<Mat>::new();
    let mut f = Mat::default();
    //let newcameramtx = Mat::default();
    //let newcameramtx2 = Mat::default();
    println!("here");

    opencv::calib3d::stereo_calibrate(
        &object_points,
        &image_points1,
        &image_points2,
        &mut camera_matrix,
        &mut dist_coeffs,
        &mut camera_matrix2,
        &mut dist_coeffs2,
        core::Size2i {
            width: 400,
            height: 200,
        },
        &mut r,
        &mut t,
        &mut e,
        &mut f,
        0,
        core::TermCriteria {
            typ: 3,
            max_count: 3,
            epsilon: 10f64,
        },
    )?;

    println!("here");

    // here export the solution
    println!(
        "dist_coeffs {:?}",
        dist_coeffs.to_vec_2d()? as Vec<Vec<f64>>
    );
    println!(
        "dist_coeffs2 {:?}",
        dist_coeffs2.to_vec_2d()? as Vec<Vec<f64>>
    );
    println!("rvecs_right {:?}", r.to_vec_2d()? as Vec<Vec<f64>>);
    println!("tvecs_right {:?}", t.to_vec_2d()? as Vec<Vec<f64>>);
    println!(
        "camera_matrix {:?}",
        camera_matrix.to_vec_2d()? as Vec<Vec<f64>>
    );
    println!(
        "camera_matrix2 {:?}",
        camera_matrix2.to_vec_2d()? as Vec<Vec<f64>>
    );

    let om = Mat::from_slice_2d(&(r.to_vec_2d()? as Vec<Vec<f64>>))?;
    let t = Mat::from_slice_2d(&(t.to_vec_2d()? as Vec<Vec<f64>>))?;
    tool::deep(
        &camera_matrix,
        &dist_coeffs,
        &camera_matrix2,
        &dist_coeffs2,
        &om,
        &t,
    )?;
    Ok(())
}
