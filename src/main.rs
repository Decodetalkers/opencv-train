use opencv::{calib3d, core, features2d, highgui, imgproc, prelude::*, videoio, Result};

fn main() -> Result<()> {
    let window = "video capture";
    let window1 = "video capture2";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
    highgui::named_window(window1, highgui::WINDOW_AUTOSIZE)?;
    let mut cam0 = videoio::VideoCapture::new_default(4)?; // 0 is the default camera
    let mut cam1 = videoio::VideoCapture::new_default(6)?; // 0 is the default camera
    opencv::viz::Color::gray()?;
    let mut orb = <dyn features2d::ORB>::default()?;
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
            let mut display = Mat::default();
            let find = calib3d::find_chessboard_corners(
                &frame1,
                core::Size_ {
                    width: 11,
                    height: 8,
                },
                &mut display,
                calib3d::CALIB_CB_ADAPTIVE_THRESH
                    + calib3d::CALIB_CB_FAST_CHECK
                    + calib3d::CALIB_CB_FILTER_QUADS
                    + calib3d::CALIB_CB_NORMALIZE_IMAGE,
            )?;
            if find {
                println!("sss");
                //println!("ss");
                calib3d::draw_chessboard_corners(
                    &mut frame1,
                    core::Size_ {
                        width: 11,
                        height: 8,
                    },
                    &display,
                    find,
                )?;
            }
            highgui::imshow(window1, &frame1)?;
        }
        let key = highgui::wait_key(10)?;
        if key > 0 && key != 255 {
            break;
        }
    }
    Ok(())
}
