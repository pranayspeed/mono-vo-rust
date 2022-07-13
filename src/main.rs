// mod visodo_w;
mod visual_odometry;
use opencv::prelude::*;
use opencv::core::*;
use opencv::imgproc::*;


fn main() {
  let file_path = "/home/pranayspeed/Work/git_repos/00/image_2";
  let scan_file_path = "/home/pranayspeed/Work/git_repos";
  let mut vo = visual_odometry::VisualOdometry::new(file_path, scan_file_path);

  let frame1 = vo.get_bw_frame(0);
  let frame2 = vo.get_bw_frame(1);

  let mut points1 = vo.extract_features(&frame1);
  let points2 = vo.track_features(&frame1, &frame2, &mut points1);
  let (recover_r, recover_t) = vo.calculate_transform(&points1, &points2);

  let mut prev_image = frame2;
  //let mut curr_image: Mat;
  let mut prev_features = points2;

  vo.set_rotation(recover_r.clone());
  vo.set_translation(recover_t.clone());

  let font_face: i32 = FONT_HERSHEY_PLAIN;
  let font_scale: f64 = 1.0;
  let thickness: i32 = 1;
  let text_org = Point::new(10, 50);

  opencv::highgui::named_window("Road facing camera", opencv::highgui::WINDOW_AUTOSIZE).unwrap(); // Create a window for display.
  opencv::highgui::named_window("Trajectory", opencv::highgui::WINDOW_AUTOSIZE).unwrap(); // Create a window for display.

  let mut traj = opencv::core::Mat::zeros(600, 600, CV_8UC3)
    .unwrap()
    .to_mat()
    .unwrap();

  for num_frame in 2..vo.get_max_frame() {
    let curr_image_c = vo.get_frame(num_frame);
    let curr_image = vo.get_bw_from_color(&curr_image_c);
    //let mut curr_features = vo.extract_features(&curr_image);
    let mut curr_features = vo.track_features(&prev_image, &curr_image, &mut prev_features);

    let (recover_r, recover_t) = vo.calculate_transform(&curr_features, &mut prev_features);

    let scale = vo.get_scale(num_frame, &vo.get_translation());
    let (rotation, translation) =
      vo.scale_transform(scale, &vo.get_rotation(), &vo.get_translation(), &recover_r, &recover_t);
      vo.set_rotation(rotation);
      vo.set_translation(translation);

    // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
    if prev_features.len() < vo.get_min_num_feat() as usize {
      //cout << "Number of tracked features reduced to " << prev_features.len() << endl;
      //cout << "trigerring redection" << endl;
      prev_features = vo.extract_features(&prev_image);
      curr_features = vo.track_features(&prev_image, &curr_image, &mut prev_features);
    }

    prev_image = curr_image.clone();
    prev_features = curr_features;

    ////////////////////////////////////Show on GUI///////////////////////////////////////////////////

    let x_c = *vo.get_translation().at::<f64>(0).unwrap() as i32 + 300;
    let y_c = *vo.get_translation().at::<f64>(2).unwrap() as i32 + 100;

    circle(
      &mut traj,
      Point { x: x_c, y: y_c },
      1,
      Scalar::new(255.0, 0.0, 0.0, 0.0),
      2,
      LINE_8,
      0,
    )
    .unwrap();

    rectangle(
      &mut traj,
      Rect2i {
        x: 10,
        y: 30,
        width: 550,
        height: 50,
      },
      Scalar::new(0.0, 0.0, 0.0, 0.0),
      CV_FILLED,
      0,
      0,
    )
    .unwrap();
    let text = format!(
      "Coordinates: x = {:.2}m y = {:.2}m z = {:.2}m",
      vo.get_translation().at::<f64>(0).unwrap(),
      vo.get_translation().at::<f64>(1).unwrap(),
      vo.get_translation().at::<f64>(2).unwrap()
    );

    opencv::imgproc::put_text(
      &mut traj,
      &text,
      text_org,
      font_face,
      font_scale,
      Scalar::new(255.0, 255.0, 255.0, 255.0),
      thickness,
      8,
      false,
    )
    .unwrap();

    opencv::highgui::imshow("Road facing camera", &curr_image_c).unwrap();
    opencv::highgui::imshow("Trajectory", &traj).unwrap();

    opencv::highgui::wait_key(1).unwrap();
  }
}
