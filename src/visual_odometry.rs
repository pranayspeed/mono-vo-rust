

use std::fs::File;
use std::io::{self, BufRead};
use std::path::Path;


use opencv::imgcodecs::*;
use opencv::types::*;

use opencv::core::*;

use opencv::imgproc::*;



fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<File>>>
where
  P: AsRef<Path>,
{
  let file = File::open(filename)?;
  Ok(io::BufReader::new(file).lines())
}



pub struct VisualOdometry {
  rotation: Mat,
  translation: Mat,
  file_path: String,
  scale_file_path: String,
  max_frame: i32,
  min_num_feat: i32,
  focal: f64,
  pp: Point2d,
}

impl VisualOdometry {
  //Construct person
  pub fn new(filepath: &str, scale_filepath: &str) -> VisualOdometry {
    VisualOdometry {
      rotation: Mat::default(),
      translation: Mat::default(),
      file_path: String::from(filepath),
      scale_file_path: String::from(scale_filepath),
      max_frame: 2000,
      min_num_feat: 2000,
      focal: 718.8560,
      pp: Point2d::new(607.1928, 185.2157),
    }
  }

  pub fn get_frame(&self, frame_id: i32) -> Mat {
    let filename = format!("{}{}", self.file_path, format!("/{:01$}.png", frame_id, 6));
    imread(&filename, IMREAD_COLOR).unwrap()
  }

  pub fn get_bw_frame(&self, frame_id: i32) -> Mat {
    let filename = format!("{}{}", self.file_path, format!("/{:01$}.png", frame_id, 6));
    let curr_image_c = imread(&filename, IMREAD_COLOR).unwrap();
    let mut img_1 = Mat::default();
    cvt_color(&curr_image_c, &mut img_1, COLOR_BGR2GRAY, 0).unwrap();
    img_1
  }

  pub fn get_bw_from_color(&self, img_color: &Mat) -> Mat {
    let mut img_1 = Mat::default();
    cvt_color(img_color, &mut img_1, COLOR_BGR2GRAY, 0).unwrap();
    img_1
  }

  pub fn extract_features(&self, curr_image: &Mat) -> VectorOfPoint2f {
    let mut points1 = VectorOfPoint2f::new(); //vectors to store the coordinates of the feature points
    self.feature_detection(curr_image, &mut points1); //detect features in img_1
    points1
  }

  pub fn track_features(
    &self,
    src_image: &Mat,
    dst_image: &Mat,
    src_image_points: &mut VectorOfPoint2f,
  ) -> VectorOfPoint2f {
    let mut status = VectorOfu8::new();
    let mut dst_image_points = VectorOfPoint2f::new(); //vectors to store the coordinates of the feature points
    self.feature_tracking(
      &src_image,
      &dst_image,
      src_image_points,
      &mut dst_image_points,
      &mut status,
    ); //track those features to img_2
    dst_image_points
  }

  pub fn get_scale(&self, frame_id: i32, transform: &Mat, curr_pos: &mut Point2d) -> f64 {
    self.get_absolute_scale(
      frame_id,
      "00",
      *transform.at::<f64>(2).unwrap(),
      &self.scale_file_path,
      curr_pos,
    )
  }

  pub fn get_absolute_scale(
    &self,
    frame_id: i32,
    sequence_id: &str,
    z_cal: f64,
    file_path: &str,
    curr_pos: &mut Point2d
  ) -> f64 {
    let mut i = 0;

    let filepath = format!("{}/{}.txt", &file_path, sequence_id);
    let filepath_path = Path::new(&filepath);

    let (mut x, mut y, mut z): (f64, f64, f64) = (0.0, 0.0, 0.0);

    let (mut x_prev, mut y_prev, mut z_prev): (f64, f64, f64) = (0.0, 0.0, 0.0);
    // Open the path in read-only mode, returns `io::Result<File>`
    if let Ok(lines) = read_lines(filepath_path) {
      // Consumes the iterator, returns an (Optional) String
      for line in lines {
        if let Ok(val) = line {
          z_prev = z;
          x_prev = x;
          y_prev = y;

          println!("x : {}, y: {}, z:{} ", x, y, z);
          let values: Vec<&str> = val.split(" ").collect();
          for (j, s) in values.iter().enumerate() {
            z = s.parse::<f64>().unwrap();
            if j == 7 {
              y = z;
            }
            if j == 3 {
              x = z;
            }
          }
        }
        i = i + 1;
        if i > frame_id {
          break;
        }
      }
    }

    curr_pos.x = x_prev;
    curr_pos.y = z_prev;
    f64::sqrt(
      (x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev) + (z - z_prev) * (z - z_prev),
    )
  }

  pub fn calculate_transform(
    &self,
    curr_features: &VectorOfPoint2f,
    prev_features: &VectorOfPoint2f,
  ) -> (Mat, Mat) {
    //recovering the pose and the essential matrix
    let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
    let essential_mat = opencv::calib3d::find_essential_mat(
      &curr_features,
      &prev_features,
      self.focal,
      self.pp,
      opencv::calib3d::RANSAC,
      0.999,
      1.0,
      &mut mask,
    )
    .unwrap();
    opencv::calib3d::recover_pose(
      &essential_mat,
      &curr_features,
      &prev_features,
      &mut recover_r,
      &mut recover_t,
      self.focal,
      self.pp,
      &mut mask,
    )
    .unwrap();

    (recover_r, recover_t)
  }

  pub fn scale_transform(
    &self,
    scale: f64,
    rotation: &Mat,
    translation: &Mat,
    recover_mat: &Mat,
    t: &Mat,
  ) -> (Mat, Mat) {
    //R = recover_mat, R_f= rotation, t_f =translation , t
    if scale > 0.1
      && (t.at::<f64>(2).unwrap() > t.at::<f64>(0).unwrap())
      && (t.at::<f64>(2).unwrap() > t.at::<f64>(1).unwrap())
    {
      // t_f = t_f + scale*(R_f*t);
      let mut rf_cross_t = opencv::core::mul_mat_mat(&rotation, &t).unwrap();
      rf_cross_t = opencv::core::mul_matexpr_f64(&rf_cross_t, scale).unwrap();
      let t_f = opencv::core::add_mat_matexpr(&translation, &rf_cross_t)
        .unwrap()
        .to_mat()
        .unwrap();

      // R_f = R*R_f;
      let r_f = opencv::core::mul_mat_mat(&recover_mat, &rotation)
        .unwrap()
        .to_mat()
        .unwrap();

      (r_f, t_f)
    } else {
      println!("scale below 0.1, or incorrect translation");
      (rotation.clone(), translation.clone())
    }
  }

  pub fn feature_tracking(
    &self,
    img_1: &opencv::core::Mat,
    img_2: &opencv::core::Mat,
    points1: &mut VectorOfPoint2f,
    points2: &mut VectorOfPoint2f,
    status: &mut VectorOfu8,
  ) {
    //this function automatically gets rid of points for which tracking fails
    let mut err = opencv::types::VectorOff32::default();
    let win_size = Size::new(21, 21);
    let termcrit = opencv::core::TermCriteria {
      typ: 3,
      max_count: 30,
      epsilon: 0.01,
    };
    let max_level = 3;
    opencv::video::calc_optical_flow_pyr_lk(
      img_1, img_2, points1, points2, status, &mut err, win_size, max_level, termcrit, 0, 0.001,
    )
    .unwrap();
    //getting rid of points for which the KLT tracking failed or those who have gone outside the framed
    let mut indez_correction = 0;
    for i in 0..status.len() {
      let pt = points2.get(i - indez_correction).unwrap();
      if (status.get(i).unwrap() == 0) || pt.x < 0.0 || pt.y < 0.0 {
        if pt.x < 0.0 || pt.y < 0.0 {
          status.set(i, 0).unwrap();
        }
        points1.remove(i - indez_correction).unwrap();
        points2.remove(i - indez_correction).unwrap();
        indez_correction = indez_correction + 1;
      }
    }
  }

  pub fn feature_detection(&self, img_1: &Mat, points1: &mut Vector<Point2f>) {
    //uses FAST as of now, modify parameters as necessary
    let mut keypoints_1 = opencv::types::VectorOfKeyPoint::new();
    let fast_threshold: i32 = 20;
    let non_max_suppression: bool = true;
    opencv::features2d::FAST(img_1, &mut keypoints_1, fast_threshold, non_max_suppression).unwrap();

    let pt_indx = opencv::types::VectorOfi32::new();
    KeyPoint::convert(&keypoints_1, points1, &pt_indx).unwrap();
  }

  pub fn set_rotation(&mut self, rotation: Mat)
  {
    self.rotation = rotation;
  }

  pub fn set_translation(&mut self, translation: Mat)
  {
    self.translation = translation;
  }

  pub fn get_rotation(&self) -> &Mat
  {
    &self.rotation
  }

  pub fn get_translation(&self) -> &Mat
  {
    &self.translation
  }

  pub fn get_max_frame(&self) -> i32
  {
    self.max_frame
  }

  pub fn get_min_num_feat(&self) -> i32
  {
    self.min_num_feat
  }

}