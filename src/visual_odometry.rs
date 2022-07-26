#![allow(dead_code)]

use opencv::types::*;
use opencv::core::*;

pub struct VisualOdometry {
  rotation: Mat,
  translation: Mat,
  min_num_feat: i32,
  focal: f64,
  pp: Point2d,
  prev_image: Mat,
  prev_features: VectorOfPoint2f,
}

impl VisualOdometry {
  //Construct person
  pub fn deafult() -> VisualOdometry {
    VisualOdometry {
      rotation: Mat::default(),
      translation: Mat::default(),
      min_num_feat: 2000,
      focal: 718.8560,
      pp: Point2d::new(607.1928, 185.2157),
      prev_image : Mat::default(),
      prev_features: VectorOfPoint2f::new(),
    }
  }

  pub fn init(&mut self, frame1: Mat, frame2: Mat)
  {
    self.prev_features = self.extract_features(&frame1);
    self.prev_image = frame1.clone();

    let curr_features = self.track_features_current(&frame2);
    let (recover_r, recover_t) = self.calculate_transform_current(&curr_features);
  
    self.prev_image = frame2.clone();
    //let mut curr_image: Mat;
    self.prev_features = curr_features;

    self.set_rotation(recover_r.clone());
    self.set_translation(recover_t.clone());
  }


  pub fn track_frame(&mut self, curr_image: &Mat, scale : f64) -> Mat
  {

      let mut curr_features = self.track_features_current(&curr_image);
  
      let (recover_r, recover_t) = self.calculate_transform_current(&curr_features);

      let (rotation, translation) =
      self.scale_transform(scale, &self.get_rotation(), &self.get_translation(), &recover_r, &recover_t);
      self.set_rotation(rotation);
      self.set_translation(translation);
  
  
      // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
      if self.prev_features.len() < self.get_min_num_feat() as usize {
        //cout << "Number of tracked features reduced to " << self.prev_features.len() << endl;
        //cout << "trigerring redection" << endl;
        //println!("Number of tracked features reduced to {:?} \ntrigerring redection ", self.prev_features.len());
        self.prev_features = self.extract_features(&self.prev_image);
        curr_features = self.track_features_current( &curr_image);
      }
  
      self.prev_image = curr_image.clone();
      self.prev_features = curr_features;

      ///////////////////////////////////////

    // let x_c = *self.get_translation().at::<f64>(0).unwrap() as i32 + 300;
    // let y_c = *self.get_translation().at::<f64>(2).unwrap() as i32 + 100;

    self.get_translation().clone()
  }

  pub fn extract_features(&self, curr_image: &Mat) -> VectorOfPoint2f {
    let mut points1 = VectorOfPoint2f::new(); //vectors to store the coordinates of the feature points
    self.feature_detection(curr_image, &mut points1); //detect features in img_1
    points1
  }



  pub fn track_features_current(
    &mut self,
    dst_image: &Mat,
  ) -> VectorOfPoint2f {
    let mut status = VectorOfu8::new();
    let mut dst_image_points = VectorOfPoint2f::new(); //vectors to store the coordinates of the feature points
    let mut prev_features = self.prev_features.clone();
    self.feature_tracking(
      &self.prev_image,
      &dst_image,
      &mut prev_features,
      &mut dst_image_points,
      &mut status,
    ); //track those features to img_2
    self.prev_features = prev_features;
    dst_image_points
  }


  pub fn calculate_transform_current(
    &self,
    curr_features: &VectorOfPoint2f,
  ) -> (Mat, Mat) {
    //recovering the pose and the essential matrix
    let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
    let essential_mat = opencv::calib3d::find_essential_mat(
      &curr_features,
      &self.prev_features,
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
      &self.prev_features,
      &mut recover_r,
      &mut recover_t,
      self.focal,
      self.pp,
      &mut mask,
    )
    .unwrap();

    (recover_r, recover_t)
  }



  pub fn track_features(
    &mut self,
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


  pub fn calculate_transform(
    &mut self,
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
      //println!("scale below 0.1, or incorrect translation");
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

  pub fn get_min_num_feat(&self) -> i32
  {
    self.min_num_feat
  }

}