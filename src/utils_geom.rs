

use std::any::TypeId;
use std::convert::TryInto;

use cv_convert::IntoCv;
use cv_convert::TryFromCv;
use cv_convert::TryIntoCv;
use nalgebra::Isometry3;
use nalgebra::Matrix4;
use nalgebra::Rotation3;
use opencv::types::*;
use opencv::core::*;

use nalgebra::Matrix;

use crate::camera_pose::Poseg2o;
use crate::frame::Frame;

// # [4x4] homogeneous T from [3x3] R and [3x1] t             
// def poseRt(R, t):
//     ret = np.eye(4)
//     ret[:3, :3] = R
//     ret[:3, 3] = t
//     return ret  
pub fn poseRt_old(R: &Mat, t: &Mat) -> Mat
{
    //ret = np.eye(4)
    let mut Rnew = R.clone();
    unsafe{
        Rnew.create_nd( &[R.rows(), R.cols()], CV_32FC1).unwrap();
    }
    let mut tnew = t.clone();
    unsafe{
        tnew.create_nd( &[t.rows(), t.cols()], CV_32FC1).unwrap();
    }
    let ret = get_transformation_matrix(&Rnew,&tnew);
    ret
}


pub fn poseRt(R: &Mat, t: &Mat) -> Poseg2o
{
    
    let mut Rnew = R.clone();
    R.convert_to(&mut Rnew, CV_32FC1, 1.0, 0.0).unwrap();

    let mut tnew = t.clone();
    t.convert_to(&mut tnew, CV_32FC1, 1.0, 0.0).unwrap();

    let mut curr_pose = Isometry3::<f32>::identity();   
    //println!("{:?}", pose);     
    let t_na = nalgebra::Matrix3x1::<f32>::try_from_cv(tnew).unwrap();
    // Updated rotation
    
    let rot_mat = nalgebra::Matrix3::<f32>::try_from_cv(Rnew).unwrap();
    //let rot_mat: Matrix3<f32> = *pose_na.slice((0,0), (3,3)).fixed_slice::<3,3>(0, 0);
    curr_pose.rotation = nalgebra::UnitQuaternion::<f32>::from_rotation_matrix(&Rotation3::<f32>::from_matrix_unchecked(rot_mat));

    curr_pose.translation = nalgebra::Translation3::<f32>::from(t_na);

    //println!("poseRt-> {:?}  ", curr_pose.to_matrix());
    // let x = pose_na.get((0,3)).unwrap();
    // let y = pose_na.get((1,3)).unwrap();
    // let z = pose_na.get((2,3)).unwrap();

    // // Updated translation
    // curr_pose.translation = nalgebra::Translation3::<f32>::new(*x,*y,*z);

    /* 
    let r1_na = nalgebra::Matrix3::<f64>::try_from_cv(R).unwrap();
    let t1_na = nalgebra::Matrix3x1::<f64>::try_from_cv(t).unwrap();
    println!("{:?}\n {:?}", r1_na, t1_na);
    println!("{:?}\n {:?}", rot_mat, t_na);
    println!("matrix {:?}", curr_pose.to_matrix());
    panic!("Done");
    */

    let mut ret_pose = Poseg2o::default();

    ret_pose._pose = curr_pose;

    ret_pose

}

pub fn get_transformation_matrix(R: &Mat, t: &Mat) -> Mat
{
    let mut proj_mat = Mat::eye(4, 4, CV_32FC1).unwrap().to_mat().unwrap();

    for i in 0..R.rows() {
        for j in 0..R.cols() {
            let val = *R.at_2d::<f32>(i, j).unwrap(); // Grayscale 1 channel uint8
            unsafe {
            *proj_mat.at_2d_unchecked_mut::<f32>(i, j).unwrap() = val;
            }
        }
        let val = *t.at_2d::<f32>(i, 0).unwrap();
        unsafe {
            *proj_mat.at_2d_unchecked_mut::<f32>(i, 3).unwrap() = val;
            }

    }

    proj_mat

}


pub fn get_rotation_from_transformation(T: &Mat) -> Mat
{
    // extract sub matrix
    let mut sub_mat_range = VectorOfRange::new();
    sub_mat_range.push(Range::new(0,3).unwrap());
    sub_mat_range.push(Range::new(0,3).unwrap());
    let R  = Mat::ranges(&T, &sub_mat_range).unwrap();
    R
}

pub fn get_translation_from_transformation(T: &Mat) -> Mat
{
    // extract sub matrix
    let mut trans_mat_range = VectorOfRange::new();
    trans_mat_range.push(Range::new(0,3).unwrap());
    trans_mat_range.push(Range::new(3,4).unwrap());

    let t = Mat::ranges(&T, &trans_mat_range).unwrap();

    t
}


//# [4x4] homogeneous inverse T^-1 from [4x4] T     
// def inv_T(T):
//     ret = np.eye(4)
//     R_T = T[:3,:3].T
//     t   = T[:3,3]
//     ret[:3, :3] = R_T
//     ret[:3, 3] = -R_T @ t
//     return ret 

pub fn inv_T_old(T: &Mat) -> Mat
{
    // extract sub matrix
    let mut sub_mat_range = VectorOfRange::new();
    sub_mat_range.push(Range::new(0,3).unwrap());
    sub_mat_range.push(Range::new(0,3).unwrap());
    let R_T  = get_rotation_from_transformation(&T).t().unwrap().to_mat().unwrap();

    let t = get_translation_from_transformation(&T);

    let R_T_t = mul_matexpr_mat(&mul_f64_mat(-1.0,&R_T).unwrap(), &t).unwrap().to_mat().unwrap();

    let ret = get_transformation_matrix(&R_T, &R_T_t);

    ret
}




pub fn inv_T(T: &Poseg2o) -> Poseg2o
{
    // extract sub matrix


//     ret = np.eye(4)
    let mut ret_pose = Poseg2o::default();
//     R_T = T[:3,:3].T
    let R_T = T._pose.rotation.to_rotation_matrix().transpose();
//     t   = T[:3,3]
    let t = T._pose.translation;
//     ret[:3, :3] = R_T
    ret_pose._pose.rotation = nalgebra::UnitQuaternion::<f32>::from_rotation_matrix(&R_T);
//     ret[:3, 3] = -R_T @ t
    //NEED TO CHECK
    ret_pose._pose.translation = (R_T * t).translation.inverse();
//     return ret  
    ret_pose


    // let mut sub_mat_range = VectorOfRange::new();
    // sub_mat_range.push(Range::new(0,3).unwrap());
    // sub_mat_range.push(Range::new(0,3).unwrap());
    // let R_T  = get_rotation_from_transformation(&T).t().unwrap().to_mat().unwrap();

    // let t = get_translation_from_transformation(&T);

    // let R_T_t = mul_matexpr_mat(&mul_f64_mat(-1.0,&R_T).unwrap(), &t).unwrap().to_mat().unwrap();

    // let ret = get_transformation_matrix(&R_T, &R_T_t);

    // ret
}





// def triangulate_normalized_points(pose_1w, pose_2w, kpn_1, kpn_2):
//     # P1w = np.dot(K1,  M1w) # K1*[R1w, t1w]
//     # P2w = np.dot(K2,  M2w) # K2*[R2w, t2w]
//     # since we are working with normalized coordinates x_hat = Kinv*x, one has         
//     P1w = pose_1w[:3,:] # [R1w, t1w]
//     P2w = pose_2w[:3,:] # [R2w, t2w]

//     point_4d_hom = cv2.triangulatePoints(P1w, P2w, kpn_1.T, kpn_2.T)
//     good_pts_mask = np.where(point_4d_hom[3]!= 0)[0]
//     point_4d = point_4d_hom / point_4d_hom[3] 
    
//     if __debug__:
//         if False: 
//             point_reproj = P1w @ point_4d;
//             point_reproj = point_reproj / point_reproj[2] - add_ones(kpn_1).T
//             err = np.sum(point_reproj**2)
//             print('reproj err: ', err)     

//     #return point_4d.T
//     points_3d = point_4d[:3, :].T
//     return points_3d, good_pts_mask  

pub fn triangulate_normalized_points(pose_1w: &Mat, pose_2w: &Mat, kpn_1: &VectorOfPoint2f, kpn_2: &VectorOfPoint2f) -> (VectorOfPoint3f, VectorOfi32)
{
//     # P2w = np.dot(K2,  M2w) # K2*[R2w, t2w]
//     # since we are working with normalized coordinates x_hat = Kinv*x, one has         
//     P1w = pose_1w[:3,:] # [R1w, t1w]
//     P2w = pose_2w[:3,:] # [R2w, t2w]
    
    let T1w = nalgebra::Matrix4::<f32>::try_from_cv(pose_1w).unwrap();
    let T2w = nalgebra::Matrix4::<f32>::try_from_cv(pose_2w).unwrap();

    let P1wna = T1w.rows(0,3);
    let P2wna = T2w.rows(0,3);

    let P1w: Mat = P1wna.try_into_cv().unwrap();
    let P2w: Mat = P2wna.try_into_cv().unwrap();


//     point_4d_hom = cv2.triangulatePoints(P1w, P2w, kpn_1.T, kpn_2.T)
//     good_pts_mask = np.where(point_4d_hom[3]!= 0)[0]
//     point_4d = point_4d_hom / point_4d_hom[3] 
    let mut point_4d_hom = Mat::default();
    opencv::calib3d::triangulate_points(&P1w, &P2w, kpn_1, kpn_2, &mut point_4d_hom).unwrap();

    let mut  good_pts_mask =  VectorOfi32::new();

            
    let triangulated_points =    point_4d_hom.t().unwrap().to_mat().unwrap();
        
    let mut points_3d = VectorOfPoint3f::new();
    opencv::calib3d::convert_points_from_homogeneous(&triangulated_points, &mut points_3d).unwrap();
    
    for i in 0..triangulated_points.rows()
    {
        if *triangulated_points.at_2d::<f32>(i, 2).unwrap() !=0.0
        {
            good_pts_mask.push(1);
        }
        else {
            good_pts_mask.push(0);
        }
    }
    (points_3d, good_pts_mask)
}




// # fit essential matrix E with RANSAC such that:  p2.T * E * p1 = 0  where  E = [t21]x * R21
// # input: kpn_ref and kpn_cur are two arrays of [Nx2] normalized coordinates of matched keypoints 
// # out: a) Trc: homogeneous transformation matrix containing Rrc, trc  ('cur' frame with respect to 'ref' frame)    pr = Trc * pc 
// #      b) mask_match: array of N elements, every element of which is set to 0 for outliers and to 1 for the other points (computed only in the RANSAC and LMedS methods)
// # N.B.1: trc is estimated up to scale (i.e. the algorithm always returns ||trc||=1, we need a scale in order to recover a translation which is coherent with previous estimated poses)
// # N.B.2: this function has problems in the following cases: [see Hartley/Zisserman Book]
// # - 'geometrical degenerate correspondences', e.g. all the observed features lie on a plane (the correct model for the correspondences is an homography) or lie a ruled quadric 
// # - degenerate motions such a pure rotation (a sufficient parallax is required) or an infinitesimal viewpoint change (where the translation is almost zero)
// # N.B.3: the five-point algorithm (used for estimating the Essential Matrix) seems to work well in the degenerate planar cases [Five-Point Motion Estimation Made Easy, Hartley]
// # N.B.4: as reported above, in case of pure rotation, this algorithm will compute a useless fundamental matrix which cannot be decomposed to return a correct rotation 
// # N.B.5: the OpenCV findEssentialMat function uses the five-point algorithm solver by D. Nister => hence it should work well in the degenerate planar cases
// def estimate_pose_ess_mat(kpn_ref, kpn_cur, method=cv2.RANSAC, prob=0.999, threshold=0.0003):


const kRansacThresholdNormalized : f64 = 1.0;//0.0003;//  # metric threshold used for normalized image coordinates 
const kRansacProb : f64 = 0.999;
pub fn 	estimate_pose_ess_mat_old(kpn_ref: &Frame, kpn_cur: &Frame) -> (Poseg2o, VectorOfi32)
{
    //, method=cv2.RANSAC, prob=0.999, threshold=0.0003

    //     # here, the essential matrix algorithm uses the five-point algorithm solver by D. Nister (see the notes and paper above )     
//     E, mask_match = cv2.findEssentialMat(kpn_cur, kpn_ref, focal=1, pp=(0., 0.), method=method, prob=prob, threshold=threshold)                         
//     _, R, t, mask = cv2.recoverPose(E, kpn_cur, kpn_ref, focal=1, pp=(0., 0.))   
//     return poseRt(R,t.T), mask_match  # Trc, mask_mat  

        //recovering the pose and the essential matrix
        let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
        let mut mask_match = VectorOfi32::new();
        //     E, self.mask_match = cv2.findEssentialMat(kpn_cur, kpn_ref, focal=1, pp=(0., 0.), method=cv2.RANSAC, prob=kRansacProb, threshold=kRansacThresholdNormalized)   
        //[TO_CHECK]: if the focal is to be set as 1 and pp as 0,0


        //println!("{:?}  {:?}  \n{:?} \n {:?} ", kpn_ref.id, kpn_cur.id, kpn_ref.kps.len(), kpn_cur.kps.len());

        let essential_mat = opencv::calib3d::find_essential_mat(
          &kpn_cur.kps,
          &kpn_ref.kps,
          1.0,
          Point2d::new(607.1928, 185.2157),
          opencv::calib3d::RANSAC,
          kRansacProb,
          kRansacThresholdNormalized,
          &mut mask_match,
        )
        .unwrap();
            //     _, R, t, mask = cv2.recoverPose(E, kpn_cur, kpn_ref, focal=1, pp=(0., 0.))   
        opencv::calib3d::recover_pose(
          &essential_mat,
          &kpn_cur.kps,
          &kpn_ref.kps,
          &mut recover_r,
          &mut recover_t,
          1.0,
          Point2d::new(607.1928, 185.2157),
          &mut mask,
        )
        .unwrap();
        
        //println!("{:?}, {:?}", recover_r, recover_t);
        
        (poseRt(&recover_r, &recover_t), mask_match) //.t().unwrap().to_mat().unwrap()), mask_match)
    
}
      


pub fn 	estimate_pose_ess_mat(       
     curr_features: &VectorOfPoint2f,
    prev_features: &VectorOfPoint2f) -> (Poseg2o, VectorOfu8)
{
    //, method=cv2.RANSAC, prob=0.999, threshold=0.0003

    //     # here, the essential matrix algorithm uses the five-point algorithm solver by D. Nister (see the notes and paper above )     
//     E, mask_match = cv2.findEssentialMat(kpn_cur, kpn_ref, focal=1, pp=(0., 0.), method=method, prob=prob, threshold=threshold)                         
//     _, R, t, mask = cv2.recoverPose(E, kpn_cur, kpn_ref, focal=1, pp=(0., 0.))   
//     return poseRt(R,t.T), mask_match  # Trc, mask_mat  

        //recovering the pose and the essential matrix
        let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
        let mut mask_match = VectorOfu8::new();
        //     E, self.mask_match = cv2.findEssentialMat(kpn_cur, kpn_ref, focal=1, pp=(0., 0.), method=cv2.RANSAC, prob=kRansacProb, threshold=kRansacThresholdNormalized)   
        //[TO_CHECK]: if the focal is to be set as 1 and pp as 0,0


       //println!(" \n{:?} \n {:?} ", curr_features.len(), prev_features.len());

       let focal = 1.0;//718.8560;
       let pp = Point2d::new(0.0,0.0); //Point2d::new(607.1928, 185.2157);
       let kRansacThresholdNormalized_ = 0.0003;//1.0;
       let kRansacProb_ = 0.999;


        let essential_mat = opencv::calib3d::find_essential_mat(
          curr_features,
          prev_features,
          focal,
          pp,
          opencv::calib3d::RANSAC,
          kRansacProb_,
          kRansacThresholdNormalized_,
          &mut mask_match,
        )
        .unwrap();
            //     _, R, t, mask = cv2.recoverPose(E, kpn_cur, kpn_ref, focal=1, pp=(0., 0.))   
        opencv::calib3d::recover_pose(
          &essential_mat,
          curr_features,
          prev_features,
          &mut recover_r,
          &mut recover_t,
          focal,
          pp,
          &mut mask,
        )
        .unwrap();
        
        //println!("{:?}, {:?}", recover_r, recover_t);
        
        (poseRt(&recover_r, &recover_t), mask_match) //.t().unwrap().to_mat().unwrap()), mask_match)
    
}
     


pub fn scale_transform(
    scale: f64,
    rotation: &Mat,
    translation: &Mat,
    recover_mat: &Mat,
    t: &Mat,
  ) -> (Mat, Mat) {
    //R = recover_mat, R_f= rotation, t_f =translation , t
    if scale > 0.1
      //&& (t.at::<f32>(2).unwrap() > t.at::<f32>(0).unwrap())
      //&& (t.at::<f32>(2).unwrap() > t.at::<f32>(1).unwrap())
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
