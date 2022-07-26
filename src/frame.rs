





use std::convert::TryInto;

use opencv::types::*;
use opencv::core::*;

use crate::camera::Camera;
use crate::camera_pose::CameraPose;
use crate::camera_pose::Poseg2o;
use crate::utils_geom::get_transformation_matrix;
use crate::utils_geom::poseRt;

use opencv::{
  prelude::*,
  core,
  features2d::{BFMatcher, FlannBasedMatcher},
  types::{VectorOfKeyPoint, VectorOfDMatch, VectorOfPoint2f},
};


        // self.is_keyframe = False  

        // # image keypoints information arrays (unpacked from array of cv::KeyPoint())
        // self.kps     = None      # keypoint coordinates                  [Nx2]
        // self.kpsu    = None      # [u]ndistorted keypoint coordinates    [Nx2]
        // self.kpsn    = None      # [n]ormalized keypoint coordinates     [Nx2] (Kinv * [kp,1])    
        // self.octaves = None      # keypoint octaves                      [Nx1]
        // self.sizes   = None      # keypoint sizes                        [Nx1] 
        // self.angles  = None      # keypoint sizes                        [Nx1]         
        // self.des     = None      # keypoint descriptors                  [NxD] where D is the descriptor length 

        // # map points information arrays 
        // self.points   = None      # map points => self.points[idx] (if is not None) is the map point matched with self.kps[idx]
        // self.outliers = None      # outliers flags for map points (reset and set by pose_optimization())
        
        // self.kf_ref = None        # reference keyframe 

#[derive(Debug, Default, Clone)]
pub struct Frame {
    pub img: Mat,
    pub is_keyframe: bool,
    pub kps: VectorOfPoint2f,
    pub kpsu: VectorOfPoint2f,
    pub kpsn: VectorOfPoint2f,
    pub octaves: VectorOfi32,
    pub sizes: VectorOff32,
    pub angles: VectorOff32,
    pub des: Mat,
    pub points: VectorOfPoint3f,
    pub outliers: VectorOfi32,
    pub kf_ref: i32,
    pub id: i32,
    pub pose: CameraPose,
    pub camera: Camera
  }

  static mut  frame_id: i32=-1;


pub fn get_new_frame_id() -> i32
{
  unsafe{
    frame_id+=1;
    frame_id
  }
}

  impl Frame {
    
    //Construct person
    pub fn new(image: &Mat, camera: &Camera) -> Frame {
        let mut this_frame = Frame {
            img: image.clone(),
            is_keyframe: false,
            kps: VectorOfPoint2f::new(),
            kpsu: VectorOfPoint2f::new(),
            kpsn: VectorOfPoint2f::new(),
            octaves: VectorOfi32::new(),
            sizes: VectorOff32::new(),
            angles: VectorOff32::new(),
            des: Mat::default(),
            points: VectorOfPoint3f::new(),
            outliers: VectorOfi32::new(),
            kf_ref: -1,
            id: get_new_frame_id(),
            pose: CameraPose::default(),
            camera: camera.clone(),
      };

      this_frame.init();
      this_frame
    }

//     Camera.fx: 718.856
// Camera.fy: 718.856
// Camera.cx: 607.1928
// Camera.cy: 185.2157

// Camera.k1: 0.0
// Camera.k2: 0.0
// Camera.p1: 0.0
// Camera.p2: 0.0

// Camera.width: 1241
// Camera.height: 376
//D = [k1, k2, p1, p2, k3]
// k3=0
    pub fn default() -> Frame {
      let mut this_frame = Frame {
          img: Mat::default(),
          is_keyframe: false,
          kps: VectorOfPoint2f::new(),
          kpsu: VectorOfPoint2f::new(),
          kpsn: VectorOfPoint2f::new(),
          octaves: VectorOfi32::new(),
          sizes: VectorOff32::new(),
          angles: VectorOff32::new(),
          des: Mat::default(),
          points: VectorOfPoint3f::new(),
          outliers: VectorOfi32::new(),
          kf_ref: -1,
          id: -1,
          pose: CameraPose::default(),
          camera: Camera::new(1241, 376, 718.856, 718.856, 607.1928, 185.2157, VectorOff32::from(vec![0.0,0.0,0.0,0.0,0.0]))
    };

    //this_frame.init();
    this_frame
  }



  // if img is not None:
  // #self.H, self.W = img.shape[0:2]                 
  // if Frame.is_store_imgs: 
  //     self.img = img.copy()  
  // else: 
  //     self.img = None                    
  // if kps_data is None:   
  //     self.kps, self.des = Frame.tracker.detectAndCompute(img)                                                         
  //     # convert from a list of keypoints to arrays of points, octaves, sizes  
  //     kps_data = np.array([ [x.pt[0], x.pt[1], x.octave, x.size, x.angle] for x in self.kps ], dtype=np.float32)                            
  //     self.kps     = kps_data[:,:2]    
  //     self.octaves = np.uint32(kps_data[:,2]) #print('octaves: ', self.octaves)                      
  //     self.sizes   = kps_data[:,3]
  //     self.angles  = kps_data[:,4]       
  // else:
  //     # FIXME: this must be updated according to the new serialization 
  //     #self.kpsu, self.des = des, np.array(list(range(len(des)))*32, np.uint8).reshape(32, len(des)).T
  //     pass 
  // self.kpsu = self.camera.undistort_points(self.kps) # convert to undistorted keypoint coordinates             
  // self.kpsn = self.camera.unproject_points(self.kpsu)
  // self.points = np.array( [None]*len(self.kpsu) )  # init map points
  // self.outliers = np.full(self.kpsu.shape[0], False, dtype=bool)

  pub fn orb_extract(img1: &Mat, kp1: &mut VectorOfKeyPoint, des1: &mut Mat)
  {

        //let img1 = msg.get_frame().grayscale_to_cv_mat();
        let mut orb: PtrOfORB =  ORB::default().unwrap();
        orb.set_max_features(2000).unwrap();

        orb.detect_and_compute(&img1,&Mat::default(),  kp1,  des1, false).unwrap();

  }









  pub fn init(&mut self)
  {
  // if kps_data is None:   
  //     self.kps, self.des = Frame.tracker.detectAndCompute(img)                                                         
  //     # convert from a list of keypoints to arrays of points, octaves, sizes  
  //     kps_data = np.array([ [x.pt[0], x.pt[1], x.octave, x.size, x.angle] for x in self.kps ], dtype=np.float32)                            
  //     self.kps     = kps_data[:,:2]    
  //     self.octaves = np.uint32(kps_data[:,2]) #print('octaves: ', self.octaves)                      
  //     self.sizes   = kps_data[:,3]
  //     self.angles  = kps_data[:,4]       
  // else:
  //     # FIXME: this must be updated according to the new serialization 
  //     #self.kpsu, self.des = des, np.array(list(range(len(des)))*32, np.uint8).reshape(32, len(des)).T
  //     pass 
  // self.kpsu = self.camera.undistort_points(self.kps) # convert to undistorted keypoint coordinates             
  // self.kpsn = self.camera.unproject_points(self.kpsu)
  // self.points = np.array( [None]*len(self.kpsu) )  # init map points
  // self.outliers = np.full(self.kpsu.shape[0], False, dtype=bool)  

  let mut kps_1 = VectorOfKeyPoint::new();
  Frame::orb_extract(&self.img, &mut kps_1, &mut self.des);

  let pt_indx = opencv::types::VectorOfi32::new();
  KeyPoint::convert(&kps_1, &mut self.kps, &pt_indx).unwrap();
  self.octaves = kps_1.iter().map(|x|{x.octave} ).collect();
  self.sizes = kps_1.iter().map(|x|{x.size} ).collect();
  self.angles = kps_1.iter().map(|x|{x.angle} ).collect();
  self.kpsu = self.camera.undistort_points(&self.kps);
  self.kpsn = self.camera.unproject_points(&self.kpsu);
  self.points = VectorOfPoint3f::from(vec![Point3f::default(); self.kpsu.len()]);
  //self.outliers = np.full(self.kpsu.shape[0], False, dtype=bool)  
    
  /* 
  println!("self.kpsu {:?} self.kpsn  {:?}  length {:?}", self.kpsu.get(0), self.kpsn.get(0), self.kps.len());
  panic!("Done");
*/

  }


  pub fn update_pose_old(&mut self, pose: &Mat)
  {
    //self.pose.update(pose)
  }

  pub fn update_pose(&mut self, pose: &Poseg2o)
  {
    self.pose.update(pose)
  }


  // def compute_points_median_depth(self, points3d = None):
  // with self._lock_pose:        
  //     Rcw2 = self._pose.Rcw[2,:3]  # just 2-nd row 
  //     tcw2 = self._pose.tcw[2]   # just 2-nd row                    
  // if points3d is None: 
  //     with self._lock_features:                
  //         points3d = np.array([p.pt for p in self.points if p is not None])
  // if len(points3d)>0:
  //     z = np.dot(Rcw2, points3d[:,:3].T) + tcw2 
  //     z = sorted(z) 
  //     return z[ ( len(z)-1)//2 ]                
  // else:
  //     Printer.red('frame.compute_points_median_depth() with no points')
  //     return -1 

  pub fn compute_points_median_depth(&self, points3d: &VectorOfPoint3f) -> f32
  {
    //  let Rcw2 = self.pose.Rcw.row_range(&Range::new(2,2).unwrap()).unwrap(); //[2,:3] //  # just 2-nd row 
    //  let tcw2 = self.pose.tcw.row_range(&Range::new(2,2).unwrap()).unwrap(); // [2]    //  # just 2-nd row 

    let mut depths = vec![];

    for pt in points3d
    {

      let r1: f32 = *self.pose.Rcw.at_2d(2, 0).unwrap();
      let r2: f32 = *self.pose.Rcw.at_2d(2, 1).unwrap();
      let r3: f32 = *self.pose.Rcw.at_2d(2, 2).unwrap();
      let t1: f32 = *self.pose.tcw.at_2d(2, 0).unwrap();

      let depth = r1*pt.x + r2*pt.y + r3*pt.z+ t1;
      if depth.is_finite()
      {
        depths.push(depth);
      }
      else
      {
        depths.push(0.0);
      }
      
    }

    if depths.len() ==0
    {
      return -1.0;
    }
    //println!("{:?}", depths);
    depths.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let d_median = depths[depths.len()/2];

    d_median
  }


  pub fn update_rotation_and_translation(&mut self, R: &Mat, t: &Mat)
  {
    //let Rt = get_transformation_matrix(R, t);
    let pose_Rt = poseRt(R,t);
    self.pose.update(&pose_Rt);
  }
}

pub type DVVectorOfMatchPair = nalgebra::MatrixXx2<i32>;

// # match frames f1 and f2
// # out: a vector of match index pairs [idx1[i],idx2[i]] such that the keypoint f1.kps[idx1[i]] is matched with f2.kps[idx2[i]]
// def match_frames(f1, f2, ratio_test=None):     
//     idx1, idx2 = Frame.feature_matcher.match(f1.des, f2.des, ratio_test)
//     idx1 = np.asarray(idx1)
//     idx2 = np.asarray(idx2)   
//     return idx1, idx2     

pub fn match_frames(f1: &Frame, f2: &Frame, ratio_test: f32) ->  (VectorOfi32, VectorOfi32)//  (opencv::types::VectorOfPoint2f, opencv::types::VectorOfPoint2f, DVVectorOfMatchPair)
{
      
      // BFMatcher to get good matches
      let bfmtch = BFMatcher::create(6 , true).unwrap(); 
      let mut mask = Mat::default(); 
      let mut matches = VectorOfDMatch::new();
      bfmtch.train_match(&f2.des, &f1.des, &mut matches, &mut mask).unwrap(); 

      // Sort the matches based on the distance in ascending order
      // Using O(n^2) sort here. Need to make the code use cv sort function
      // by providing custom comparator

      let mut sorted_matches = VectorOfDMatch::new();
      let mut added = vec![false; matches.len()];
      for i in 0.. matches.len() {
          if added[i] == true {
              continue;
          }
          let mut mn = i;
          let mut dist = matches.get(i).unwrap().distance;
          for j in 0..matches.len() {
              let dmatch2 = matches.get(j).unwrap();
              if dist > dmatch2.distance && !added[j] {
                  mn = j;
                  dist = dmatch2.distance;
              }        
          }      
          let dmatch1 = matches.get(mn).unwrap();
          sorted_matches.push(dmatch1);
          added[mn] = true;
      }

      // Point vectors to hold the corresponding matched points 
      let mut p2f1 = core::Vector::<core::Point2f>::new(); 
      let mut p2f2 = core::Vector::<core::Point2f>::new();

      let mut match_indices = nalgebra::MatrixXx2::<i32>::from_element(sorted_matches.len(), 0);//from_element(sorted_matches.len(), 2, 0.0);

      let mut idx_1 = VectorOfi32::new();
      let mut idx_2 = VectorOfi32::new();

      for i in 0..sorted_matches.len(){
          p2f1.push(f1.kps.get(sorted_matches.get(i).unwrap().train_idx.try_into().unwrap() ).unwrap());
          p2f2.push(f2.kps.get(sorted_matches.get(i).unwrap().query_idx.try_into().unwrap() ).unwrap());

          match_indices[(i,0)]  = sorted_matches.get(i).unwrap().train_idx.try_into().unwrap();
          match_indices[(i,1)]  = sorted_matches.get(i).unwrap().query_idx.try_into().unwrap();


          idx_1.push(sorted_matches.get(i).unwrap().train_idx.try_into().unwrap());
          idx_2.push(sorted_matches.get(i).unwrap().query_idx.try_into().unwrap());

      }

      //(p2f1, p2f2, match_indices)
      (idx_1, idx_2)
}
