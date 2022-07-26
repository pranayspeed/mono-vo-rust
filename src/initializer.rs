


use cv_convert::TryFromCv;
use opencv::types::*;
use opencv::core::*;

use crate::camera::Camera;
use crate::camera_pose::Poseg2o;
use crate::frame::Frame;
use crate::frame::match_frames;
use crate::parameters::InitializerNumMinFeatures;
use crate::parameters::kFeatureMatchRatioTestInitializer;
use crate::parameters::kInitializerDesiredMedianDepth;
use crate::parameters::kInitializerNumMinTriangulatedPoints;

use std::collections::VecDeque;
//use crate::parameters::Parameters;

use crate::utils_geom::*;

// class InitializerOutput(object):
// def __init__(self):    
//     self.pts = None    # 3d points [Nx3]
//     self.kf_cur = None 
//     self.kf_ref = None 
//     self.idxs_cur = None 
//     self.idxs_ref = None 


// class Initializer(object):
// def __init__(self):
//     self.mask_match = None
//     self.mask_recover = None 
//     self.frames = deque(maxlen=kMaxLenFrameDeque)  # deque with max length, it is thread-safe      
//     self.idx_f_ref = 0   # index of the reference frame in self.frames buffer  
//     self.f_ref = None 
    
//     self.num_min_features = Parameters.kInitializerNumMinFeatures
//     self.num_min_triangulated_points = Parameters.kInitializerNumMinTriangulatedPoints       
//     self.num_failures = 0 


   
const kRansacThresholdNormalized : f64 = 1.0;// 0.0003;//  # metric threshold used for normalized image coordinates 
const kRansacProb : f64 = 0.999;

const kMaxIdDistBetweenIntializingFrames : i32 = 5;//   # N.B.: worse performances with values smaller than 5!

//const kFeatureMatchRatioTestInitializer : f32 = kFeatureMatchRatioTestInitializer;

const kNumOfFailuresAfterWichNumMinTriangulatedPointsIsHalved : i32 = 10;

const kMaxLenFrameDeque: i32 = 20;

//NOTE: for now not using keyframe and keeping it simple.
// refering all keyframe fields as Frames
#[derive(Debug, Default, Clone)]
pub struct InitializerOutput
{
    pub pts: VectorOfPoint3f,
    pub kf_cur: Frame,
    pub kf_ref: Frame,
    pub idxs_cur: VectorOfi32,
    pub idxs_ref: VectorOfi32,
} 
impl InitializerOutput {
    //Construct person
    pub fn new() -> InitializerOutput {
        InitializerOutput {
            pts: VectorOfPoint3f::new(),
            kf_cur: Frame::default(),
            kf_ref: Frame::default(),
            idxs_cur: VectorOfi32::new(),
            idxs_ref: VectorOfi32::new(),
      }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Initializer {
    // pts: VectorOfPoint3f,
    // kf_cur: Frame,
    // kf_ref: Frame,
    // idxs_cur: i32,
    // idxs_ref: i32,
    out: InitializerOutput,
    mask_match: Mat,
    mask_recover: Mat,
    frames:  VecDeque<Frame>,
    idx_f_ref: i32,
    f_ref: Frame,
    num_min_features: i32,
    num_min_triangulated_points: i32,
    num_failures: i32,

  }

  impl Initializer {
    //Construct person
    pub fn new() -> Initializer {
        Initializer {
            // pts: VectorOfPoint3f::new(),
            // kf_cur: Frame::new(Mat::deafult()),
            // kf_ref: Frame::new(Mat::deafult()),
            // idxs_cur: -1,
            // idxs_ref: -1,
            out: InitializerOutput::new(),
            mask_match: Mat::default(),
            mask_recover:  Mat::default(),
            frames:  VecDeque::new(),
            idx_f_ref: 0,
            f_ref: Frame::default(),
            num_min_features: InitializerNumMinFeatures,
            num_min_triangulated_points: kInitializerNumMinTriangulatedPoints,
            num_failures: 0,
      }
    }

    pub fn default() -> Initializer {
        Initializer {
            // pts: VectorOfPoint3f::new(),
            // kf_cur: Frame::new(Mat::deafult()),
            // kf_ref: Frame::new(Mat::deafult()),
            // idxs_cur: -1,
            // idxs_ref: -1,
            out: InitializerOutput::new(),
            mask_match: Mat::default(),
            mask_recover:  Mat::default(),
            frames:  VecDeque::new(),
            idx_f_ref: 0,
            f_ref: Frame::default(),
            num_min_features: InitializerNumMinFeatures,
            num_min_triangulated_points: kInitializerNumMinTriangulatedPoints,
            num_failures: 0,
      }
    }


    // # fit essential matrix E with RANSAC such that:  p2.T * E * p1 = 0  where  E = [t21]x * R21
    // # out: Trc  homogeneous transformation matrix with respect to 'ref' frame,  pr_= Trc * pc_
    // # N.B.1: trc is estimated up to scale (i.e. the algorithm always returns ||trc||=1, we need a scale in order to recover a translation which is coherent with previous estimated poses)
    // # N.B.2: this function has problems in the following cases: [see Hartley/Zisserman Book]
    // # - 'geometrical degenerate correspondences', e.g. all the observed features lie on a plane (the correct model for the correspondences is an homography) or lie a ruled quadric 
    // # - degenerate motions such a pure rotation (a sufficient parallax is required) or anum_edges viewpoint change (where the translation is almost zero)
    // # N.B.3: the five-point algorithm (used for estimating the Essential Matrix) seems to work well in the degenerate planar cases [Five-Point Motion Estimation Made Easy, Hartley]
    // # N.B.4: as reported above, in case of pure rotation, this algorithm will compute a useless fundamental matrix which cannot be decomposed to return the rotation     
    // # N.B.5: the OpenCV findEssentialMat function uses the five-point algorithm solver by D. Nister => hence it should work well in the degenerate planar cases
    // def estimatePose(self, kpn_ref, kpn_cur):	     
    //     # here, the essential matrix algorithm uses the five-point algorithm solver by D. Nister (see the notes and paper above )     
    //     E, self.mask_match = cv2.findEssentialMat(kpn_cur, kpn_ref, focal=1, pp=(0., 0.), method=cv2.RANSAC, prob=kRansacProb, threshold=kRansacThresholdNormalized)                         
    //     _, R, t, mask = cv2.recoverPose(E, kpn_cur, kpn_ref, focal=1, pp=(0., 0.))                                                     
    //     return poseRt(R,t.T)  # Trc  homogeneous transformation matrix with respect to 'ref' frame,  pr_= Trc * pc_        

    // pub fn estimatePose(&self, kpn_ref, kpn_cur)
    // {

    // }
    pub fn estimatePose(
        &mut self,
        curr_features: &VectorOfPoint2f,
        prev_features: &VectorOfPoint2f,
      ) -> Poseg2o {
        //recovering the pose and the essential matrix
        let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
        //     E, self.mask_match = cv2.findEssentialMat(kpn_cur, kpn_ref, focal=1, pp=(0., 0.), method=cv2.RANSAC, prob=kRansacProb, threshold=kRansacThresholdNormalized)   
        //[TO_CHECK]: if the focal is to be set as 1 and pp as 0,0

        let focal = 1.0;//718.8560;
        let pp = Point2d::new(0.0,0.0); //Point2d::new(607.1928, 185.2157);
        let kRansacThresholdNormalized_ = 0.0003;//1.0;
        let kRansacProb_ = 0.999;

        let essential_mat = opencv::calib3d::find_essential_mat(
          &curr_features,
          &prev_features,
          focal,
          pp,
          opencv::calib3d::RANSAC,
          kRansacProb_,
          kRansacThresholdNormalized_,
          &mut self.mask_match,
        )
        .unwrap();
            //     _, R, t, mask = cv2.recoverPose(E, kpn_cur, kpn_ref, focal=1, pp=(0., 0.))   
        opencv::calib3d::recover_pose(
          &essential_mat,
          &curr_features,
          &prev_features,
          &mut recover_r,
          &mut recover_t,
          focal,
          pp,
          &mut mask,
        )
        .unwrap();
        

        poseRt(&recover_r, &recover_t) //.t().unwrap().to_mat().unwrap())
      }

    // # push the first image
    // def init(self, f_cur):
    //     self.frames.append(f_cur)    
    //     self.f_ref = f_cur  
    pub fn init(&mut self, f_cur: &Frame) 
    {
        self.frames.push_back(f_cur.clone());
        self.f_ref = f_cur.clone();
    }



    // # actually initialize having two available images 
    // def initialize(self, f_cur, img_cur):

    pub fn initialize(&mut self, f_cur: &mut Frame, img_cur: &Mat) -> (InitializerOutput, bool)
    {
    //     if self.num_failures > kNumOfFailuresAfterWichNumMinTriangulatedPointsIsHalved: 
    //         self.num_min_triangulated_points = 0.5 * Parameters.kInitializerNumMinTriangulatedPoints
    //         self.num_failures = 0
    //         Printer.orange('Initializer: halved min num triangulated features to ', self.num_min_triangulated_points)            

        if self.num_failures > kNumOfFailuresAfterWichNumMinTriangulatedPointsIsHalved
        {
            self.num_min_triangulated_points = (0.5 * kInitializerNumMinTriangulatedPoints as f32) as i32;
            self.num_failures = 0;
            //println!("Initializer: halved min num triangulated features to  {:?}", self.num_min_triangulated_points);
        }

    //     # prepare the output 
    //     out = InitializerOutput()
    //     is_ok = False 

    //     #print('num frames: ', len(self.frames))

        let mut out = InitializerOutput::new();
        let mut is_ok = false;
        //println!("num frames: {:?} ", self.frames.len());

    //     # if too many frames have passed, move the current idx_f_ref forward 
    //     # this is just one very simple policy that can be used 
    //     if self.f_ref is not None: 
    //         if f_cur.id - self.f_ref.id > kMaxIdDistBetweenIntializingFrames: 
    //             self.f_ref = self.frames[-1]  # take last frame in the buffer
    //             #self.idx_f_ref = len(self.frames)-1  # take last frame in the buffer
    //             #self.idx_f_ref = self.frames.index(self.f_ref)  # since we are using a deque, the code of the previous commented line is not valid anymore 
    //             #print('*** idx_f_ref:',self.idx_f_ref)
    //     #self.f_ref = self.frames[self.idx_f_ref] 
    //     f_ref = self.f_ref 
    //     #print('ref fid: ',self.f_ref.id,', curr fid: ', f_cur.id, ', idxs_ref: ', self.idxs_ref)

        if self.f_ref.id != -1
        {
            if f_cur.id - self.f_ref.id > kMaxIdDistBetweenIntializingFrames
            {
                self.f_ref = self.frames.back().unwrap().clone()
            }
        }
        let mut f_ref = self.f_ref.clone();
        //println!("ref fid: {:?} ,curr fid: {:?}",self.f_ref.id, f_cur.id);
    //     # append current frame 
    //     self.frames.append(f_cur)
        self.frames.push_back(f_cur.clone());

                


    //     # if the current frames do no have enough features exit 
    //     if len(f_ref.kps) < self.num_min_features or len(f_cur.kps) < self.num_min_features:
    //         Printer.red('Inializer: ko - not enough features!') 
    //         self.num_failures += 1
    //         return out, is_ok

        // if the current frames do no have enough features exit 
        if f_ref.kps.len() < self.num_min_features as usize || f_cur.kps.len() < self.num_min_features as usize
        {
            //println!("Inializer: ko - not enough features!");
            self.num_failures += 1;
            return (out , is_ok);   
        }

    //     # find keypoint matches
    //     idxs_cur, idxs_ref = match_frames(f_cur, f_ref, kFeatureMatchRatioTestInitializer)      
        let (idxs_cur, idxs_ref) = match_frames(f_cur, &f_ref, kFeatureMatchRatioTestInitializer);

    //     print('|------------')        
    //     #print('deque ids: ', [f.id for f in self.frames])
    //     print('initializing frames ', f_cur.id, ', ', f_ref.id)
    //     print("# keypoint matches: ", len(idxs_cur))  
                
    //     Trc = self.estimatePose(f_ref.kpsn[idxs_ref], f_cur.kpsn[idxs_cur])
    //     Tcr = inv_T(Trc)  # Tcr w.r.t. ref frame 
    //     f_ref.update_pose(np.eye(4))        
    //     f_cur.update_pose(Tcr)
        
        //println!("|------------") ;
        //println!("initializing frames {:?}, {:?} ", f_cur.id, f_ref.id);
        //println!("# keypoint matches: {:?} ", idxs_cur.len()) ;    
        

        let mut curr_features = VectorOfPoint2f::new();
        let mut prev_features = VectorOfPoint2f::new();
        for idx in idxs_cur.iter()
        {
            curr_features.push(f_cur.kpsn.get(idx as usize).unwrap());
        }
        for idx in idxs_ref.iter()
        {
            prev_features.push(f_ref.kpsn.get(idx as usize).unwrap());
        }

        let Trc = self.estimatePose(&curr_features, &prev_features);

        let init_pose = Poseg2o::default();
        f_ref.update_pose(&init_pose); 

        let Tcr = inv_T(&Trc); //  # Tcr w.r.t. ref frame         
        f_cur.update_pose(&Tcr);

        /* 
        for i in 0..Trc.Tcw().rows()
        {
            for j in 0..Trc.Tcw().cols()
            {
                print!("\t {:?}", Trc.Tcw().at_2d::<f32>(i, j));
            }
            println!("");
        }
        println!("\t\n\n");

        for i in 0..Tcr.Tcw().rows()
        {
            for j in 0..Tcr.Tcw().cols()
            {
                print!("\t {:?}", Tcr.Tcw().at_2d::<f32>(i, j));
            }
            println!("");
        }
        println!("\t");        
        panic!("Done");
        
        //println!("f_ref ->-> -> -> ->  {:?}", f_ref.pose._pose._pose.to_matrix());
        //println!("f_curr ->-> -> -> ->   {:?}", f_cur.pose._pose._pose.to_matrix());
        */
    
    //     # remove outliers from keypoint matches by using the mask computed with inter frame pose estimation        
    //     mask_idxs = (self.mask_match.ravel() == 1)
    //     self.num_inliers = sum(mask_idxs)
    //     print('# keypoint inliers: ', self.num_inliers )
    //     idx_cur_inliers = idxs_cur[mask_idxs]
    //     idx_ref_inliers = idxs_ref[mask_idxs]
        

        let mut idx_cur_inliers= VectorOfi32::new();
        let mut idx_ref_inliers= VectorOfi32::new();

        for i in 0..self.mask_match.rows()
        {
            if *self.mask_match.at_2d::<u8>(i,0).unwrap()==1
            {
                idx_cur_inliers.push(idxs_cur.get(i as usize).unwrap());
                idx_ref_inliers.push(idxs_ref.get(i as usize).unwrap());
                
            }
            
        }

//======================================================================
////////////// TODO: implement map
    //     # create a temp map for initializing 
    //     map = Map()
    //     f_ref.reset_points()
    //     f_cur.reset_points()
        
    //     #map.add_frame(f_ref)        
    //     #map.add_frame(f_cur)  
        
    //     kf_ref = KeyFrame(f_ref)
    //     kf_cur = KeyFrame(f_cur, img_cur) 
    
    
    //     map.add_keyframe(kf_ref)        
    //     map.add_keyframe(kf_cur)      
//======================================================================

        let mut kf_ref: Frame = f_ref.clone();
        let mut kf_cur: Frame = f_cur.clone();
        kf_cur.img = img_cur.clone();

        
        let mut kf_curr_kpsn = VectorOfPoint2f::new();
        let mut kf_ref_kpsn = VectorOfPoint2f::new();
        for idx in idx_cur_inliers.iter()
        {
            kf_curr_kpsn.push(kf_cur.kpsn.get(idx as usize).unwrap());
        }
        for idx in idx_ref_inliers.iter()
        {
            kf_ref_kpsn.push(kf_ref.kpsn.get(idx as usize).unwrap());
        }

    //     pts3d, mask_pts3d =                      (kf_cur.Tcw, kf_ref.Tcw, kf_cur.kpsn[idx_cur_inliers], kf_ref.kpsn[idx_ref_inliers])

        let (pts3d, mask_pts3d) = triangulate_normalized_points(&kf_cur.pose.Tcw, &kf_ref.pose.Tcw, &kf_curr_kpsn, &kf_ref_kpsn);


        //======================================================================
    //     new_pts_count, mask_points, _ = map.add_points(pts3d, mask_pts3d, kf_cur, kf_ref, idx_cur_inliers, idx_ref_inliers, img_cur, do_check=True, cos_max_parallax=Parameters.kCosMaxParallaxInitializer)
    //     print("# triangulated points: ", new_pts_count)   
    //======================================================================
    

    
    //     if new_pts_count > self.num_min_triangulated_points:  
    //         err = map.optimize(verbose=False, rounds=20,use_robust_kernel=True)
    //         print("init optimization error^2: %f" % err)         

    //         num_map_points = len(map.points)
    //         print("# map points:   %d" % num_map_points)   
    //         is_ok = num_map_points > self.num_min_triangulated_points

        //TODO: implement triangulation point check
        is_ok = true;
        // ========ignoring optimization and map processing for now===================================



    //         out.pts = pts3d[mask_points]
    //         out.kf_cur = kf_cur
    //         out.idxs_cur = idx_cur_inliers[mask_points]        
    //         out.kf_ref = kf_ref 
    //         out.idxs_ref = idx_ref_inliers[mask_points]

    let mut filtered_3dpts = VectorOfPoint3f::new();
    let mut idx_cur_inliers_mask = VectorOfi32::new();
    let mut idx_ref_inliers_mask = VectorOfi32::new();    

        for idx in mask_pts3d.iter()
        {
            //if mask_pts3d.get(idx as usize).unwrap()==1
            if idx ==1
            {
                filtered_3dpts.push(pts3d.get(idx as usize).unwrap());

                idx_cur_inliers_mask.push(idx_cur_inliers.get(idx as usize).unwrap());
                idx_ref_inliers_mask.push(idx_ref_inliers.get(idx as usize).unwrap());
            }
        }

        out.pts = filtered_3dpts;
        out.kf_cur = kf_cur.clone();
        out.idxs_cur = idx_cur_inliers_mask;
        out.kf_ref = kf_ref.clone();
        out.idxs_ref = idx_ref_inliers_mask;

    //         # set scene median depth to equal desired_median_depth'
    //         desired_median_depth = Parameters.kInitializerDesiredMedianDepth
    //         median_depth = kf_cur.compute_points_median_depth(out.pts)        
    //         depth_scale = desired_median_depth/median_depth 
    //         print('forcing current median depth ', median_depth,' to ',desired_median_depth)

        let desired_median_depth = kInitializerDesiredMedianDepth;
        let median_depth = kf_cur.compute_points_median_depth(&out.pts);

        let depth_scale = desired_median_depth as f32/median_depth ;

        //out.pts = out.pts * depth_scale;
        
    //         out.pts[:,:3] = out.pts[:,:3] * depth_scale  # scale points 
    //         tcw = kf_cur.tcw * depth_scale  # scale initial baseline 
    //         kf_cur.update_translation(tcw)
            
    //     map.delete()
  
    //     if is_ok:
    //         Printer.green('Inializer: ok!')    
    //     else:
    //         self.num_failures += 1            
    //         Printer.red('Inializer: ko!')                         
    //     print('|------------')               
    //     return out, is_ok



        return (out , is_ok); 

    }











}