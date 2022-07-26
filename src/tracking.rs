
use cv_convert::TryFromCv;
use opencv::{types::{VectorOfu8, VectorOfi32}, prelude::{Mat, MatExprTraitConst}, core::CV_32F};

use crate::{slam::Slam, camera::Camera, initializer::Initializer, frame::{Frame, match_frames}, map::Map, parameters::kFeatureMatchRatioTestInitializer, utils_geom::{estimate_pose_ess_mat, inv_T, get_rotation_from_transformation, scale_transform}, camera_pose::Poseg2o};

use opencv::types::*;
use opencv::core::*;


const kNumMinInliersEssentialMat: i32 = 8;

#[derive(Debug, Default, Clone, PartialEq)]
pub enum SlamState {
    #[default] NO_IMAGES_YET,
    NOT_INITIALIZED,
    OK,
}

#[derive(Debug, Default, Clone)]
pub struct Tracking{

             
//system :  Slam,                     
pub camera : Camera,
pub map: Map,
pub local_mapping: bool,
pub initializer: Initializer,

// self.motion_model = MotionModel()  # motion model for current frame pose prediction without damping  
// #self.motion_model = MotionModelDamping()  # motion model for current frame pose prediction with damping       

// self.dyn_config = SLAMDynamicConfig()
// self.descriptor_distance_sigma = Parameters.kMaxDescriptorDistance 
// self.reproj_err_frame_map_sigma = Parameters.kMaxReprojectionDistanceMap        

// self.max_frames_between_kfs = int(system.camera.fps) 
// self.min_frames_between_kfs = 0         

// self.state = SlamState.NO_IMAGES_YET
pub state : SlamState,
pub num_matched_kps: i32 ,// = None           # current number of matched keypoints 
pub num_inliers: i32 ,// = None               # current number of matched points 
pub num_matched_map_points: i32 ,// = None         # current number of matched map points (matched and found valid in current pose optimization)     
pub num_kf_ref_tracked_points: i32 ,// = None # number of tracked points in k_ref (considering a minimum number of observations)      
pub mask_match: VectorOfu8 ,// = None 

// self.pose_is_ok = False 
// self.predicted_pose = None 
// self.velocity = None 

pub f_cur: Frame,// = None 
pub idxs_cur: VectorOfi32,//  = None 
pub f_ref: Frame,//  = None 
pub idxs_ref: VectorOfi32,//  = None        
pub kf_ref: Frame,// = None    # reference keyframe (in general, different from last keyframe depending on the used approach)
pub kf_last: Frame,// = None   # last keyframe  
// self.kid_last_BA = -1 # last keyframe id when performed BA 

// self.local_keyframes = [] # local keyframes 
// self.local_points    = [] # local points 
 
// self.tracking_history = TrackingHistory()

// self.timer_verbose = kTimerVerbose  # set this to True if you want to print timings 
// self.timer_main_track = TimerFps('Track', is_verbose = self.timer_verbose)
// self.timer_pose_opt = TimerFps('Pose optimization', is_verbose = self.timer_verbose)
// self.timer_seach_frame_proj = TimerFps('Search frame by proj', is_verbose = self.timer_verbose) 
// self.timer_match = TimerFps('Match', is_verbose = self.timer_verbose)                   
// self.timer_pose_est = TimerFps('Ess mat pose estimation', is_verbose = self.timer_verbose)
// self.timer_frame = TimerFps('Frame', is_verbose = self.timer_verbose)
// self.timer_seach_map = TimerFps('Search map', is_verbose = self.timer_verbose)     

// self.init_history = True 
// self.poses = []       # history of poses
// self.t0_est = None    # history of estimated translations      
// self.t0_gt = None     # history of ground truth translations (if available)
// self.traj3d_est = []  # history of estimated translations centered w.r.t. first one
// self.traj3d_gt = []   # history of estimated ground truth translations centered w.r.t. first one                 

pub cur_R: Mat,// = None # current rotation w.r.t. world frame  
pub cur_t: Mat,// = None # current translation w.r.t. world frame 
// self.trueX, self.trueY, self.trueZ = None, None, None
// self.groundtruth = system.groundtruth  # not actually used here; could be used for evaluating performances 
}


impl Tracking
{

    pub fn new(system: &Slam) -> Tracking {
        Tracking {
            camera : system.camera.clone(),
            map: Map::default(),
            local_mapping: false,
            initializer: Initializer::new(),

            state : SlamState::NO_IMAGES_YET, //SlamState.NO_IMAGES_YET

            num_matched_kps: 0 ,// = None           # current number of matched keypoints 
            num_inliers: 0 ,// = None               # current number of matched points 
            num_matched_map_points: 0 ,// = None         # current number of matched map points (matched and found valid in current pose optimization)     
            num_kf_ref_tracked_points: 0 ,// = None # number of tracked points in k_ref (considering a minimum number of observations)      
            mask_match: VectorOfu8::new() ,// = None 
            f_cur: Frame::default(),// = None 
            idxs_cur: VectorOfi32::new(),//  = None 
            f_ref: Frame::default(),//  = None 
            idxs_ref: VectorOfi32::new(),//  = None 
                    
            kf_ref: Frame::default(),// = None    # reference keyframe (in general, different from last keyframe depending on the used approach)
            kf_last: Frame::default(),// = None   # last keyframe  
                        
            cur_R: Mat::eye(3, 3, CV_32F).unwrap().to_mat().unwrap(),// = None # current rotation w.r.t. world frame  
            cur_t: Mat::zeros(3, 1, CV_32F).unwrap().to_mat().unwrap(),// = None # current translation w.r.t. world frame 

        }
    }

    pub fn track(&mut self, img: &Mat, frame_id: i32, timestamp: f32)
    {

       
        
//     Printer.cyan('@tracking')
//     time_start = time.time()
            
//     # check image size is coherent with camera params 
//     print("img.shape: ", img.shape)
//     print("camera ", self.camera.height," x ", self.camera.width)
//     assert img.shape[0:2] == (self.camera.height, self.camera.width)   
//     if timestamp is not None: 
//         print('timestamp: ', timestamp)  
    //println!("Tracking started...");
    //println!("img.shape: ({:?} x {:?})", img.rows(), img.cols());
    //println!("camera {:?}  x  {:?} ", self.camera.height, self.camera.width);
  


//     self.timer_main_track.start()

//     # build current frame 
//     self.timer_frame.start()        
//     f_cur = Frame(img, self.camera, timestamp=timestamp) 
//     self.f_cur = f_cur 
//     print("frame: ", f_cur.id)        
//     self.timer_frame.refresh()   
    
        let mut f_cur = Frame::new(img, &self.camera);
        self.f_cur = f_cur.clone();
        //println!("frame: {:?}", f_cur.id) ;
        //println!("ROTATION MATRIX .... : {:?}    {:?}", frame_id , f_cur.pose.Rcw );


//     # reset indexes of matches 
//     self.idxs_ref = [] 
//     self.idxs_cur = []           
        self.idxs_ref = VectorOfi32::new();
        self.idxs_cur = VectorOfi32::new();

//     if self.state == SlamState.NO_IMAGES_YET: 
//         # push first frame in the inizializer 
//         self.intializer.init(f_cur) 
//         self.state = SlamState.NOT_INITIALIZED
//         return # EXIT (jump to second frame)
        if self.state == SlamState::NO_IMAGES_YET
        {
            //println!(" NO_IMAGES_YET.....");
            self.initializer.init(&f_cur);
            self.state= SlamState::NOT_INITIALIZED;
            return
        }

        
//     if self.state == SlamState.NOT_INITIALIZED:
//         # try to inizialize 
//         initializer_output, intializer_is_ok = self.intializer.initialize(f_cur, img)

        if self.state == SlamState::NOT_INITIALIZED
        {
            //println!("Not initialized");
            let (initializer_output, intializer_is_ok) = self.initializer.initialize(&mut f_cur, img);


//         if intializer_is_ok:
            if intializer_is_ok
            {
//             kf_ref = initializer_output.kf_ref
//             kf_cur = initializer_output.kf_cur   
                let kf_ref = initializer_output.kf_ref.clone();
                let kf_cur = initializer_output.kf_cur.clone();


              //TODO: add MAP implementation
//===========================================================================
//             # add the two initialized frames in the map 
//             self.map.add_frame(kf_ref) # add first frame in map and update its frame id
//             self.map.add_frame(kf_cur) # add second frame in map and update its frame id  
//             # add the two initialized frames as keyframes in the map               
//             self.map.add_keyframe(kf_ref) # add first keyframe in map and update its kid
//             self.map.add_keyframe(kf_cur) # add second keyframe in map and update its kid                        
//             kf_ref.init_observations()
//             kf_cur.init_observations()
            
        
        self.map.add_frame(&kf_ref);
        self.map.add_frame(&kf_cur);

        self.map.add_keyframe(&kf_ref);
        self.map.add_keyframe(&kf_cur);     
        
        
//             # add points in map 
//             new_pts_count,_,_ = self.map.add_points(initializer_output.pts, None, kf_cur, kf_ref, initializer_output.idxs_cur, initializer_output.idxs_ref, img, do_check=False)
//             Printer.green("map: initialized %d new points" % (new_pts_count))                 
//             # update covisibility graph connections 
//             kf_ref.update_connections()
//             kf_cur.update_connections()     
//===========================================================================   


//             # update tracking info            
//             self.f_cur = kf_cur 
//             self.f_cur.kf_ref = kf_ref                         
//             self.kf_ref = kf_cur  # set reference keyframe 
//             self.kf_last = kf_cur # set last added keyframe                                     
//             self.map.local_map.update(self.kf_ref)
//             self.state = SlamState.OK      

                self.f_cur = kf_cur.clone();
                self.f_cur.kf_ref = kf_ref.id; //TODO: switch to Keyframe, might cause issues
                self.kf_ref = kf_ref;
                self.kf_last = kf_cur.clone();

                self.state = SlamState::OK;

//             self.update_tracking_history()
//             self.motion_model.update_pose(kf_cur.timestamp,kf_cur.position,kf_cur.quaternion)
//             self.motion_model.is_ok = False   # after initialization you cannot use motion model for next frame pose prediction (time ids of initialized poses may not be consecutive)
                

               
//             self.intializer.reset()
            
//             if kUseDynamicDesDistanceTh: 
//                 self.descriptor_distance_sigma = self.dyn_config.update_descriptor_stat(kf_ref, kf_cur, initializer_output.idxs_ref, initializer_output.idxs_cur)                     
            }

//         return # EXIT (jump to next frame)
            return
        }


//     # get previous frame in map as reference        
//     f_ref   = self.map.get_frame(-1) 
//     #f_ref_2 = self.map.get_frame(-2)
//     self.f_ref = f_ref 
        //println!("Just before get last frame ............");
        let mut f_ref = self.map.get_last_frame();
        //println!(" f_ref ---> \n {:?} ", f_ref.pose._pose._pose.to_matrix() );
        //println!("Just After get last frame ............");
        self.f_ref = f_ref.clone();

//     # add current frame f_cur to map                  
//     self.map.add_frame(f_cur)          
//     self.f_cur.kf_ref = self.kf_ref  
    
        // self.map.add_frame(&f_cur);
        // self.f_cur.kf_ref = self.kf_ref.id;

//     # reset pose state flag 
//     self.pose_is_ok = False 
        
        //=============Assuming Pose is ok = true

//     with self.map.update_lock:
//         # check for map point replacements in previous frame f_ref (some points might have been replaced by local mapping during point fusion)
//         self.f_ref.check_replaced_map_points()
                                                
//         if kUseDynamicDesDistanceTh:         
//             print('descriptor_distance_sigma: ', self.descriptor_distance_sigma)
//             self.local_mapping.descriptor_distance_sigma = self.descriptor_distance_sigma
                
//         # udpdate (velocity) old motion model                                             # c1=ref_ref, c2=ref, c3=cur;  c=cur, r=ref
//         #self.velocity = np.dot(f_ref.pose, inv_T(f_ref_2.pose))                          # Tc2c1 = Tc2w * Twc1   (predicted Tcr)
//         #self.predicted_pose = g2o.Isometry3d(np.dot(self.velocity, f_ref.pose))          # Tc3w = Tc2c1 * Tc2w   (predicted Tcw)        
                                                
//         # set intial guess for current pose optimization                         
//         if kUseMotionModel and self.motion_model.is_ok:
//             print('using motion model for next pose prediction')                   
//             # update f_ref pose according to its reference keyframe (the pose of the reference keyframe could be updated by local mapping)
//             self.f_ref.update_pose(self.tracking_history.relative_frame_poses[-1] * self.f_ref.kf_ref.isometry3d)                                  
//             # predict pose by using motion model 
//             self.predicted_pose,_ = self.motion_model.predict_pose(timestamp, self.f_ref.position, self.f_ref.orientation)            
//             f_cur.update_pose(self.predicted_pose)
//         else:
//             print('setting f_cur.pose <-- f_ref.pose')
//             # use reference frame pose as initial guess 
//             f_cur.update_pose(f_ref.pose)

            // println!(" Tcw: ");
            // let Tcr = f_ref.pose.Tcw.clone();
            // for i in 0..Tcr.rows()
            // {
            //     for j in 0..Tcr.cols()
            //     {
            //         print!("\t {:?}", Tcr.at_2d::<f32>(i, j));
            //     }
            // }
            // println!("\t");

            //f_cur.update_pose(&f_ref.pose._pose);

//         # track camera motion from f_ref to f_cur
//         self.track_previous_frame(f_ref, f_cur)

            self.track_previous_frame(&f_ref, &mut f_cur);

            
            self.map.add_frame(&f_cur);
            self.f_cur.kf_ref = self.kf_ref.id;
            
//         if not self.pose_is_ok:
//             # if previous track didn't go well then track the camera motion from kf_ref to f_cur 
//             self.track_keyframe(self.kf_ref, f_cur) 
                                        
//         # now, having a better estimate of f_cur pose, we can find more map point matches: 
//         # find matches between {local map points} (points in the local map) and {unmatched keypoints of f_cur}
//         if self.pose_is_ok: 
//             self.track_local_map(f_cur)
            
//     # end block {with self.map.update_lock:}
    
//     # TODO: add relocalization 

//     # HACK: since local mapping is not fast enough in python (and tracking is not in real-time) => give local mapping more time to process stuff  
//     self.wait_for_local_mapping()  # N.B.: this must be outside the `with self.map.update_lock:` block

//     with self.map.update_lock:
        
//         # update slam state 
//         if self.pose_is_ok:
//             self.state=SlamState.OK          
//         else:                
//             self.state=SlamState.LOST
//             Printer.red('tracking failure')
            
//         # update motion model state     
//         self.motion_model.is_ok = self.pose_is_ok                    
                            
//         if self.pose_is_ok:   # if tracking was successful
            
//             # update motion model                     
//             self.motion_model.update_pose(timestamp, f_cur.position, f_cur.quaternion)  
                                                                
//             f_cur.clean_vo_map_points()
                    
//             # do we need a new KeyFrame? 
//             need_new_kf = self.need_new_keyframe(f_cur)
                                
//             if need_new_kf: 
//                 Printer.green('adding new KF with frame id % d: ' %(f_cur.id))
//                 if kLogKFinfoToFile:
//                     self.kf_info_logger.info('adding new KF with frame id % d: ' %(f_cur.id))                
//                 kf_new = KeyFrame(f_cur, img)                                     
//                 self.kf_last = kf_new  
//                 self.kf_ref = kf_new 
//                 f_cur.kf_ref = kf_new                  
                
//                 self.local_mapping.push_keyframe(kf_new) 
//                 if not kLocalMappingOnSeparateThread:
//                     self.local_mapping.do_local_mapping()                                      
//             else: 
//                 Printer.yellow('NOT KF')      
                
//             # From ORBSLAM2: 
//             # Clean outliers once keyframe generation has been managed:
//             # we allow points with high innovation (considered outliers by the Huber Function)
//             # pass to the new keyframe, so that bundle adjustment will finally decide
//             # if they are outliers or not. We don't want next frame to estimate its position
//             # with those points so we discard them in the frame.                
//             f_cur.clean_outlier_map_points()                    
                            
//         if self.f_cur.kf_ref is None:
//             self.f_cur.kf_ref = self.kf_ref  
                                
//         self.update_tracking_history()    # must stay after having updated slam state (self.state)                                                                  
                
//         Printer.green("map: %d points, %d keyframes" % (self.map.num_points(), self.map.num_keyframes()))

            self.update_history()
        
//         self.timer_main_track.refresh()
        
//         duration = time.time() - time_start
//         print('tracking duration: ', duration)             

    }



        
    // # track camera motion of f_cur w.r.t. f_ref 
    // def track_previous_frame(self, f_ref, f_cur):  
    pub fn track_previous_frame(&mut self, f_ref: &Frame, f_cur: &mut Frame)     
    {
    //     print('>>>> tracking previous frame ...')        
    //     is_search_frame_by_projection_failure = False 
    //     use_search_frame_by_projection = self.motion_model.is_ok and kUseSearchFrameByProjection and kUseMotionModel
        //println!(">>>> tracking previous frame ...");
        let mut is_search_frame_by_projection_failure = false;


        let use_search_frame_by_projection = false;

        if use_search_frame_by_projection
        {
            print!("NEED to Implement");
        }
        else {
            self.track_reference_frame(f_ref, f_cur, &String::from("_"));
        }
    }     
    //     print('>>>> tracking previous frame ...')        
    //     is_search_frame_by_projection_failure = False 
    //     use_search_frame_by_projection = self.motion_model.is_ok and kUseSearchFrameByProjection and kUseMotionModel
        
    //     if use_search_frame_by_projection: 
    //         # search frame by projection: match map points observed in f_ref with keypoints of f_cur
    //         print('search frame by projection') 
    //         search_radius = Parameters.kMaxReprojectionDistanceFrame          
    //         f_cur.reset_points()               
    //         self.timer_seach_frame_proj.start()
    //         idxs_ref, idxs_cur, num_found_map_pts = search_frame_by_projection(f_ref, f_cur,
    //                                                                          max_reproj_distance=search_radius,
    //                                                                          max_descriptor_distance=self.descriptor_distance_sigma)
    //         self.timer_seach_frame_proj.refresh()  
    //         self.num_matched_kps = len(idxs_cur)    
    //         print("# matched map points in prev frame: %d " % self.num_matched_kps)
                                    
    //         # if not enough map point matches consider a larger search radius 
    //         if self.num_matched_kps < Parameters.kMinNumMatchedFeaturesSearchFrameByProjection:
    //             f_cur.remove_frame_views(idxs_cur)
    //             f_cur.reset_points()   
    //             idxs_ref, idxs_cur, num_found_map_pts = search_frame_by_projection(f_ref, f_cur,
    //                                                                              max_reproj_distance=2*search_radius,
    //                                                                              max_descriptor_distance=0.5*self.descriptor_distance_sigma)
    //             self.num_matched_kps = len(idxs_cur)    
    //             Printer.orange("# matched map points in prev frame (wider search): %d " % self.num_matched_kps)    
                                                
    //         if kDebugDrawMatches and True: 
    //             img_matches = draw_feature_matches(f_ref.img, f_cur.img, 
    //                                                f_ref.kps[idxs_ref], f_cur.kps[idxs_cur], 
    //                                                f_ref.sizes[idxs_ref], f_cur.sizes[idxs_cur],
    //                                                 horizontal=False)
    //             cv2.imshow('tracking frame by projection - matches', img_matches)
    //             cv2.waitKey(1)                
                        
    //         if self.num_matched_kps < Parameters.kMinNumMatchedFeaturesSearchFrameByProjection:
    //             f_cur.remove_frame_views(idxs_cur)
    //             f_cur.reset_points()                   
    //             is_search_frame_by_projection_failure = True                   
    //             Printer.red('Not enough matches in search frame by projection: ', self.num_matched_kps)
    //         else:   
    //             # search frame by projection was successful 
    //             if kUseDynamicDesDistanceTh: 
    //                 self.descriptor_distance_sigma = self.dyn_config.update_descriptor_stat(f_ref, f_cur, idxs_ref, idxs_cur)         
                              
    //             # store tracking info (for possible reuse)                                                    
    //             self.idxs_ref = idxs_ref 
    //             self.idxs_cur = idxs_cur 
                                         
    //             # f_cur pose optimization 1:  
    //             # here, we use f_cur pose as first guess and exploit the matched map point of f_ref 
    //             self.pose_optimization(f_cur,'proj-frame-frame')
    //              # update matched map points; discard outliers detected in last pose optimization 
    //             num_matched_points = f_cur.clean_outlier_map_points()   
    //             print('     # num_matched_map_points: %d' % (self.num_matched_map_points) )
    //             #print('     # matched points: %d' % (num_matched_points) )
                                      
    //             if not self.pose_is_ok or self.num_matched_map_points < kNumMinInliersPoseOptimizationTrackFrame:
    //                 Printer.red('failure in tracking previous frame, # matched map points: ', self.num_matched_map_points)                    
    //                 self.pose_is_ok = False                                                                                                   
        
    //     if not use_search_frame_by_projection or is_search_frame_by_projection_failure:
    //         self.track_reference_frame(f_ref, f_cur,'match-frame-frame')                        
   
    
    // # track camera motion of f_cur w.r.t. f_ref
    // # estimate motion by matching keypoint descriptors                    
    // def track_reference_frame(self, f_ref, f_cur, name=''):
    pub fn track_reference_frame(&mut self, f_ref: &Frame, f_cur: &mut Frame, name: &String)
    {
    //     print('>>>> tracking reference %d ...' %(f_ref.id))        
    //     if f_ref is None:
    //         return 
        //println!(">>>> tracking reference {:?} ...", f_ref.id);

        if f_ref.id==-1
        {
            return
        }
    //     # find keypoint matches between f_cur and kf_ref   
    //     print('matching keypoints with ', Frame.feature_matcher.type.name)              
    //     self.timer_match.start()
    //     idxs_cur, idxs_ref = match_frames(f_cur, f_ref) 
    //     self.timer_match.refresh()          
    //     self.num_matched_kps = idxs_cur.shape[0]    
    //     print("# keypoints matched: %d " % self.num_matched_kps)  
    //     if kUseEssentialMatrixFitting: 
    //         # estimate camera orientation and inlier matches by fitting and essential matrix (see the limitations above)             
    //         idxs_ref, idxs_cur = self.estimate_pose_by_fitting_ess_mat(f_ref, f_cur, idxs_ref, idxs_cur)      
        
        let (mut idxs_cur,mut idxs_ref) = match_frames(f_cur, f_ref, kFeatureMatchRatioTestInitializer);

        self.num_matched_kps = idxs_cur.len() as i32;  
        //println!("# keypoints matched: {:?} " , self.num_matched_kps);

        let (idxs_ref, idxs_cur) = self.estimate_pose_by_fitting_ess_mat(f_ref, f_cur, &idxs_ref, &idxs_cur);



    //     if kUseDynamicDesDistanceTh: 
    //         self.descriptor_distance_sigma = self.dyn_config.update_descriptor_stat(f_ref, f_cur, idxs_ref, idxs_cur)        
                               
    //     # propagate map point matches from kf_ref to f_cur  (do not override idxs_ref, idxs_cur)
    //     num_found_map_pts_inter_frame, idx_ref_prop, idx_cur_prop = propagate_map_point_matches(f_ref, f_cur, idxs_ref, idxs_cur, 
    //                                                                                             max_descriptor_distance=self.descriptor_distance_sigma) 
    //     print("# matched map points in prev frame: %d " % num_found_map_pts_inter_frame)      

        
               
    //     if kDebugDrawMatches and True: 
    //         img_matches = draw_feature_matches(f_ref.img, f_cur.img, 
    //                                            f_ref.kps[idx_ref_prop], f_cur.kps[idx_cur_prop], 
    //                                            f_ref.sizes[idx_ref_prop], f_cur.sizes[idx_cur_prop],
    //                                            horizontal=False)
    //         cv2.imshow('tracking frame (no projection) - matches', img_matches)
    //         cv2.waitKey(1)      
                                
    //     # store tracking info (for possible reuse)              
    //     self.idxs_ref = idxs_ref 
    //     self.idxs_cur = idxs_cur   


        self.idxs_ref = idxs_ref;
        self.idxs_cur = idxs_cur;

    }


 
                                    
    //     # f_cur pose optimization using last matches with kf_ref:  
    //     # here, we use first guess of f_cur pose and propated map point matches from f_ref (matched keypoints) 
    //     self.pose_optimization(f_cur, name)  
    //     # update matched map points; discard outliers detected in last pose optimization 
    //     num_matched_points = f_cur.clean_outlier_map_points()   
    //     print('      # num_matched_map_points: %d' % (self.num_matched_map_points) ) 
    //     #print('     # matched points: %d' % (num_matched_points) )               
    //     if not self.pose_is_ok or self.num_matched_map_points < kNumMinInliersPoseOptimizationTrackFrame:
    //         f_cur.remove_frame_views(idxs_cur)
    //         f_cur.reset_points()               
    //         Printer.red('failure in tracking reference %d, # matched map points: %d' %(f_ref.id,self.num_matched_map_points))  
    //         self.pose_is_ok = False            
        
   
    



    // # estimate a pose from a fitted essential mat; 
    // # since we do not have an interframe translation scale, this fitting can be used to detect outliers, estimate interframe orientation and translation direction 
    // # N.B. read the NBs of the method estimate_pose_ess_mat(), where the limitations of this method are explained  
    // def estimate_pose_by_fitting_ess_mat(self, f_ref, f_cur, idxs_ref, idxs_cur): 
    pub fn estimate_pose_by_fitting_ess_mat(&mut self, f_ref: &Frame, f_cur: &mut Frame, idxs_ref: &VectorOfi32, idxs_cur: &VectorOfi32) -> (VectorOfi32, VectorOfi32)
    {
    //     # N.B.: in order to understand the limitations of fitting an essential mat, read the comments of the method self.estimate_pose_ess_mat() 
    //     self.timer_pose_est.start()
    //     # estimate inter frame camera motion by using found keypoint matches 
    //     # output of the following function is:  Trc = [Rrc, trc] with ||trc||=1  where c=cur, r=ref  and  pr = Trc * pc 
    //     Mrc, self.mask_match = estimate_pose_ess_mat(f_ref.kpsn[idxs_ref], f_cur.kpsn[idxs_cur], 
    //                                                  method=cv2.RANSAC, prob=kRansacProb, threshold=kRansacThresholdNormalized)  


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

            //println!("NEW POSE ESTIMATION,...");
            let (mut Mrc, mask_match) = estimate_pose_ess_mat(&curr_features, &prev_features);
            //println!("NEW POSE ESTIMATION done ...");
            self.mask_match = mask_match;

    //     #Mcr = np.linalg.inv(poseRt(Mrc[:3, :3], Mrc[:3, 3]))   
    //     Mcr = inv_T(Mrc)
    //     estimated_Tcw = np.dot(Mcr, f_ref.pose)
    //     self.timer_pose_est.refresh()      
            let Mcr = inv_T(&Mrc);
            let estimated_Tcw = Poseg2o::new(&(Mcr._pose * f_ref.pose._pose._pose));
            //let estimated_Tcw = Mcr.mul(&f_ref.pose.Tcw, 1.0).unwrap().to_mat().unwrap();


            /* 
            println!("estimate_pose_by_fitting_ess_mat\n\n\n\n\n");
            println!("{:?}", Mrc._pose.to_matrix());
            println!("{:?}", Mcr._pose.to_matrix());
            println!("{:?}", f_ref.pose._pose._pose.to_matrix());
            println!("{:?}", estimated_Tcw._pose.to_matrix());
            println!("estimate_pose_by_fitting_ess_mat\n\n\n\n\n");
            panic!("Done"); 
            */

    //     # remove outliers from keypoint matches by using the mask computed with inter frame pose estimation        
    //     mask_idxs = (self.mask_match.ravel() == 1)
    //     self.num_inliers = sum(mask_idxs)
    //     print('# inliers: ', self.num_inliers )
    //     idxs_ref = idxs_ref[mask_idxs]
    //     idxs_cur = idxs_cur[mask_idxs]
    let mut idx_cur_inliers= VectorOfi32::new();
    let mut idx_ref_inliers= VectorOfi32::new();
    
    self.num_inliers = 0;
    for i in 0..self.mask_match.len()
    {
        if self.mask_match.get(i).unwrap()==1
        {
            idx_cur_inliers.push(idxs_cur.get(i as usize).unwrap());
            idx_ref_inliers.push(idxs_ref.get(i as usize).unwrap());
            self.num_inliers+=1;
        }
        
    }
    
    //     # if there are not enough inliers do not use the estimated pose 
    //     if self.num_inliers < kNumMinInliersEssentialMat:
    //         #f_cur.update_pose(f_ref.pose) # reset estimated pose to previous frame 
    //         Printer.red('Essential mat: not enough inliers!')  
    //     else:
    //         # use the estimated pose as an initial guess for the subsequent pose optimization 
    //         # set only the estimated rotation (essential mat computation does not provide a scale for the translation, see above) 
    //         #f_cur.pose[:3,:3] = estimated_Tcw[:3,:3] # copy only the rotation 
    //         #f_cur.pose[:,3] = f_ref.pose[:,3].copy() # override translation with ref frame translation 
    //         Rcw = estimated_Tcw[:3,:3] # copy only the rotation 
    //         tcw = f_ref.pose[:3,3]     # override translation with ref frame translation          
    //         f_cur.update_rotation_and_translation(Rcw, tcw)     
    //     return  idxs_ref, idxs_cur

        if self.num_inliers< kNumMinInliersEssentialMat
        {
            println!("Not enough inliers ....: {:?}", self.num_inliers);
        }

        //println!("inliers ....: {:?}", self.num_inliers);
        //let Rcw = estimated_Tcw.Rcw().clone();//get_rotation_from_transformation(&estimated_Tcw);
        //let tcw = f_ref.pose.tcw.clone();

        //println!(" Rotation Rcw \n {:?}  \n {:?}", Mcr._pose.to_matrix() , f_ref.pose._pose._pose.to_matrix());
        //f_cur.update_rotation_and_translation(&Rcw, &tcw);

        let mut Tcw  = Poseg2o::default();
        Tcw._pose.rotation = estimated_Tcw._pose.rotation.clone();
        Tcw._pose.translation = f_ref.pose._pose._pose.translation.clone();

        f_cur.update_pose(&Tcw);

        (idx_ref_inliers, idx_cur_inliers)


    }
 




    // # def update_history(self):
    // #     f_cur = self.map.get_frame(-1)
    // #     self.cur_R = f_cur.pose[:3,:3].T
    // #     self.cur_t = np.dot(-self.cur_R,f_cur.pose[:3,3])
    // #     if (self.init_history is True) and (self.trueX is not None):
    // #         self.t0_est = np.array([self.cur_t[0], self.cur_t[1], self.cur_t[2]])  # starting translation 
    // #         self.t0_gt  = np.array([self.trueX, self.trueY, self.trueZ])           # starting translation 
    // #     if (self.t0_est is not None) and (self.t0_gt is not None):             
    // #         p = [self.cur_t[0]-self.t0_est[0], self.cur_t[1]-self.t0_est[1], self.cur_t[2]-self.t0_est[2]]   # the estimated traj starts at 0
    // #         self.traj3d_est.append(p)
    // #         self.traj3d_gt.append([self.trueX-self.t0_gt[0], self.trueY-self.t0_gt[1], self.trueZ-self.t0_gt[2]])            
    // #         self.poses.append(poseRt(self.cur_R, p)) 
    
    pub fn update_history(&mut self)
    {
        let f_cur = self.map.get_last_frame();

        /* 
        println!("Frameid {:?}", f_cur.id);
        println!("Rotation {:?}", f_cur.pose._pose._pose.rotation);
        println!("Translation {:?}", f_cur.pose._pose._pose.translation);
        */

        self.cur_R = f_cur.pose.Rcw.t().unwrap().to_mat().unwrap();

        let cur_R_minus = opencv::core::mul_f64_mat(-1.0, &self.cur_R).unwrap();

        let cur_t =  opencv::core::mul_matexpr_mat(&cur_R_minus, &f_cur.pose.tcw).unwrap().to_mat().unwrap();
        self.cur_t = (self.cur_t.clone()+ cur_t).into_result().unwrap().to_mat().unwrap();      

    }

}
