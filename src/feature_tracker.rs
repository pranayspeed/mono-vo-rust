
use opencv::prelude::Feature2DTrait;
use opencv::prelude::ORB;
use opencv::types::*;
use opencv::core::*;

use crate::camera::Camera;
use crate::frame::Frame;
use crate::parameters::*;

use crate::slam::Slam;


const kMinNumFeatureDefault:i32 = 2000;
//kLkPyrOpticFlowNumLevelsMin = 3   # maximal pyramid level number for LK optic flow 
const kRatioTest : f32 = kFeatureMatchRatioTestInitializer;


pub trait Tracker
{
    fn detectAndCompute(&self, frame: &Mat, mask : &VectorOfi32 ) -> (VectorOfKeyPoint, Mat);
    fn track(&self, img: &Mat, frame_id: i32, timestamp: f32);
}




// class FeatureTrackingResult(object): 
//     def __init__(self):
//         self.kps_ref = None          # all reference keypoints (numpy array Nx2)
//         self.kps_cur = None          # all current keypoints   (numpy array Nx2)
//         self.des_cur = None          # all current descriptors (numpy array NxD)
//         self.idxs_ref = None         # indexes of matches in kps_ref so that kps_ref_matched = kps_ref[idxs_ref]  (numpy array of indexes)
//         self.idxs_cur = None         # indexes of matches in kps_cur so that kps_cur_matched = kps_cur[idxs_cur]  (numpy array of indexes)
//         self.kps_ref_matched = None  # reference matched keypoints, kps_ref_matched = kps_ref[idxs_ref]
//         self.kps_cur_matched = None  # current matched keypoints, kps_cur_matched = kps_cur[idxs_cur]


// pub struct FeatureTrackingResult {
//         kps_ref: Frame,// = None          # all reference keypoints (numpy array Nx2)
//         kps_cur = None          # all current keypoints   (numpy array Nx2)
//         des_cur = None          # all current descriptors (numpy array NxD)
//         idxs_ref = None         # indexes of matches in kps_ref so that kps_ref_matched = kps_ref[idxs_ref]  (numpy array of indexes)
//         idxs_cur = None         # indexes of matches in kps_cur so that kps_cur_matched = kps_cur[idxs_cur]  (numpy array of indexes)
//         kps_ref_matched = None  # reference matched keypoints, kps_ref_matched = kps_ref[idxs_ref]
//         kps_cur_matched = None  # current matched keypoints, kps_cur_matched = kps_cur[idxs_cur]  
//   }

//   impl FeatureTrackingResult {
    
//     //Construct FeatureTrackingResult
//     pub fn default() -> FeatureTrackingResult {
//         FeatureTrackingResult {

//       }
//     }




// # Extract features by using desired detector and descriptor, match keypoints by using desired matcher on computed descriptors
// class DescriptorFeatureTracker(FeatureTracker): 
//     def __init__(self, num_features=kMinNumFeatureDefault, 
//                        num_levels = 1,                                    # number of pyramid levels for detector  
//                        scale_factor = 1.2,                                # detection scale factor (if it can be set, otherwise it is automatically computed)                
//                        detector_type = FeatureDetectorTypes.FAST, 
//                        descriptor_type = FeatureDescriptorTypes.ORB,
//                        match_ratio_test = kRatioTest, 
//                        tracker_type = FeatureTrackerTypes.DES_FLANN):
//         super().__init__(num_features=num_features, 
//                          num_levels=num_levels, 
//                          scale_factor=scale_factor, 
//                          detector_type=detector_type, 
//                          descriptor_type=descriptor_type, 
//                          match_ratio_test = match_ratio_test,
//                          tracker_type=tracker_type)
//         self.feature_manager = feature_manager_factory(num_features=num_features, 
//                                                        num_levels=num_levels, 
//                                                        scale_factor=scale_factor, 
//                                                        detector_type=detector_type, 
//                                                        descriptor_type=descriptor_type)                     

//         if tracker_type == FeatureTrackerTypes.DES_FLANN:
//             self.matching_algo = FeatureMatcherTypes.FLANN
//         elif tracker_type == FeatureTrackerTypes.DES_BF:
//             self.matching_algo = FeatureMatcherTypes.BF
//         else:
//             raise ValueError("Unmanaged matching algo for feature tracker %s" % self.tracker_type)                   
                    
//         # init matcher 
//         self.matcher = feature_matcher_factory(norm_type=self.norm_type, ratio_test=match_ratio_test, type=self.matching_algo)    

#[derive(Debug, Default, Clone)]
pub struct DescriptorFeatureTracker {
    num_features: i32,
    num_levels : i32, //1,  //                                  # number of pyramid levels for detector  
    scale_factor :f32, // 1.2,   //                             # detection scale factor (if it can be set, otherwise it is automatically computed)                
    detector_type: i32, //= FeatureDetectorTypes.FAST, 
    descriptor_type : i32, //= FeatureDescriptorTypes.ORB,
    match_ratio_test : f32, //= kRatioTest, 
    tracker_type: i32,  // = FeatureTrackerTypes.DES_FLANN   
    camera: Camera,
    f_cur: Frame,
  }

  impl DescriptorFeatureTracker {
    
    //Construct DescriptorFeatureTracker
    pub fn new(system: &Slam) -> DescriptorFeatureTracker {
        DescriptorFeatureTracker {
            num_features: kMinNumFeatureDefault,
            num_levels : 1, //1,  //                                  # number of pyramid levels for detector  
            scale_factor :1.2, // 1.2,   //                             # detection scale factor (if it can be set, otherwise it is automatically computed)                
            detector_type: -1, //= FeatureDetectorTypes.FAST, 
            descriptor_type : -1, //= FeatureDescriptorTypes.ORB,
            match_ratio_test : kRatioTest, //= kRatioTest, 
            tracker_type: -1,  // = FeatureTrackerTypes.DES_FLANN  
            camera: system.camera.clone(),
            f_cur: Frame::default()               
      }
    }



//     # out: keypoints and descriptors 
//     def detectAndCompute(self, frame, mask=None):
//         return self.feature_manager.detectAndCompute(frame, mask) 

    pub fn detectAndCompute(&self, frame: &Mat, mask : &VectorOfi32 ) -> (VectorOfKeyPoint, Mat)
    {   
        
        let mut kp1 = VectorOfKeyPoint::new();
        let mut des1 = Mat::default();

        DescriptorFeatureTracker::orb_extract(frame, &mut kp1, &mut des1);

        (kp1, des1)
    }


    pub fn orb_extract(img1: &Mat, kp1: &mut VectorOfKeyPoint, des1: &mut Mat)
    {
  
          //let img1 = msg.get_frame().grayscale_to_cv_mat();
          let mut orb: PtrOfORB =  ORB::default().unwrap();
          orb.detect_and_compute(&img1,&Mat::default(),  kp1,  des1, false).unwrap();
  
    }



//     # out: FeatureTrackingResult()
//     def track(self, image_ref, image_cur, kps_ref, des_ref):
//         kps_cur, des_cur = self.detectAndCompute(image_cur)
//         # convert from list of keypoints to an array of points 
//         kps_cur = np.array([x.pt for x in kps_cur], dtype=np.float32) 
    
//         idxs_ref, idxs_cur = self.matcher.match(des_ref, des_cur)  #knnMatch(queryDescriptors,trainDescriptors)
//         #print('num matches: ', len(matches))

//         res = FeatureTrackingResult()
//         res.kps_ref = kps_ref  # all the reference keypoints  
//         res.kps_cur = kps_cur  # all the current keypoints       
//         res.des_cur = des_cur  # all the current descriptors         
        
//         res.kps_ref_matched = np.asarray(kps_ref[idxs_ref]) # the matched ref kps  
//         res.idxs_ref = np.asarray(idxs_ref)                  
        
//         res.kps_cur_matched = np.asarray(kps_cur[idxs_cur]) # the matched cur kps  
//         res.idxs_cur = np.asarray(idxs_cur)
        
//         return res       

    // pub fn track(self, image_ref, image_cur, kps_ref, des_ref)
    // {

    // }



// # @ main track method @
// def track(self, img, frame_id, timestamp=None):

}


