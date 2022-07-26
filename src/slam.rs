

use opencv::types::*;
use opencv::core::*;


use crate::feature_tracker::DescriptorFeatureTracker;
use crate::tracking::Tracking;

use crate::camera::Camera;


// # main slam class containing all the required modules 
// class Slam(object):
//     def __init__(self, camera, feature_tracker, groundtruth = None):    
//         self.init_feature_tracker(feature_tracker)
//         self.camera = camera 
//         self.map = Map()
//         self.local_mapping = LocalMapping(self.map)
//         if kLocalMappingOnSeparateThread:
//             self.local_mapping.start()
//         self.groundtruth = groundtruth  # not actually used here; could be used for evaluating performances 
//         self.tracking = Tracking(self)
#[derive(Debug, Default, Clone)]
pub struct Slam {
    pub camera: Camera,
    pub map: bool,
    pub local_mapping: bool,
    pub tracking:   Tracking,
}

impl Slam {

// //Construct Slam
// pub fn default() -> Slam {
//     Slam {
//         camera: Camera::new(1241, 376, 718.856, 718.856, 607.1928, 185.2157, VectorOff32::from(vec![0.0,0.0,0.0,0.0,0.0])),
//         map: false,
//         local_mapping: false,
//         tracking: Tracking::default(),
//   }
// }

pub fn new(camera: &Camera) -> Slam {
    let mut slam = Slam {
        camera: camera.clone(),
        map: false,
        local_mapping: false,
        tracking: Tracking::default(),
  };
  slam.tracking = Tracking::new(&slam);
  slam
}

//     def init_feature_tracker(self, tracker):
//         Frame.set_tracker(tracker) # set the static field of the class 
//         if kUseEssentialMatrixFitting:
//             Printer.orange('forcing feature matcher ratio_test to 0.8')
//             tracker.matcher.ratio_test = 0.8
//         if tracker.tracker_type == FeatureTrackerTypes.LK:
//             raise ValueError("You cannot use Lukas-Kanade tracker in this SLAM approach!")  
        
        
//     # @ main track method @
//     def track(self, img, frame_id, timestamp=None):
//         return self.tracking.track(img,frame_id,timestamp)

    pub fn track(&mut self, img: &Mat, frame_id: i32, timestamp: f32)
    {
        self.tracking.track(img, frame_id, timestamp)
    }
}


        
//     def quit(self):
//         if kLocalMappingOnSeparateThread:
//             self.local_mapping.quit()                       


