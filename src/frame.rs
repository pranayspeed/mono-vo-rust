





use opencv::types::*;
use opencv::core::*;


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


pub struct Frame {
    img: Mat,
    is_keyframe: bool,
    kps: VectorOfPoint2f,
    kpsu: VectorOfPoint2f,
    kpsn: VectorOfPoint2f,
    octaves: VectorOfi32,
    sizes: VectorOfi32,
    angles: VectorOff32,
    des: VectorOff32,
    points: VectorOfPoint3d,
    outliers: VectorOfi32,
    kf_ref: i32,

  }

  impl Frame {
    //Construct person
    pub fn new(image: &Mat) -> Frame {
        Frame {
            img: image.clone(),
            is_keyframe: false,
            kps: VectorOfPoint2f::new(),
            kpsu: VectorOfPoint2f::new(),
            kpsn: VectorOfPoint2f::new(),
            octaves: VectorOfi32::new(),
            sizes: VectorOfi32::new(),
            angles: VectorOff32::new(),
            des: VectorOff32::new(),
            points: VectorOfPoint3d::new(),
            outliers: VectorOfi32::new(),
            kf_ref: -1,
      }
    }
}


