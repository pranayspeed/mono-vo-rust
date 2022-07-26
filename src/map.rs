




// class Map(object):
// def __init__(self):
//     self._lock = RLock()   
//     self._update_lock = RLock()  
    
//     self.frames = deque(maxlen=kMaxLenFrameDeque)  # deque with max length, it is thread-safe 
//     self.keyframes = OrderedSet()  
//     self.points = set()
    
//     self.max_frame_id = 0     # 0 is the first frame id
//     self.max_point_id = 0     # 0 is the first point id
//     self.max_keyframe_id = 0  # 0 is the first keyframe id

//     # local map 
//     #self.local_map = LocalWindowMap(map=self)
//     self.local_map = LocalCovisibilityMap(map=self)

use opencv::core::Point3f;

use crate::frame::Frame;


#[derive(Debug, Default, Clone)]
pub struct Map {
    pub keyframes : Vec<Frame>,
    pub frames: Vec<Frame>,
    pub points: Vec<Point3f>,
    pub max_frame_id: i32,
    pub max_point_id: i32,
    pub max_keyframe_id: i32,
}

impl Map {

//Construct Map
pub fn default() -> Map {
    Map {
        keyframes : Vec::new(),
        frames: Vec::new(),
        points: Vec::new(),
        max_frame_id: 0,
        max_point_id: 0,
        max_keyframe_id: 0,
  }
}

    pub fn add_frame(&mut self, frame: &Frame)
    {
        //println!("ROTATION MATRIX : {:?}    {:?}",self.max_frame_id , frame.pose.Rcw );
        self.frames.push(frame.clone());
        self.max_frame_id+=1;
    }

    pub fn add_keyframe(&mut self, frame: &Frame)
    {
        self.keyframes.push(frame.clone());
        self.max_keyframe_id+=1;
    }

    pub fn get_frame(&self, index: usize) -> Frame
    {
        self.frames.get(index).unwrap().clone()
    }

    pub fn get_keyframe(&self, index: usize) -> Frame
    {
        self.keyframes.get(index).unwrap().clone()
    }

    pub fn get_last_frame(&self) -> Frame
    {
        //println!("Frame count {:?}", self.max_frame_id);
        self.frames.last().unwrap().clone()
    }
}