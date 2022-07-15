





use opencv::types::*;
use opencv::core::*;

use crate::dvutils::*;

extern crate nalgebra as na;

// class Camera: 
//     def __init__(self, width, height, fx, fy, cx, cy, D, fps = 1): # D = [k1, k2, p1, p2, k3]
//         self.width = width
//         self.height = height
//         self.fx = fx
//         self.fy = fy
//         self.cx = cx
//         self.cy = cy
//         self.D = np.array(D,dtype=np.float32) # np.array([k1, k2, p1, p2, k3])  distortion coefficients 
//         self.fps = fps 
        
//         self.is_distorted = np.linalg.norm(self.D) > 1e-10
//         self.initialized = False    

        
// class PinholeCamera(Camera):
//     def __init__(self, width, height, fx, fy, cx, cy, D, fps = 1):
//         super().__init__(width, height, fx, fy, cx, cy, D, fps)
//         self.K = np.array([[fx, 0,cx],
//                            [ 0,fy,cy],
//                            [ 0, 0, 1]])
//         self.Kinv = np.array([[1/fx,    0,-cx/fx],
//                               [   0, 1/fy,-cy/fy],
//                               [   0,    0,    1]])             
        
//         self.u_min, self.u_max = 0, self.width 
//         self.v_min, self.v_max = 0, self.height       
//         self.init()    
        
//     def init(self):
//         if not self.initialized:
//             self.initialized = True 
//             self.undistort_image_bounds()    


pub struct Camera {
    width: i32,
    height: i32,
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
    D: VectorOff32,
    fps: i32,
    is_distorted: bool,
    initialized:bool,
    K: Mat,
    Kinv : Mat,
    u_min: i32,
    u_max: i32,
    v_min: i32,
    v_max: i32,


  }

  impl Camera {
    //Construct person
    pub fn new(width: i32, height:i32, fx:f32, fy:f32, cx:f32, cy:f32, D:VectorOff32) -> Camera {
        Camera {
            width: width,
            height: height,
            fx: fx,
            fy: fy,
            cx: cx,
            cy: cy,
            D: D,// np.array([k1, k2, p1, p2, k3])  distortion coefficients 
            fps: 1,
            is_distorted: false, //(D.norm()> 1e-10),
            initialized:false,
            K: Mat::default(),
            Kinv : Mat::default(),
            u_min: 0,
            u_max: width,
            v_min: 0,
            v_max: height,
      }
    }

    pub fn init(&mut self) 
    {
        if self.initialized == false{
            self.initialized=true;
            self.is_distorted = opencv::core::norm(&self.D, opencv::core::NORM_L2, &opencv::core::no_array().unwrap()).unwrap()> 1e-10;
            self.undistort_image_bounds();
            self.K = self.get_cv_K(self.fx, self.fy, self.cx, self.cy);
            self.Kinv = self.get_cv_Kinv(self.fx, self.fy, self.cx, self.cy);
        }
    }

    // # update image bounds     
    // def undistort_image_bounds(self):
    //     uv_bounds = np.array([[self.u_min, self.v_min],
    //                             [self.u_min, self.v_max],
    //                             [self.u_max, self.v_min],
    //                             [self.u_max, self.v_max]], dtype=np.float32).reshape(4,2)
    //     #print('uv_bounds: ', uv_bounds)
    //     if self.is_distorted:
    //             uv_bounds_undistorted = cv2.undistortPoints(np.expand_dims(uv_bounds, axis=1), self.K, self.D, None, self.K)      
    //             uv_bounds_undistorted = uv_bounds_undistorted.ravel().reshape(uv_bounds_undistorted.shape[0], 2)
    //     else:
    //         uv_bounds_undistorted = uv_bounds 
    //     #print('uv_bounds_undistorted: ', uv_bounds_undistorted)                
    //     self.u_min = min(uv_bounds_undistorted[0][0],uv_bounds_undistorted[1][0])
    //     self.u_max = max(uv_bounds_undistorted[2][0],uv_bounds_undistorted[3][0])        
    //     self.v_min = min(uv_bounds_undistorted[0][1],uv_bounds_undistorted[2][1])    
    //     self.v_max = max(uv_bounds_undistorted[1][1],uv_bounds_undistorted[3][1])  
    //     # print('camera u_min: ', self.u_min)
    //     # print('camera u_max: ', self.u_max)
    //     # print('camera v_min: ', self.v_min)         
    //     # print('camera v_max: ', self.v_max)   
    pub fn undistort_image_bounds(&mut self)
    {
        let mut uv_bounds= VectorOfPoint2f::new();
        uv_bounds.push(Point2f::new(self.u_min as f32, self.v_min as f32));
        uv_bounds.push(Point2f::new(self.u_min as f32, self.v_max as f32));
        uv_bounds.push(Point2f::new(self.u_max as f32, self.v_min as f32));
        uv_bounds.push(Point2f::new(self.u_max as f32, self.v_max as f32));

        let mut uv_bounds_undistorted = uv_bounds.clone();
        if self.is_distorted
        {  
            //uv_bounds_undistorted = cv2.undistortPoints(np.expand_dims(uv_bounds, axis=1), self.K, self.D, None, self.K)
            
            opencv::calib3d::undistort_points(&uv_bounds,&mut uv_bounds_undistorted, &self.K, &self.D, &Mat::eye(3, 3, opencv::core::CV_32F).unwrap(),&self.K).unwrap();
            //uv_bounds_undistorted = uv_bounds_undistorted.ravel().reshape(uv_bounds_undistorted.shape[0], 2)
        }
        println!("uv_bounds_undistorted: {:?}", uv_bounds_undistorted);

        self.u_min = std::cmp::min(uv_bounds_undistorted.get(0).unwrap().x as i32 ,uv_bounds_undistorted.get(1).unwrap().x as i32);
        self.u_max = std::cmp::max(uv_bounds_undistorted.get(2).unwrap().x as i32 ,uv_bounds_undistorted.get(3).unwrap().x as i32);      
        self.v_min = std::cmp::min(uv_bounds_undistorted.get(0).unwrap().y as i32 ,uv_bounds_undistorted.get(2).unwrap().y as i32);    
        self.v_max = std::cmp::max(uv_bounds_undistorted.get(1).unwrap().y as i32 ,uv_bounds_undistorted.get(3).unwrap().y as i32);  
        println!("camera u_min: {:?}", self.u_min);
        println!("camera u_max: {:?}", self.u_max);
        println!("camera v_min: {:?}", self.v_min);         
        println!("camera v_max: {:?}", self.v_max);   

    }

    pub fn get_cv_K(&self, fx:f32, fy:f32, cx:f32, cy:f32) -> Mat
    {
        // form calibration matrix
        let mut K = Mat::new_rows_cols_with_default(3, 3 ,CV_32FC1, opencv::core::Scalar::all(0.0)).unwrap();
        // Mat K = (Mat_<double>(3,3) << 718.8560, 0, 607.1928, 0, 718.8560, 185.2157, 0, 0, 1 );
        unsafe {
            *K.at_2d_unchecked_mut::<f32>(0,0).unwrap() = fx;
            *K.at_2d_unchecked_mut::<f32>(0,1).unwrap() = 0.0;
            *K.at_2d_unchecked_mut::<f32>(0,2).unwrap() = cx;
            *K.at_2d_unchecked_mut::<f32>(1,0).unwrap() = 0.0;
            *K.at_2d_unchecked_mut::<f32>(1,1).unwrap() = fy;
            *K.at_2d_unchecked_mut::<f32>(1,2).unwrap() = cy;
            *K.at_2d_unchecked_mut::<f32>(2,0).unwrap() = 0.0;
            *K.at_2d_unchecked_mut::<f32>(2,1).unwrap() = 0.0;
            *K.at_2d_unchecked_mut::<f32>(2,2).unwrap() = 1.0;
        }

        K
    }
    pub fn get_cv_Kinv(&self, fx:f32, fy:f32, cx:f32, cy:f32) -> Mat
    {

        //DVMatrix3::new(1.0/fx,    0.0,-cx/fx, 0.0, 1.0/fy,-cy/fy, 0.0,    0.0,    1.0)
        // form calibration matrix
        let mut K = Mat::new_rows_cols_with_default(3, 3 ,CV_32FC1, opencv::core::Scalar::all(0.0)).unwrap();
        // Mat K = (Mat_<double>(3,3) << 718.8560, 0, 607.1928, 0, 718.8560, 185.2157, 0, 0, 1 );
        unsafe {
            *K.at_2d_unchecked_mut::<f32>(0,0).unwrap() = 1.0/fx;
            *K.at_2d_unchecked_mut::<f32>(0,1).unwrap() = 0.0;
            *K.at_2d_unchecked_mut::<f32>(0,2).unwrap() = -cx/fx;
            *K.at_2d_unchecked_mut::<f32>(1,0).unwrap() = 0.0;
            *K.at_2d_unchecked_mut::<f32>(1,1).unwrap() = 1.0/fy;
            *K.at_2d_unchecked_mut::<f32>(1,2).unwrap() = -cy/fy;
            *K.at_2d_unchecked_mut::<f32>(2,0).unwrap() = 0.0;
            *K.at_2d_unchecked_mut::<f32>(2,1).unwrap() = 0.0;
            *K.at_2d_unchecked_mut::<f32>(2,2).unwrap() = 1.0;
        }

        K
    }


    // # project a 3D point or an array of 3D points (w.r.t. camera frame), of shape [Nx3]
    // # out: Nx2 image points, [Nx1] array of map point depths     
    // def project(self, xcs):
    //     #u = self.fx * xc[0]/xc[2] + self.cx
    //     #v = self.fy * xc[1]/xc[2] + self.cy  
    //     projs = self.K @ xcs.T     
    //     zs = projs[-1]      
    //     projs = projs[:2]/ zs   
    //     return projs.T, zs
        
    pub fn project(&self, xcs: &VectorOfPoint3f) -> (VectorOfPoint2f, VectorOff32)
    {
        //#u = self.fx * xc[0]/xc[2] + self.cx
        //#v = self.fy * xc[1]/xc[2] + self.cy  
        let mut projs = VectorOfPoint3f::new();
        opencv::core::transform(&xcs, &mut projs, &self.K).unwrap();    
        
        let mut h_points = VectorOfPoint2f::new();
        opencv::calib3d::convert_points_from_homogeneous(&projs, &mut h_points).unwrap();

        //let zs = projs[-1];      
        //projs = projs[:2]/ zs;  
        let mut zs = VectorOff32::new();
        for pt in projs{
            zs.push(pt.z);
        }
 
        return (h_points , zs);
    }

    // # unproject 2D point uv (pixels on image plane) on 
    // def unproject(self, uv):
    //     x = (uv[0] - self.cx)/self.fx
    //     y = (uv[1] - self.cy)/self.fy
    //     return x,y
    pub fn unproject(&self, uv: &Point2f) -> (f32, f32)
    {
        let x = (uv.x - self.cx)/self.fx;
        let y = (uv.y - self.cy)/self.fy;
        return (x,y);        
    }


    // // # in:  uvs [Nx2]
    // // # out: xcs array [Nx3] of normalized coordinates     
    // // def unproject_points(self, uvs):
    // //     return np.dot(self.Kinv, add_ones(uvs).T).T[:, 0:2]  
    
    
    // pub fn unproject_points(&self, uvs : &VectorOfPoint2f) -> VectorOfPoint3f
    // {
    //     let unproj_pts = VectorOfPoint3f::new();
        
    //     for pt in uvs{
    //         unproj_pts.push(Point3f::new(pt.x, pt.y, 1.0));
    //     }

    //     unproj_pts
    // }



    // # in:  uvs [Nx2]
    // # out: uvs_undistorted array [Nx2] of undistorted coordinates  
    // def undistort_points(self, uvs):
    //     if self.is_distorted:
    //         #uvs_undistorted = cv2.undistortPoints(np.expand_dims(uvs, axis=1), self.K, self.D, None, self.K)   # =>  Error: while undistorting the points error: (-215:Assertion failed) src.isContinuous() 
    //         uvs_contiguous = np.ascontiguousarray(uvs[:, :2]).reshape((uvs.shape[0], 1, 2))
    //         uvs_undistorted = cv2.undistortPoints(uvs_contiguous, self.K, self.D, None, self.K)            
    //         return uvs_undistorted.ravel().reshape(uvs_undistorted.shape[0], 2)
    //     else:
    //         return uvs 

    pub fn undistort_points(&self, uvs: &VectorOfPoint2f)-> VectorOfPoint2f
    {
        if self.is_distorted
        {
            //uvs_contiguous = np.ascontiguousarray(uvs[:, :2]).reshape((uvs.shape[0], 1, 2))
//         uvs_undistorted = cv2.undistortPoints(uvs_contiguous, self.K, self.D, None, self.K)            
//         return uvs_undistorted.ravel().reshape(uvs_undistorted.shape[0], 2)
            let mut uv_bounds_undistorted = VectorOfPoint2f::new();
             opencv::calib3d::undistort_points(&uvs,&mut uv_bounds_undistorted, &self.K, &self.D, &Mat::eye(3, 3, opencv::core::CV_32F).unwrap(),&self.K).unwrap();

            return uv_bounds_undistorted;
        }
        else 
        {
            return uvs.clone();
        }

    }
}


