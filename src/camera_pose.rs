


//use opencv::types::*;
use opencv::core::*;


use nalgebra::{Isometry3, IsometryMatrix3, Point3, Vector3, Translation3, Rotation3, Quaternion, AbstractRotation};
use std::f32;
use nalgebra::geometry::UnitQuaternion;

use cv_convert::{FromCv, IntoCv, TryFromCv, TryIntoCv};

use nalgebra::Matrix3;

use crate::utils_geom::get_rotation_from_transformation;
/* 
// // Temperory pose defenition for g2o
// pub struct Poseg2o {
//     quat: Vec4f,
//     pos: Vec3f,
//   }

//   impl Poseg2o {
//     //Construct Poseg2o
//     pub fn default() -> Poseg2o {
//         Poseg2o {
//             quat: Vec4f::new(0.0,0.0,0.0,0.0),
//             pos: Vec3f::all(0.0),
//       }
//     }
//     pub fn new( quat: &Vec4f, pos: &Vec3f) -> Poseg2o {
//         Poseg2o {
//             quat: quat.clone(),
//             pos: pos.clone(),
//       }
//     }

//     pub fn position(&self) -> Vec3f
//     {
//         self.pos.clone()
//     }
//     pub fn orientation(&self) -> Vec4f
//     {
//         self.quat.clone()
//     }

//     pub fn matrix(&self) -> Mat
//     {
//         let matrix = Mat::default();

//         matrix
//     }
// }

*/


#[derive(Debug, Default, Clone)]
pub struct Poseg2o {
    pub _pose: Isometry3<f32>,
  }

  impl Poseg2o {
    //Construct Poseg2o
    pub fn default() -> Poseg2o {
        Poseg2o {
            _pose:  Isometry3::<f32>::identity(), //Isometry3::<f32>::new(Vector3::new(0.0,0.0,0.0), Vector3::y()),
      }
    }
    pub fn new_from_trans_axis( trans: &Vector3<f32>, axisangle: &Vector3<f32>) -> Poseg2o {
        Poseg2o {
            _pose: Isometry3::<f32>::new(*trans, *axisangle),
      }
    }

    pub fn new_old( pose: &Mat) -> Poseg2o {
        Poseg2o {
            _pose: Poseg2o::create_pose_from_mat(pose),
      }
    }
    pub fn new( pose: &Isometry3<f32>) -> Poseg2o {
        Poseg2o {
            _pose: pose.clone(),
      }
    }

    pub fn create_pose_from_mat(pose: &Mat) -> Isometry3<f32>
    {   
        let mut curr_pose = Isometry3::<f32>::identity();   
        //println!("{:?}", pose);     
        let pose_na = nalgebra::Matrix4::<f32>::try_from_cv(pose).unwrap();
        // Updated rotation
        
        let rot_mat = nalgebra::Matrix3::<f32>::try_from_cv(&get_rotation_from_transformation(pose)).unwrap();
        //let rot_mat: Matrix3<f32> = *pose_na.slice((0,0), (3,3)).fixed_slice::<3,3>(0, 0);
        curr_pose.rotation = nalgebra::UnitQuaternion::<f32>::from_rotation_matrix(&Rotation3::<f32>::from_matrix_unchecked(rot_mat));
        let x = pose_na.get((0,3)).unwrap();
        let y = pose_na.get((1,3)).unwrap();
        let z = pose_na.get((2,3)).unwrap();

        // Updated translation
        curr_pose.translation = nalgebra::Translation3::<f32>::new(*x,*y,*z);


        curr_pose
    }

    pub fn update(&mut self, pose: &Mat)
    {        
        let pose_na = nalgebra::Matrix3::<f32>::try_from_cv(pose).unwrap();
        // Updated rotation
        self._pose.rotation = nalgebra::UnitQuaternion::<f32>::from_rotation_matrix(&Rotation3::<f32>::from_matrix_unchecked(pose_na));
        let x = pose_na.get((0,3)).unwrap();
        let y = pose_na.get((1,3)).unwrap();
        let z = pose_na.get((2,3)).unwrap();

        // Updated translation
        self._pose.translation = nalgebra::Translation3::<f32>::new(*x,*y,*z);

    }

    pub fn tcw(&self) -> Mat
    {
        self._pose.translation.try_into_cv().unwrap()
    }
    // pub fn orientation(&self) -> Quaternion<f32>
    // {
    //     let q1 = UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_4, 0.0, 0.0);
    //     self._pose.transform_vector(v)
    // }

    pub fn Rcw(&self) -> Mat
    {
        self._pose.rotation.to_rotation_matrix().matrix().try_into_cv().unwrap()
    }
    

    pub fn Rwc(&self) -> Mat
    {
        self.Rcw().t().unwrap().to_mat().unwrap()
    }

    pub fn Tcw(&self) -> Mat
    {
        self._pose.to_matrix().try_into_cv().unwrap()
    }

    pub fn Ow(&self) -> Mat
    {
        //-(self.Rwc @ self.tcw)  # origin of camera frame w.r.t world
        //#NOTE: Might be a possilbe bug - TO_CHECK
        (self._pose.rotation.to_rotation_matrix().transpose().to_homogeneous() * self._pose.translation.to_homogeneous()).scale(-1.0).try_into_cv().unwrap()
        //Mat::default()
        //(-1.0* self._pose.rotation.to_rotation_matrix().transpose(). * self._pose.translation).try_into_cv().unwrap()
    }
}


// # camera pose representation by using g2o.Isometry3d()
// class CameraPose(object):
//     def __init__(self, pose=None):
//         #self._pose = None  # g2o.Isometry3d 
//         if pose is None: 
//             pose = g2o.Isometry3d()      
//         self.set(pose)
//         self.covariance = np.identity(6)          # pose covariance



#[derive(Debug, Default, Clone)]
pub struct CameraPose {
    pub _pose: Poseg2o,
    pub covariance: Mat,
    pub Tcw: Mat,// self._pose.matrix()     # homogeneous transformation matrix: (4, 4)   pc_ = Tcw * pw_
    pub Rcw: Mat,// self.Tcw[:3,:3]
    pub tcw: Mat,// self.Tcw[:3,3]    #  pc = Rcw * pw + tcw
    pub Rwc: Mat,// self.Rcw.T
    pub Ow : Mat,//-(self.Rwc @ self.tcw)  # origin of camera frame w.r.t world
    pub is_initialized: bool,
  }

  impl CameraPose {
    //Construct CameraPose
    pub fn default() -> CameraPose {
        CameraPose {
            _pose: Poseg2o::default(),
            covariance: Mat::eye(6,6, CV_32F).unwrap().to_mat().unwrap(),
            Tcw: Mat::eye(4,4, CV_32F).unwrap().to_mat().unwrap(),// self._pose.matrix()     # homogeneous transformation matrix: (4, 4)   pc_ = Tcw * pw_
            Rcw: Mat::default(),// self.Tcw[:3,:3]
            tcw: Mat::default(),// self.Tcw[:3,3]    #  pc = Rcw * pw + tcw
            Rwc: Mat::default(),// self.Rcw.T
            Ow : Mat::default(),//-(self.Rwc @ self.tcw)  # origin of camera frame w.r.t world
            is_initialized: false,
      }
    }
    pub fn new(pose: &Poseg2o) -> CameraPose {
        CameraPose {
            _pose: pose.clone(),
            covariance: Mat::eye(6,6, CV_32F).unwrap().to_mat().unwrap(),
            Tcw: pose.Tcw().clone(),// self._pose.matrix()     # homogeneous transformation matrix: (4, 4)   pc_ = Tcw * pw_
            Rcw: pose.Rcw(),// self.Tcw[:3,:3]
            tcw: pose.tcw(),// self.Tcw[:3,3]    #  pc = Rcw * pw + tcw
            Rwc: pose.Rwc(),// self.Rcw.T
            Ow : pose.Ow(),//-(self.Rwc @ self.tcw)  # origin of camera frame w.r.t world
            is_initialized: false,
      }
    }

//     # input pose is expected to be an g2o.Isometry3d      
//     def set(self, pose):
//         if isinstance(pose, g2o.SE3Quat) or isinstance(pose, g2o.Isometry3d):
//             self._pose = g2o.Isometry3d(pose.orientation(), pose.position())
//         else:
//             self._pose = g2o.Isometry3d(pose)     # g2o.Isometry3d                          
//         #self.position = pose.position()          # np.zeros(3)
//         #self.orientation = pose.orientation()    # g2o.Quaternion(),  quat_cw     
//         self.Tcw = self._pose.matrix()     # homogeneous transformation matrix: (4, 4)   pc_ = Tcw * pw_
//         self.Rcw = self.Tcw[:3,:3]
//         self.tcw = self.Tcw[:3,3]    #  pc = Rcw * pw + tcw
//         self.Rwc = self.Rcw.T
//         self.Ow = -(self.Rwc @ self.tcw)  # origin of camera frame w.r.t world


    //pub fn set(&mut self, pose: &Poseg2o)
    pub fn set_old(&mut self, pose: &Mat)
    {
        self._pose = Poseg2o::new_old(pose);
        self.Tcw= self._pose.Tcw().clone();// self._pose.matrix()     # homogeneous transformation matrix: (4, 4)   pc_ = Tcw * pw_
        self.Rcw= self._pose.Rcw();// self.Tcw[:3,:3]
        self.tcw= self._pose.tcw();// self.Tcw[:3,3]    #  pc = Rcw * pw + tcw
        self.Rwc= self._pose.Rwc();// self.Rcw.T
        self.Ow = self._pose.Ow();//-(self.Rwc @ self.tcw)  # origin of camera frame w.r.t world
        self.is_initialized= true;
    }

    pub fn set(&mut self, pose: &Poseg2o)
    {
        self._pose = pose.clone();// Poseg2o::new(pose);
        self.Tcw= self._pose.Tcw().clone();// self._pose.matrix()     # homogeneous transformation matrix: (4, 4)   pc_ = Tcw * pw_
        self.Rcw= self._pose.Rcw();// self.Tcw[:3,:3]
        self.tcw= self._pose.tcw();// self.Tcw[:3,3]    #  pc = Rcw * pw + tcw
        self.Rwc= self._pose.Rwc();// self.Rcw.T
        self.Ow = self._pose.Ow();//-(self.Rwc @ self.tcw)  # origin of camera frame w.r.t world
        self.is_initialized= true;
    }


    //     def update(self,pose):
//         self.set(pose)
    //pub fn update(&mut self, pose: &Poseg2o)
    pub fn update_old(&mut self, pose: &Mat)
    {
        //self.set(pose)
    }
    pub fn update(&mut self, pose: &Poseg2o)
    {
        self.set(pose)
    }           
//     @property    
//     def isometry3d(self):  # pose as g2o.Isometry3d 
//         return self._pose 
    pub fn isometry3d(&mut self) -> Poseg2o
    {
        self._pose.clone() 
    }   

    //     @property    
//     def quaternion(self): # g2o.Quaternion(),  quaternion_cw  
//         return self._pose.orientation()  
    pub fn quaternion(&mut self) -> Mat
    {
        self._pose._pose.rotation.to_rotation_matrix().matrix().try_into_cv().unwrap()
    }     


//     @property    
//     def orientation(self): # g2o.Quaternion(),  quaternion_cw  
//         return self._pose.orientation()     
    pub fn orientation(&mut self) -> Mat
    {
        self.quaternion()
    }    

//     @property    
//     def position(self):    # 3D vector tcw (world origin w.r.t. camera frame) 
//         return self._pose.position()        
    pub fn position(&mut self) -> Mat
    {
        self._pose.tcw()
    }  
        
// //     def get_rotation_angle_axis(self):
// //         angle_axis = g2o.AngleAxis(self._pose.orientation())
// //         #angle = angle_axis.angle()
// //         #axis = angle_axis.axis()  
// //         return angle_axis  
//     pub fn get_rotation_angle_axis(&mut self) -> Vec3f
//     {
//         self._pose._pose.rotation.axis().try_into_cv().unwrap();
//         Vec3f::default()
//     }  
        

//     def get_inverse_matrix(self):
//         return self._pose.inverse().matrix()     
    pub fn get_inverse_matrix(&mut self) -> Mat
    {
        self._pose._pose.to_matrix().try_inverse().unwrap().try_into_cv().unwrap()
    }                
//     # set from orientation (g2o.Quaternion()) and position (3D vector)    
//     def set_from_quaternion_and_position(self,quaternion,position):
//         self.set(g2o.Isometry3d(quaternion, position))       
        
//     # set from 4x4 homogeneous transformation matrix Tcw  (pc_ = Tcw * pw_)
//     def set_from_matrix(self, Tcw):
//         self.set(g2o.Isometry3d(Tcw))
        
//     def set_from_rotation_and_translation(self, Rcw, tcw): 
//         self.set(g2o.Isometry3d(g2o.Quaternion(Rcw), tcw))     
        
//     def set_quaternion(self, quaternion):
//         self.set(g2o.Isometry3d(quaternion, self._pose.position()))  
                
//     def set_rotation_matrix(self, Rcw):
//         self.set(g2o.Isometry3d(g2o.Quaternion(Rcw), self._pose.position()))  
        
//     def set_translation(self, tcw):
//         self.set(g2o.Isometry3d(self._pose.orientation(), tcw))          
        

}


        

        



