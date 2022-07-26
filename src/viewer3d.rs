#![allow(dead_code)]



//extern crate kiss3d;
extern crate nalgebra as na;

use std::thread::JoinHandle;

use na::{Vector3, UnitQuaternion};
//use kiss3d::window::Window;
//use kiss3d::light::Light;

pub struct Viewer3D {

}
impl Viewer3D {
}

// pub struct Viewer3D {
//     winname: String,
//     width: i32,
//     height:i32,
//     window_thread_handle: JoinHandle<()>
//   }



//   impl Viewer3D {
//     //Construct person
//     pub fn deafult() -> Viewer3D {
//       Viewer3D {
//         winname: String::from("3D Points"),
//         width: 600,
//         height: 600,
//         window_thread_handle: std::thread::spawn(|| Viewer3D::update(5,5))
//       }
//     }


//     pub fn update(x: i32, y: i32) {
//       let mut window = Window::new("Kiss3d: cube");
//       let mut c      = window.add_cube(1.0, 1.0, 1.0);
  
//       c.set_color(1.0, 0.0, 0.0);
  
//       window.set_light(Light::StickToCamera);
  
//       let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);
  
//       while window.render() {
//           c.prepend_to_local_rotation(&rot);
//       }
//   }
//     pub fn join(&self)
//     {
//       //self.window_thread_handle.join().unwrap();      
//     }

  
//   }
