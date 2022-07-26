#![allow(dead_code)]

use opencv::prelude::*;
use opencv::core::*;
use opencv::imgproc::*;


pub struct Viewer2D {
    winname: String,
    width: i32,
    height:i32,
    traj_img: Mat,
    offset_x: i32,
    offset_y:i32,
  }



  impl Viewer2D {
    //Construct person
    pub fn deafult() -> Viewer2D {
        Viewer2D {
        winname: String::from("Trajectory"),
        width: 600,
        height: 600,
        traj_img: Mat::default(),
        offset_x: 300,
        offset_y: 100,
      }
    }

    pub fn init(&mut self, width: i32, height: i32)
    {


        self.width = width;
        self.height = height;
        opencv::highgui::named_window("Road facing camera", opencv::highgui::WINDOW_AUTOSIZE).unwrap(); // Create a window for display.

        opencv::highgui::named_window(&self.winname, opencv::highgui::WINDOW_AUTOSIZE).unwrap(); // Create a window for display.


        self.traj_img = opencv::core::Mat::zeros(self.width, self.height , CV_8UC3)
          .unwrap()
          .to_mat()
          .unwrap();
    }

    pub fn update_current_frame(&mut self, curr_image_c : &Mat)
    {
        opencv::highgui::imshow("Road facing camera", &curr_image_c).unwrap();
    }
    pub fn update_2d_motion(&mut self, translation: &Mat)
    {

      let x_c = *translation.at::<f32>(0).unwrap() as i32 + &self.offset_x;
      let y_c = *translation.at::<f32>(2).unwrap() as i32 + &self.offset_y;
  
      circle(
        &mut self.traj_img,
        Point { x: x_c, y: y_c },
        1,
        Scalar::new(255.0, 0.0, 0.0, 0.0),
        2,
        LINE_8,
        0,
      )
      .unwrap();
  
      rectangle(
        &mut self.traj_img,
        Rect2i {
          x: 10,
          y: 30,
          width: 550,
          height: 50,
        },
        Scalar::new(0.0, 0.0, 0.0, 0.0),
        FILLED,
        0,
        0,
      )
      .unwrap();
      let text = format!(
        "Coordinates: x = {:.2}m y = {:.2}m z = {:.2}m",
        translation.at::<f32>(0).unwrap(),
        translation.at::<f32>(1).unwrap(),
        translation.at::<f32>(2).unwrap()
      );
  

      let font_face: i32 = FONT_HERSHEY_PLAIN;
      let font_scale: f64 = 1.0;
      let thickness: i32 = 1;
      let text_org = Point::new(10, 50);


      opencv::imgproc::put_text(
        &mut self.traj_img,
        &text,
        text_org,
        font_face,
        font_scale,
        Scalar::new(255.0, 255.0, 255.0, 255.0),
        thickness,
        8,
        false,
      )
      .unwrap();
  

      opencv::highgui::imshow(&self.winname, &self.traj_img).unwrap();
  
      opencv::highgui::wait_key(1).unwrap();
    }


}