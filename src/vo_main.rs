// mod visodo_w;

use crate::visual_odometry::VisualOdometry;
use crate::frame_loader::FrameLoader;
use crate::viewer2d::Viewer2D;

use crate::viewer3d::Viewer3D;

pub fn vo_main(scale_value: bool) {

    // test data path for Kitti dataset 00 sequence
    let file_path = "/home/pranayspeed/Work/00/image_2";
    let scan_file_path = "/home/pranayspeed/Work/mono-vo-rust";
    // sequence_id is only needed if we have scale information
    // For our experiment, sequence_id is not needed.
    let sequence_id_str="00";

    let mut frame_loader = FrameLoader::new(file_path, scan_file_path, sequence_id_str);

    let mut vo = VisualOdometry::deafult();
  
    let frame1 = frame_loader.get_next_bw_frame();
    let frame2 = frame_loader.get_next_bw_frame();


    vo.init(frame1, frame2);
  
    let mut viewer2d = Viewer2D::deafult();

    viewer2d.init(600, 600);


    /////////////////////////////////////
    //let vidwer3d = Viewer3D::deafult();
    ///////////////////////
    


    for num_frame in 2..frame_loader.get_max_frame() {
      let curr_image_c = frame_loader.get_next_frame();
      let curr_image = frame_loader.get_bw_from_color(&curr_image_c);

      let mut scale =1.0;
      if scale_value {
          scale = frame_loader.get_absolute_scale(num_frame);
      }

      //Track new frame with scale
      //if scale value not avaialble, set it to 1.0
      vo.track_frame(&curr_image, scale);

      ////////////////////////////////////Show on GUI///////////////////////////////////////////////////
      viewer2d.update_2d_motion(vo.get_translation());
      viewer2d.update_current_frame(&curr_image_c);
      //////////////////////////////////////////////////////////////////////////////////////////////////



      //#TOCHECK pyslam
      // initializer.py line 132
      //     def initialize(self, f_cur, img_cur):  //line 97
    }
  }
  

