use crate::frame_loader::FrameLoader;
use crate::viewer2d::Viewer2D;

use crate::camera::Camera;

use opencv::types::*;
use opencv::core::*;


use crate::visual_odometry::VisualOdometry;

use crate::slam::Slam;

pub fn slam_main() {


//     config = Config()

//     dataset = dataset_factory(config.dataset_settings)

//     #groundtruth = groundtruth_factory(config.dataset_settings)
//     groundtruth = None # not actually used by Slam() class; could be used for evaluating performances 



    // test data path for Kitti dataset 00 sequence
    let file_path = "/home/pranayspeed/Work/00/image_2";
    let scan_file_path = "/home/pranayspeed/Work/mono-vo-rust";
    // sequence_id is only needed if we have scale information
    // For our experiment, sequence_id is not needed.
    let sequence_id_str="00";

    let mut frame_loader = FrameLoader::new(file_path, scan_file_path, sequence_id_str);



//     cam = PinholeCamera(config.cam_settings['Camera.width'], config.cam_settings['Camera.height'],
//                         config.cam_settings['Camera.fx'], config.cam_settings['Camera.fy'],
//                         config.cam_settings['Camera.cx'], config.cam_settings['Camera.cy'],
//                         config.DistCoef, config.cam_settings['Camera.fps'])
    
//     num_features=2000 


    let cam = Camera::new(1241, 376, 718.856, 718.856, 607.1928, 185.2157, VectorOff32::from(vec![0.0,0.0,0.0,0.0,0.0]));

    let num_features=2000 ;


//     tracker_type = FeatureTrackerTypes.DES_BF      # descriptor-based, brute force matching with knn 
//     #tracker_type = FeatureTrackerTypes.DES_FLANN  # descriptor-based, FLANN-based matching 

//     # select your tracker configuration (see the file feature_tracker_configs.py) 
//     # FeatureTrackerConfigs: SHI_TOMASI_ORB, FAST_ORB, ORB, ORB2, ORB2_FREAK, ORB2_BEBLID, BRISK, AKAZE, FAST_FREAK, SIFT, ROOT_SIFT, SURF, SUPERPOINT, FAST_TFEAT, CONTEXTDESC
//     tracker_config = FeatureTrackerConfigs.TEST
//     tracker_config['num_features'] = num_features
//     tracker_config['tracker_type'] = tracker_type
    
//     print('tracker_config: ',tracker_config)    
//     feature_tracker = feature_tracker_factory(**tracker_config)
    
//     # create SLAM object 
//     slam = Slam(cam, feature_tracker, groundtruth)
//     time.sleep(1) # to show initial messages 

    

    let mut slam = Slam::new(&cam);

    //println!("slam camera {:?}", slam.camera.K);
    let mut display2d = Viewer2D::deafult();

    // // let frame1 = frame_loader.get_next_bw_frame();
    // // let frame2 = frame_loader.get_next_bw_frame();


    // // vo.init(frame1, frame2);
  
    display2d.init(600, 600);


    let img_id_start = 0;

    let mut img_processed = 0;
    for img_id in img_id_start..frame_loader.get_max_frame() {

        let curr_image_c = frame_loader.get_frame(img_id);
        //let curr_image = frame_loader.get_bw_from_color(&curr_image_c);
        let timestamp = 0.0; // currently ignoring timestamp

        slam.track(&curr_image_c, img_id, timestamp);

        if img_processed>2
        {
        
    //slam.tracking.update_history();
    //   ////////////////////////////////////Show on GUI///////////////////////////////////////////////////
        //println!("CURRENT TRACKING ..........{:?}, {:?}, {:?}", slam.tracking.cur_t.at_2d::<f32>(0, 0), slam.tracking.cur_t.at_2d::<f32>(1, 0), slam.tracking.cur_t.at_2d::<f32>(2, 0));

        display2d.update_2d_motion(&slam.tracking.cur_t);

        }
        display2d.update_current_frame(&curr_image_c);
    //   //////////////////////////////////////////////////////////////////////////////////////////////////
        



    img_processed+=1;
    //   #TOCHECK pyslam
    //   initializer.py line 132
    //       def initialize(self, f_cur, img_cur):  //line 97
    }




//     viewer3D = Viewer3D()
    
//     if platform.system()  == 'Linux':    
//         display2d = Display2D(cam.width, cam.height)  # pygame interface 
//     else: 
//         display2d = None  # enable this if you want to use opencv window

//     matched_points_plt = Mplot2d(xlabel='img id', ylabel='# matches',title='# matches')    

//     do_step = False   
//     is_paused = False 
    
//     img_id = 0  #180, 340, 400   # you can start from a desired frame id if needed 
//     while dataset.isOk():
            
//         if not is_paused: 
//             print('..................................')
//             print('image: ', img_id)                
//             img = dataset.getImageColor(img_id)
//             if img is None:
//                 print('image is empty')
//                 getchar()
//             timestamp = dataset.getTimestamp()          # get current timestamp 
//             next_timestamp = dataset.getNextTimestamp() # get next timestamp 
//             frame_duration = next_timestamp-timestamp 

//             if img is not None:
//                 time_start = time.time()                  
//                 slam.track(img, img_id, timestamp)  # main SLAM function 
                                
//                 # 3D display (map display)
//                 if viewer3D is not None:
//                     viewer3D.draw_map(slam)

//                 img_draw = slam.map.draw_feature_trails(img)
                    
//                 # 2D display (image display)
//                 if display2d is not None:
//                     display2d.draw(img_draw)
//                 else: 
//                     cv2.imshow('Camera', img_draw)

//                 if matched_points_plt is not None: 
//                     if slam.tracking.num_matched_kps is not None: 
//                         matched_kps_signal = [img_id, slam.tracking.num_matched_kps]     
//                         matched_points_plt.draw(matched_kps_signal,'# keypoint matches',color='r')                         
//                     if slam.tracking.num_inliers is not None: 
//                         inliers_signal = [img_id, slam.tracking.num_inliers]                    
//                         matched_points_plt.draw(inliers_signal,'# inliers',color='g')
//                     if slam.tracking.num_matched_map_points is not None: 
//                         valid_matched_map_points_signal = [img_id, slam.tracking.num_matched_map_points]   # valid matched map points (in current pose optimization)                                       
//                         matched_points_plt.draw(valid_matched_map_points_signal,'# matched map pts', color='b')  
//                     if slam.tracking.num_kf_ref_tracked_points is not None: 
//                         kf_ref_tracked_points_signal = [img_id, slam.tracking.num_kf_ref_tracked_points]                    
//                         matched_points_plt.draw(kf_ref_tracked_points_signal,'# $KF_{ref}$  tracked pts',color='c')   
//                     if slam.tracking.descriptor_distance_sigma is not None: 
//                         descriptor_sigma_signal = [img_id, slam.tracking.descriptor_distance_sigma]                    
//                         matched_points_plt.draw(descriptor_sigma_signal,'descriptor distance $\sigma_{th}$',color='k')                                                                 
//                     matched_points_plt.refresh()    
                
//                 duration = time.time()-time_start 
//                 if(frame_duration > duration):
//                     print('sleeping for frame')
//                     time.sleep(frame_duration-duration)        
                    
//             img_id += 1  
//         else:
//             time.sleep(1)                                 
        
//         # get keys 
//         key = matched_points_plt.get_key()  
//         key_cv = cv2.waitKey(1) & 0xFF    
        
//         # manage interface infos  
        
//         if slam.tracking.state==SlamState.LOST:
//             if display2d is not None:     
//                 getchar()                              
//             else: 
//                 key_cv = cv2.waitKey(0) & 0xFF   # useful when drawing stuff for debugging 
         
//         if do_step and img_id > 1:
//             # stop at each frame
//             if display2d is not None:            
//                 getchar()  
//             else: 
//                 key_cv = cv2.waitKey(0) & 0xFF         
        
//         if key == 'd' or (key_cv == ord('d')):
//             do_step = not do_step  
//             Printer.green('do step: ', do_step) 
                      
//         if key == 'q' or (key_cv == ord('q')):
//             if display2d is not None:
//                 display2d.quit()
//             if viewer3D is not None:
//                 viewer3D.quit()
//             if matched_points_plt is not None:
//                 matched_points_plt.quit()
//             break
        
//         if viewer3D is not None:
//             is_paused = not viewer3D.is_paused()         
                        
//     slam.quit()
    
//     #cv2.waitKey(0)
//     cv2.destroyAllWindows()



    // let mut vo = VisualOdometry::deafult();
  





    // /////////////////////////////////////
    // //let vidwer3d = Viewer3D::deafult();
    // ///////////////////////
    


  }
  




// done till slam.py line number 511
