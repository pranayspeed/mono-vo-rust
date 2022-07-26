// mod visodo_w;
mod visual_odometry;
mod frame_loader;
mod viewer2d;
mod viewer3d;
mod camera;
mod frame;
mod dvutils;
mod parameters;
mod utils_geom;
mod camera_pose;
mod vo_main;
mod slam_main;
mod tracking;
mod slam;
mod feature_tracker;
mod initializer;
mod map;
fn main() {
  //vo_main::vo_main(true);
  
  slam_main::slam_main();
}

