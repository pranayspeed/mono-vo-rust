// mod visodo_w;
mod visual_odometry;
mod frame_loader;
mod viewer2d;
mod viewer3d;
mod camera;
mod frame;
mod dvutils;
mod vo_main;

fn main() {
  vo_main::vo_main(true);
}
