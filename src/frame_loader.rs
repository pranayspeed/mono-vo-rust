#![allow(dead_code)]

use std::fs::File;
use std::io::{self, BufRead};
use std::path::Path;


use opencv::imgcodecs::*;

use opencv::core::*;

use opencv::imgproc::*;


fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<File>>>
where
  P: AsRef<Path>,
{
  let file = File::open(filename)?;
  Ok(io::BufReader::new(file).lines())
}



pub struct FrameLoader {
    frame_idx: i32,
    file_path: String,
    scale_file_path: String,
    max_frame: i32,
    sequence_id: String,
  }

  impl FrameLoader {
    //Construct person
    pub fn new(filepath: &str, scale_filepath: &str, sequence_id_str: &str) -> FrameLoader {
        FrameLoader {
        frame_idx:0,
        file_path: String::from(filepath),
        scale_file_path: String::from(scale_filepath),
        max_frame: 2000,
        sequence_id: String::from(sequence_id_str),
      }
    }

    pub fn get_next_frame(&mut self) -> Mat {    
        let filename = format!("{}{}", self.file_path, format!("/{:01$}.png", self.frame_idx, 6));
        self.frame_idx +=1;
        imread(&filename, IMREAD_COLOR).unwrap()
      }



      pub fn get_next_bw_frame(&mut self) -> Mat {
        let filename = format!("{}{}", self.file_path, format!("/{:01$}.png", self.frame_idx, 6));
        self.frame_idx +=1;
        let curr_image_c = imread(&filename, IMREAD_COLOR).unwrap();
        let mut img_1 = Mat::default();
        cvt_color(&curr_image_c, &mut img_1, COLOR_BGR2GRAY, 0).unwrap();
        img_1
      }      


      pub fn get_frame(&self, frame_id: i32) -> Mat {
        let filename = format!("{}{}", self.file_path, format!("/{:01$}.png", frame_id, 6));
        imread(&filename, IMREAD_COLOR).unwrap()
      }


      pub fn get_bw_frame(&self, frame_id: i32) -> Mat {
        let filename = format!("{}{}", self.file_path, format!("/{:01$}.png", frame_id, 6));
        let curr_image_c = imread(&filename, IMREAD_COLOR).unwrap();
        let mut img_1 = Mat::default();
        cvt_color(&curr_image_c, &mut img_1, COLOR_BGR2GRAY, 0).unwrap();
        img_1
      }
    
      pub fn get_bw_from_color(&self, img_color: &Mat) -> Mat {
        let mut img_1 = Mat::default();
        cvt_color(img_color, &mut img_1, COLOR_BGR2GRAY, 0).unwrap();
        img_1
      }


      pub fn get_max_frame(&self) -> i32
      {
        self.max_frame
      }

      pub fn get_absolute_scale(
        &self,
        frame_id: i32,
        //curr_pos: &mut Point2d
      ) -> f64 {
        let mut i = 0;
    
        let filepath = format!("{}/{}.txt", &self.scale_file_path, self.sequence_id);
        let filepath_path = Path::new(&filepath);
    
        let (mut x, mut y, mut z): (f64, f64, f64) = (0.0, 0.0, 0.0);
    
        let (mut x_prev, mut y_prev, mut z_prev): (f64, f64, f64) = (0.0, 0.0, 0.0);
        // Open the path in read-only mode, returns `io::Result<File>`
        if let Ok(lines) = read_lines(filepath_path) {
          // Consumes the iterator, returns an (Optional) String
          for line in lines {
            if let Ok(val) = line {
              z_prev = z;
              x_prev = x;
              y_prev = y;
    
              println!("x : {}, y: {}, z:{} ", x, y, z);
              let values: Vec<&str> = val.split(" ").collect();
              for (j, s) in values.iter().enumerate() {
                z = s.parse::<f64>().unwrap();
                if j == 7 {
                  y = z;
                }
                if j == 3 {
                  x = z;
                }
              }
            }
            i = i + 1;
            if i > frame_id {
              break;
            }
          }
        }
    
        //curr_pos.x = x_prev;
        //curr_pos.y = z_prev;
        f64::sqrt(
          (x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev) + (z - z_prev) * (z - z_prev),
        )
      }
}