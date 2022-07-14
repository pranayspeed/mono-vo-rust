#![allow(dead_code)]


pub struct Viewer3D {
    winname: String,
    width: i32,
    height:i32,
  }



  impl Viewer3D {
    //Construct person
    pub fn deafult() -> Viewer3D {
      Viewer3D {
        winname: String::from("3D Points"),
        width: 600,
        height: 600,
      }
    }

  }