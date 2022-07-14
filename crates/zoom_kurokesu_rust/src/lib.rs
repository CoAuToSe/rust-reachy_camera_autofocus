//! Rust Package for controlling the zoom of Kurokesu"s camera.
// import serial
pub use serialport::*;
use std::io::{Read, Write};
use std::sync::Arc;
// import time
// pub use json::*;
pub use std::time;

// #[macro_use]
// #[path = "personal_macro.rs"]
// pub mod personal_macro;
// pub use personal_macro::*;
use utils::*;

// #[allow(unused)]
// fn inside<T: std::cmp::PartialEq>(to_compate: &T, rtgze: Vec<T>) -> bool {
//     for e in rtgze {
//         if e == *to_compate {
//             return true;
//         }
//     }
//     return false;
// }

pub struct ZoomController {
    pub ser: Box<dyn SerialPort>,
    pub speed: isize,
}
// pub struct ZoomFocus<T> {
//     pub zoom: T,
//     pub focus: T,
// }

// pub enum Side<T> {
//     Left(T),
//     Right(T),
// }
// pub enum SideOnly {
//     Left,
//     Right,
// }

// impl TryFrom<&str> for SideOnly {
//     fn try_from(side: &str) -> std::result::Result<Self, ()> {
//         match side {
//             "left_eye" | "left_side" | "left" => Ok(Self::Left),
//             "right_eye" | "right_side" | "right" => Ok(Self::Right),
//             _ => Err(()),
//         }
//     }
//     type Error = ();
// }

// impl TryFrom<String> for SideOnly {
//     fn try_from(side: String) -> std::result::Result<Self, ()> {
//         match &side as &str {
//             "left_eye" | "left_side" | "left" => Ok(Self::Left),
//             "right_eye" | "right_side" | "right" => Ok(Self::Right),
//             _ => Err(()),
//         }
//     }
//     type Error = ();
// }

// #[derive(Clone, Copy, Debug)]
// pub enum ZoomLevel {
//     In,
//     Inter,
//     Out,
// }
// impl TryFrom<&str> for ZoomLevel {
//     fn try_from(zoom_level: &str) -> std::result::Result<Self, ()> {
//         match zoom_level {
//             "in" | "In" | "IN" => Ok(Self::In),
//             "inter" | "Inter" | "INTER" => Ok(Self::Inter),
//             "out" | "Out" | "OUT" => Ok(Self::Out),
//             _ => Err(()),
//         }
//     }
//     type Error = ();
// }
// impl TryFrom<String> for ZoomLevel {
//     fn try_from(zoom_level: String) -> std::result::Result<Self, ()> {
//         match &zoom_level as &str {
//             "in" | "In" | "IN" => Ok(Self::In),
//             "inter" | "Inter" | "INTER" => Ok(Self::Inter),
//             "out" | "Out" | "OUT" => Ok(Self::Out),
//             _ => Err(()),
//         }
//     }
//     type Error = ();
// }
// impl From<Side<ZoomLevel>> for Side<ZoomFocus<isize>> {
//     fn from(side_zoom_level: Side<ZoomLevel>) -> Self {
//         match side_zoom_level {
//             Side::Left(zoom_level) => match zoom_level {
//                 ZoomLevel::In => Side::Left(ZoomFocus {
//                     zoom: 457,
//                     focus: 70,
//                 }),
//                 ZoomLevel::Inter => Side::Left(ZoomFocus {
//                     zoom: 270,
//                     focus: 331,
//                 }),
//                 ZoomLevel::Out => Side::Left(ZoomFocus {
//                     zoom: 60,
//                     focus: 455,
//                 }),
//             },
//             Side::Right(zoom_level) => match zoom_level {
//                 ZoomLevel::In => Side::Right(ZoomFocus {
//                     zoom: 457,
//                     focus: 42,
//                 }),
//                 ZoomLevel::Inter => Side::Right(ZoomFocus {
//                     zoom: 270,
//                     focus: 321,
//                 }),
//                 ZoomLevel::Out => Side::Right(ZoomFocus {
//                     zoom: 60,
//                     focus: 445,
//                 }),
//             },
//         }
//     }
// }

// impl<'a, T> Side<T> {
//     pub const fn zoom(&self) -> &'a str {
//         match self {
//             Self::Left(_) => "X",
//             Self::Right(_) => "Z",
//         }
//     }
//     pub const fn focus(&self) -> &'a str {
//         match self {
//             Self::Left(_) => "Y",
//             Self::Right(_) => "A",
//         }
//     }
// }

// impl<'a, T> Side<ZoomFocus<T>> {
//     pub fn zoom_value(&'a self) -> &'a T {
//         match self {
//             Self::Left(zoom_focus) => &zoom_focus.zoom,
//             Self::Right(zoom_focus) => &zoom_focus.zoom,
//         }
//     }
//     pub fn focus_value(&'a self) -> &'a T {
//         match self {
//             Self::Left(zoom_focus) => &zoom_focus.focus,
//             Self::Right(zoom_focus) => &zoom_focus.focus,
//         }
//     }
// }
impl Default for ZoomController {
    fn default() -> Self {
        ZoomController::new("/dev/ttyACM0", 115200, 10, 10_000)
    }
}

impl ZoomController {
    ///Connect to the serial port and run the initialisation sequence.
    pub fn new(port: &str, baudrate: isize, timeout: isize, speed: isize) -> Self {
        let mut ser = serialport::new(port, baudrate.try_into().unwrap())
            .timeout(std::time::Duration::from_secs(timeout.try_into().unwrap()))
            .open()
            .expect("Failed to open port");
        let speed = speed;

        let init_seq = "G100 P9 L144 N0 S0 F1 R1";
        // print!("\r{:?}              ", format!("{}{}", init_seq, "\\n"));
        // std::io::stdout().flush().unwrap();
        println!("{}{}", init_seq, "\\n");

        ser.write(format!("{}{}", init_seq, "\n").as_bytes())
            .unwrap();

        // println!("wrinting finished");
        // let zae = ser.

        // std::thread::sleep(time::Duration::from_secs(1));
        // let mut buf: Vec<u8> = vec![];
        // let mut azer = String::new();
        // while azer == String::new() {
        // let _size_read = ser.read(&mut buf);
        // let temp = ser.bytes_to_read();
        // let aze = ser.bytes_to_write();
        // let aztte = ser.bytes().count();//no
        // let aztte = ser.read_to_string(&mut azer); //do something

        // println!("{:?}", ser.bytes_to_read());
        // self.ser.read_to_string(&mut buf);
        let mut buf = [0; 128];
        ser.read(&mut buf);
        let response = String::from_utf8(buf.to_vec()).unwrap();
        // println!("reading finished {:?}", response);

        // println!("{{buf:?}} | {{temp:?}}| {{aze:?}}| {aztte:?}| {azer:?}");
        // std::io::stdout().flush().unwrap();
        // }
        // let response = aztte;
        // eprintln!(
        //     "{:?} if equal to 1, means that didn't receive \"ok\" response",
        //     response.split("\r\n").collect::<Vec<&str>>().len()
        // );
        // if response != String::from("ok\r\n") {
        //     panic!("Initialization of zoom controller failed, check that the control board is correctly plugged in.")
        // }
        let emp: Vec<_> = response.split("\n").collect();
        // let aemp = String::from("").split("\n").count();
        // println!("{emp:?} {aemp:?}");
        if emp.len() == 1 {
            panic!("Initialization of zoom controller failed, check that the control board is correctly plugged in.")
        }
        ZoomController { ser, speed }
    }
    ///Set zoom level of a given camera.
    ///
    ///Given the camera side and the zoom level required,
    ///produce the corresponding G-code and send it over the serial port.
    ///
    ///Args:
    ///    side: "right" or "left"
    ///    zoom_level: either "in", "inter" or "out". "in" level for far
    ///         objects, "out" for close objects, "inter" is in between
    ///         "in" and "out" levels
    ///
    pub fn set_zoom_level(&mut self, size: &SideOnly, zoom_level: &ZoomLevel) -> () {
        match size {
            SideOnly::Left => match zoom_level {
                ZoomLevel::In => self._send_custom_command(
                    Side::Left(ZoomFocus {
                        zoom: 457,
                        focus: 70,
                    })
                    .into(),
                ),
                ZoomLevel::Inter => self._send_custom_command(
                    Side::Left(ZoomFocus {
                        zoom: 270,
                        focus: 331,
                    })
                    .into(),
                ),
                ZoomLevel::Out => self._send_custom_command(
                    Side::Left(ZoomFocus {
                        zoom: 60,
                        focus: 455,
                    })
                    .into(),
                ),
            },
            SideOnly::Right => match zoom_level {
                ZoomLevel::In => self._send_custom_command(
                    Side::Right(ZoomFocus {
                        zoom: 457,
                        focus: 42,
                    })
                    .into(),
                ),
                ZoomLevel::Inter => self._send_custom_command(
                    Side::Right(ZoomFocus {
                        zoom: 270,
                        focus: 321,
                    })
                    .into(),
                ),
                ZoomLevel::Out => self._send_custom_command(
                    Side::Right(ZoomFocus {
                        zoom: 60,
                        focus: 445,
                    })
                    .into(),
                ),
            },
        }
    }

    ///Send custom command to camera controller.
    ///
    ///Args:
    ///    commands: dictionnary containing the requested camera name along
    ///    with requested focus and zoom value. Instructions for both cameras
    ///    can be sent in one call of this method. However, instructions will
    ///    be sent sequentially and there is no synchronization.
    ///
    pub fn _send_custom_command<T>(&mut self, commands: Vec<Side<ZoomOrFocus<T>>>)
    where
        T: Into<i32> + Clone + Copy,
    {
        let speedy = self.speed;
        for command in commands {
            match &command.inner() {
                ZoomOrFocus::Zoom(value) => {
                    let message_zoom = |meg: &Side<ZoomOrFocus<T>>| -> String {
                        let speed = &speedy;
                        let ident = meg.zoom();
                        // let value: i32 = (*meg.zoom_value()).clone().into();
                        let value: i32 = value.clone().into();
                        format!("G1 {ident}{value} F{speed}")
                    };
                    self.send_no_out(&command, &message_zoom).unwrap();
                }
                ZoomOrFocus::Focus(value) => {
                    let message_focus = |meg: &Side<ZoomOrFocus<T>>| -> String {
                        let speed = &speedy;
                        let ident = meg.focus();
                        // let value: i32 = (*meg.focus_value()).clone().into();
                        let value: i32 = value.clone().into();
                        format!("G1 {ident}{value} F{speed}")
                    };
                    self.send_no_out(&command, &message_focus).unwrap();
                }
            }
        }
    }
    /// simplification of communication
    ///
    /// # Exemple:
    ///
    /// ```rust
    /// let zoom_controller = ZoomController::default();
    /// let cmd_gen = |msg: &Side<ZoomFocus<isize>>| -> String {
    ///     let zoom = msg.zoom();
    ///     let zoom_value = msg.zoom_value();
    ///     let focus = msg.focus();
    ///     let focus_value = msg.focus_value();
    ///     format!("G92 {zoom}{zoom_value} {focus}{focus_value}")
    /// };
    /// zoom_controller.send_no_out(Side::Left(ZoomFocus { zoom: 0, focus: 0 }), cmd_gen);
    /// ```
    pub fn send_no_out<'b, T>(
        &'b mut self,
        cmd: &Side<ZoomOrFocus<T>>,
        formatter: &dyn Fn(&Side<ZoomOrFocus<T>>) -> String,
    ) -> std::io::Result<()> {
        // print!(
        //     "\r{:?}               ",
        //     format!("{}{}", formatter(&cmd), "\n")
        // );
        // std::io::stdout().flush().unwrap();

        println!("{}", format!("{}{}", formatter(&cmd), "\\n"));

        self.ser
            .write(format!("{}{}", formatter(&cmd), "\n").as_bytes())?;
        // println!("wrinting finished");
        // println!("{:?}", self.ser.bytes_to_read());
        let mut buf = [0; 128];
        // self.ser.read_to_string(&mut buf);
        self.ser.read(&mut buf).unwrap();
        // println!("reading finished {:?}", String::from_utf8(buf.to_vec()));
        Ok(())
    }
    pub fn send_no_out_string<'b>(&'b mut self, cmd: String) -> std::io::Result<()> {
        // print!(
        //     "\r{:?}               ",
        //     format!("{}{}", formatter(&cmd), "\n")
        // );
        // std::io::stdout().flush().unwrap();

        println!("{}\\n", cmd);

        self.ser.write(format!("{}{}", cmd, "\n").as_bytes())?;
        let mut buf = String::new();
        // let _ = self.ser.read_to_string(&mut buf);
        let mut buf = [0; 128];
        self.ser.read(&mut buf).unwrap();
        // let response = String::from_utf8(buf.to_vec()).unwrap();
        Ok(())
    }

    ///Use serial port to perform homing sequence on given camera.
    ///
    ///Args:
    ///    side: "right", "left".
    ///
    pub fn homing(&mut self, side_impl: SideOnly) -> () {
        // let side =
        //     SideOnly::try_from(side_impl).unwrap_or(panic!("could not understand homing request"));
        let side = side_impl;
        let msg_gen = match side {
            SideOnly::Left => {
                |zoom, focus| -> Side<ZoomFocus<_>> { Side::Left(ZoomFocus { zoom, focus }) }
            }
            SideOnly::Right => {
                |zoom, focus| -> Side<ZoomFocus<_>> { Side::Right(ZoomFocus { zoom, focus }) }
            }
        };
        let msg_def = match side {
            SideOnly::Left => Side::Left(ZoomFocus { zoom: 0, focus: 0 }),
            SideOnly::Right => Side::Right(ZoomFocus { zoom: 0, focus: 0 }),
        };
        // let mot = self.motors[self.connector[side]];
        let cmd_gen = |msg: &Side<ZoomFocus<i32>>| -> String {
            let zoom = msg.zoom();
            let zoom_value = msg.zoom_value().clone();
            let focus = msg.focus();
            let focus_value = msg.focus_value();
            format!("G92 {zoom}{zoom_value} {focus}{focus_value}")
        };
        // self.send_no_out(&msg_def, &cmd_gen).unwrap();
        self.send_no_out_string(cmd_gen(&msg_def)).unwrap();
        std::thread::sleep(std::time::Duration::from_millis(100));
        self._send_custom_command(msg_gen(0, -500).into());
        std::thread::sleep(std::time::Duration::from_secs(1));
        self._send_custom_command(msg_gen(-600, -500).into());
        std::thread::sleep(std::time::Duration::from_secs(1));
        // self.send_no_out(&msg_def, &cmd_gen).unwrap();
        self.send_no_out_string(cmd_gen(&msg_def)).unwrap();
        std::thread::sleep(std::time::Duration::from_millis(100))
    }

    ///Set motors speed.
    ///
    ///Args:
    ///    speed_value: int between 4000 and 40000
    ///
    pub fn set_speed<Inum: Into<isize>>(
        &mut self,
        speed_value: Inum,
    ) -> std::result::Result<(), &str> {
        let speed: isize = speed_value.into();
        if !(4000 <= speed && speed <= 40000) {
            return Err("Speed value must be between 4000 and 40000");
        }
        self.speed = speed;
        Ok(())
    }
}
// fn main() {
//     println!("shouldn't happend (calling of zoom_kurokesu)")
// }
