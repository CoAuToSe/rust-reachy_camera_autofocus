use v4l::buffer::Type;
use v4l::io::mmap::Stream;
use v4l::io::traits::CaptureStream;
use v4l::video::Capture;
use v4l::Device;
use v4l::FourCC;

use sensor_msgs::msg::CompressedImage as CImage;

use anyhow::{Error, Result};
use std::env;
use std::sync::{Arc, Mutex};

const LEFT: bool = true;
const RIGHT: bool = true;

fn main() {
    let mut some = CameraPublisher::new(
        "image_publisher",
        640,
        480,
        b"MJPG",
        "/dev/video2",
        "/dev/video6",
    );
    &some.thread1.unwrap().join().unwrap();
    // println!("{:?}", some);
    std::thread::sleep(std::time::Duration::from_millis(10000));
}

struct CameraPublisher {
    node: rclrs::Node,
    left: Option<Arc<Mutex<Camera>>>,
    right: Option<Arc<Mutex<Camera>>>,
    thread1: Option<std::thread::JoinHandle<()>>,
    thread2: Option<std::thread::JoinHandle<()>>,
}
impl std::fmt::Debug for CameraPublisher {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CameraPublisher")
            .field("node", &self.node)
            .field("left", &self.left)
            .field("right", &self.right)
            .field("thread1", &self.thread1)
            .field("thread2", &self.thread2)
            .finish()
    }
}

impl CameraPublisher {
    fn new(
        name: &str,
        width: u32,
        height: u32,
        forma: &[u8; 4],
        left_path: &str,
        right_path: &str,
    ) -> Self {
        let context = rclrs::Context::new(env::args()).unwrap();
        // let width = 640;
        // let height = 480;
        let node = context.create_node(name).unwrap();
        // std::thread::sleep(std::time::Duration::from_millis(10000));
        // let forma = b"MJPG";
        let left = if LEFT {
            Some(Arc::new(Mutex::new(Camera::new(
                &node,
                "left_image",
                left_path, // "/dev/video0",
                width,
                height,
                forma,
            ))))
        } else {
            None
        };
        let right = if RIGHT {
            Some(Arc::new(Mutex::new(Camera::new(
                &node,
                "right_image",
                right_path, // "/dev/video2",
                width,
                height,
                forma,
            ))))
        } else {
            None
        };

        let thread1;
        if let Some(ref val) = left {
            thread1 = if LEFT {
                let clone1 = Arc::clone(&val);
                Some(std::thread::spawn(move || {
                    clone1.lock().unwrap().publisherin("left");
                }))
            } else {
                None
            };
        } else {
            thread1 = None;
        }

        let thread2;
        if let Some(ref val) = right {
            thread2 = if RIGHT {
                let clone2 = Arc::clone(&val);
                Some(std::thread::spawn(move || {
                    clone2.lock().unwrap().publisherin("right");
                }))
            } else {
                None
            };
        } else {
            thread2 = None;
        }

        CameraPublisher {
            node,
            left,
            right,
            thread1,
            thread2,
        }
    }
}

struct Camera {
    device: v4l::Device,
    publisher: rclrs::Publisher<CImage>,
    compr_img: CImage,
}

impl std::fmt::Debug for Camera {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CameraPublisher")
            .field("device", &"hi")
            .field("publisher", &"hi")
            .field("compr_img", &self.compr_img)
            .finish()
    }
}

impl Camera {
    fn new(
        node: &rclrs::Node,
        topic: &str,
        path: &str,
        width: u32,
        height: u32,
        format: &[u8; 4],
    ) -> Camera {
        let device = Device::with_path(path).expect("Failed to open device");

        // Let's say we want to explicitly request another format
        let mut fmt = device.format().expect("Failed to read format");
        fmt.width = width;
        fmt.height = height;
        fmt.fourcc = FourCC::new(format);
        device.set_format(&fmt).expect("Failed to write format");

        let publisher = node
            .create_publisher::<CImage>(topic, rclrs::QOS_PROFILE_DEFAULT)
            .unwrap();

        let mut compr_img: CImage = Default::default();
        compr_img.format = String::from("jpeg");
        Camera {
            device,
            publisher,
            compr_img,
        }
    }

    fn publisherin(&mut self, side: &str) {
        let angle: usize;
        if side == "right" {
            angle = 270
        } else {
            angle = 90
        }
        let mut stream = Stream::with_buffers(&mut &self.device, Type::VideoCapture, 4)
            .expect("Failed to create buffer stream");

        while let Ok(frame) = stream.next() {
            print!("\rsending {side:?} image");
            use std::io::Write;
            std::io::stdout().flush().unwrap();
            //maybe overkill
            self.publish_img(Vec::from(
                // self._rotate(Vec::try_from(frame.0).unwrap(), angle)
                //     .unwrap()
                //     .as_bytes(),
                // self._rotate(
                Vec::try_from(frame.0).unwrap(), // , angle)
                                                 // .unwrap()
                                                 // .as_bytes(),
            ));
            // stream.
            std::thread::sleep(std::time::Duration::from_millis(500));
        }
    }
    fn publish_img(&mut self, frame: Vec<u8>) {
        let mut compr_img = &mut self.compr_img;
        compr_img.header.stamp = builtin_interfaces::msg::Time::default();
        compr_img.data = frame;
        self.publisher.publish(&*compr_img);
    }

    fn _rotate(&self, frame: Vec<u8>, angle: usize) -> Option<std::string::String> {
        use subprocess::{Popen, PopenConfig, Redirection};
        let mut process = Popen::create(
            &["jpegtran", "-rotate", format!("{}", angle).as_str()],
            PopenConfig {
                stdin: Redirection::Pipe,
                stdout: Redirection::Pipe,
                stderr: Redirection::Pipe,
                detached: Default::default(),
                executable: Default::default(),
                env: Default::default(),
                cwd: Default::default(),
                setuid: Default::default(),
                setgid: Default::default(),
                setpgid: Default::default(),
                _use_default_to_construct: Default::default(),
            },
        )
        .unwrap();
        // rclrs::rcl_node_fini();
        let (out, _err) = process.communicate(Some("")).unwrap();
        process.wait().unwrap();
        return out;
    }
}
