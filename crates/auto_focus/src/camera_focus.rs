//! Node to perform autofocus on Reachy"s cameras.
use futures::executor::block_on;

use rclrs::{Client, Node, Service, Subscription};
use reachy_msgs::{
    msg::ZoomCommand,
    srv::{
        GetCameraZoomFocus, GetCameraZoomFocus_Request, GetCameraZoomFocus_Response,
        SetCameraZoomFocus, SetCameraZoomFocus_Request, SetCameraZoomFocus_Response, SetFocusState,
        SetFocusState_Request, SetFocusState_Response,
    },
};
use sensor_msgs::msg::CompressedImage;
use std::{
    convert::TryFrom,
    fmt::Debug,
    future::Future,
    io::Write,
    sync::{Arc, Mutex, MutexGuard},
    thread::{self, JoinHandle},
};
mod opencv_canny;
use opencv_canny::canny_sharpness_function;

use utils::*;

struct Eye<T> {
    pos: T,
    final_pos: T,
    min_pos: T,
    max_pos: T,
    init: bool,
    current_zoom: T,
    compressed_img: Option<sensor_msgs::msg::CompressedImage>,
    focus_flag: bool,
}

impl<T: std::convert::From<i32>> Default for Eye<T> {
    fn default() -> Self {
        Eye {
            pos: T::from(0),
            final_pos: T::from(0),
            min_pos: T::from(0),
            max_pos: T::from(0),
            init: true,
            current_zoom: T::from(-1),
            compressed_img: None,
            focus_flag: false,
        }
    }
}

struct CameraFocus<T> {
    node: Node,
    eyes_info: Vec<Arc<Mutex<Eye<T>>>>,
    camera_subscriber_left: Arc<Mutex<Arc<Subscription<CompressedImage>>>>,
    camera_subscriber_right: Arc<Mutex<Arc<Subscription<CompressedImage>>>>,
    set_focus_state_service: Arc<Mutex<Arc<Service<SetFocusState>>>>,
    set_camera_zoom_focus_client: Arc<Mutex<Arc<Client<SetCameraZoomFocus>>>>,
    get_camera_zoom_focus_client: Arc<Mutex<Arc<Client<GetCameraZoomFocus>>>>,
    right_eye_thread: JoinHandle<()>,
    left_eye_thread: JoinHandle<()>,
}

// class CameraFocus(Node):
///Handle the autofocus of both reachy cameras in real time.
impl<T> CameraFocus<T>
where
    T: 'static
        // + From<isize>
        + std::convert::From<f64>
        + std::convert::From<i32>
        + std::ops::Mul<Output = T>
        + Copy
        + Debug
        + Default
        + std::marker::Send
        + std::ops::AddAssign
        + std::ops::Add<Output = T>
        + std::cmp::PartialOrd
        + std::ops::Sub<Output = T>
        + std::ops::Div<Output = T>
        + std::marker::Sync,
    f64: std::convert::From<T>,
{
    ///Set up variables shared between threads, publishers and clients.
    fn new(name_node: &str) -> Self {
        let context = rclrs::Context::new(std::env::args()).unwrap();
        let mut node = context.create_node(name_node).unwrap();
        // super().__init__("camera_focus")

        let mut eyes_info = amvec![Eye::default(), Eye::default()];

        // let mut logger = let mut get_logger();
        // let mut bridge = CvBridge();

        let current_side = SideOnly::Left;
        let am_current_eye = Arc::clone(&eyes_info[side_to_index(current_side)]);
        let camera_subscriber_left = am!(node
            .create_subscription::<CompressedImage, _>(
                "left_image",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: CompressedImage| {
                    Self::on_image_update(&am_current_eye, Side::Left(msg));
                },
            )
            .unwrap());

        let current_side = SideOnly::Right;
        let am_current_eye = Arc::clone(&eyes_info[side_to_index(current_side)]);
        let camera_subscriber_right = am!(node
            .create_subscription::<CompressedImage, _>(
                "right_image",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: CompressedImage| {
                    Self::on_image_update(&am_current_eye, Side::Right(msg));
                },
            )
            .unwrap());

        let left_side = SideOnly::Left;
        let am_left_eye = Arc::clone(&eyes_info[side_to_index(left_side)]);
        let right_side = SideOnly::Right;
        let am_right_eye = Arc::clone(&eyes_info[side_to_index(right_side)]);
        let set_focus_state_service = am!(node
            .create_service::<SetFocusState, _>(
                "set_focus_state",
                move |_request_header: &rclrs::rmw_request_id_t,
                      request: SetFocusState_Request|
                      -> SetFocusState_Response {
                    Self::set_focus_state_callback(&am_left_eye, &am_right_eye, request)
                },
            )
            .unwrap());

        let set_camera_zoom_focus_client = am!(node
            .create_client::<SetCameraZoomFocus>("set_camera_zoom_focus")
            .unwrap());

        let get_camera_zoom_focus_client = am!(node
            .create_client::<GetCameraZoomFocus>("get_camera_zoom_focus")
            .unwrap());

        // let mut right_eye_thread = threading.Thread(
        //     target = Self::focusing_algorithm,
        //     args = ("right_eye",),
        //     daemon = true,
        // );
        // let mut left_eye_thread = threading.Thread(
        //     target = Self::focusing_algorithm,
        //     args = ("left_eye",),
        //     daemon = true,
        // );
        let current_side = SideOnly::Right;
        let am_current_eye = Arc::clone(&eyes_info[side_to_index(current_side)]);
        let am_set_camera_zoom_focus_client = Arc::clone(&set_camera_zoom_focus_client);
        let am_get_camera_zoom_focus_client = Arc::clone(&get_camera_zoom_focus_client);
        let right_eye_thread = std::thread::spawn(move || {
            Self::focusing_algorithm(
                &am_current_eye,
                &am_set_camera_zoom_focus_client,
                &am_get_camera_zoom_focus_client,
                current_side,
            )
        });

        let current_side = SideOnly::Left;
        let am_current_eye = Arc::clone(&eyes_info[side_to_index(current_side)]);
        let am_set_camera_zoom_focus_client = Arc::clone(&set_camera_zoom_focus_client);
        let am_get_camera_zoom_focus_client = Arc::clone(&get_camera_zoom_focus_client);
        let left_eye_thread = std::thread::spawn(move || {
            Self::focusing_algorithm(
                &am_current_eye,
                &am_set_camera_zoom_focus_client,
                &am_get_camera_zoom_focus_client,
                current_side,
            )
        });

        CameraFocus {
            node,
            eyes_info,
            camera_subscriber_left,
            camera_subscriber_right,
            set_focus_state_service,
            set_camera_zoom_focus_client,
            get_camera_zoom_focus_client,
            right_eye_thread,
            left_eye_thread,
        }
    }
    // fn _wait_for(self, future: impl Future) {
    //     for _ in range(10000) {
    //         if future.done() {
    //             return future.result();
    //         }
    //         std::thread::sleep(std::time::Duration::from_millis(1));
    //     }
    // }

    ///Return range limitation regarding current zoom position.
    ///
    ///Args:
    ///    eye: either "left_eye" or "right_eye".
    ///
    /// need an Eye<T>
    fn compute_poses_maxima(am_eye: &Arc<Mutex<Eye<T>>>) {
        let mut current_eye;
        arc_mutex!(am_eye=>current_eye);
        current_eye.min_pos = T::from(max(
            500. - (f64::from(current_eye.current_zoom * T::from(0.01)).exp() + 25.) * 5.,
            0.,
        ));
        current_eye.max_pos = T::from(min(
            500. - (f64::from(current_eye.current_zoom * T::from(0.05) / T::from(6.)).exp() + 5.)
                * 5.,
            500.,
        ))
    }

    ///Compute the next position to reach regarding range limitations.
    ///
    ///Args:
    ///    eye: either "left_eye" or "right_eye"
    ///    step:step between the current position and the next desired,
    ///    can be positive as negative value
    ///
    /// need "self.eyes_info[eye]"
    fn compute_next_pose(eye: &Arc<Mutex<Eye<T>>>, step: impl Into<T>) -> T {
        let step_in = step.into();
        let mut current_eye;
        arc_mutex!(eye=>current_eye);

        if current_eye.min_pos < current_eye.pos + step_in
            && current_eye.pos + step_in < current_eye.max_pos
        {
            current_eye.pos += step_in
        } else if current_eye.pos + step_in >= current_eye.max_pos {
            current_eye.pos = current_eye.max_pos
        } else if current_eye.pos + step_in <= current_eye.min_pos {
            current_eye.pos = current_eye.min_pos
        }
        return current_eye.pos;
    }

    //     ///Return the shaprness of im through canny edge dectection algorithm.
    //     ///
    //     ///Args:
    //     ///    im: image used in canny edge detection algorithm
    // fn canny_sharpness_function(self, im){

    //     im = self.bridge.compressed_imgmsg_to_cv2(im)
    //     im = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
    //     im = cv.Canny(im, 50, 100)
    //     im_sum = cv.integral(im)
    //     return im_sum[-1][-1]/(im.shape[0]*im.shape[1])}

    ///Get data from image. Callback for "/"side"_image "subscriber.
    ///
    /// need "self.eyes_info[side + "_eye"]"
    /// "self.eyes_info["left_eye"]" "self.eyes_info["right_eye"]"
    fn on_image_update(eye: &Arc<Mutex<Eye<T>>>, msg: Side<CompressedImage>) {
        let mut current_eye;
        arc_mutex!(eye=>current_eye);
        match msg {
            Side::Left(compressed_image) => {
                current_eye.compressed_img = Some(compressed_image);
            }
            Side::Right(compressed_image) => {
                current_eye.compressed_img = Some(compressed_image);
            }
        };
        // self.eyes_info[side + "_eye"]["compressed_img"] = msg
    }
    /// need "self.eyes_info[eye]"
    fn set_focus_state_callback(
        am_left_eye: &Arc<Mutex<Eye<T>>>,
        am_righ_eye: &Arc<Mutex<Eye<T>>>,
        request: SetFocusState_Request,
        // response: SetFocusState_Response,
    ) -> SetFocusState_Response {
        let mut response = SetFocusState_Response::default();
        for (eye, state) in request.eye.iter().zip(request.state) {
            if !inside(eye, stvec!["left_eye", "right_eye"]) {
                // self.logger.warning("Invalid name sent to focus controller (must be in ("left_eye", "right_eye")).")
                response.success = false;
                return response;
            }
            match &eye as &str {
                "left_eye" => {
                    // let mut left_eye;
                    // arc_mutex!(am_left_eye=>left_eye);
                    ameis!(am_left_eye=>left_eye;{left_eye.focus_flag = state});
                    // left_eye.focus_flag = state;
                    if state {
                        println!("Starting autofocus on {eye}.");
                        ameis!(am_left_eye=>left_eye;{left_eye.current_zoom = T::from(-1)});
                        // left_eye.current_zoom = T::from(-1);
                        ameis!(am_left_eye=>left_eye;{left_eye.init = true});
                        // left_eye.init = true;
                        std::thread::sleep(std::time::Duration::from_millis(1000));
                    } else {
                        println!("Stopping autofocus on {eye}.");
                    }
                }
                "right_eye" => {
                    // let mut right_eye;
                    // arc_mutex!(am_righ_eye=>right_eye);
                    ameis!(am_righ_eye=>right_eye;{right_eye.focus_flag = state});
                    // right_eye.focus_flag = state;
                    if state {
                        println!("Starting autofocus on {eye}.");
                        ameis!(am_righ_eye=>right_eye;{right_eye.current_zoom = T::from(-1)});
                        // right_eye.current_zoom = T::from(-1);
                        ameis!(am_righ_eye=>right_eye;{right_eye.init = true});
                        // right_eye.init = true;
                        std::thread::sleep(std::time::Duration::from_millis(1000));
                    } else {
                        println!("Stopping autofocus on {eye}.");
                    }
                }
                _ => unreachable!(),
            }
        }
        response.success = true;
        return response;
    }

    ///Set the focus and/or zoom of a given camera using SetCameraZoomFocus service.
    fn send_request_set_camera_zoom_focus(
        set_camera_zoom_focus_client: &Arc<Mutex<Arc<Client<SetCameraZoomFocus>>>>,
        command: Vec<(SideOnly, ZoomOrFocus<T>)>,
    ) -> SetCameraZoomFocus_Response
    where
        f64: std::convert::From<T>,
    {
        // ) {
        let mut req = SetCameraZoomFocus_Request::default();
        for (e, zoom_or_focus) in command {
            let mut zoom_cmd_msg = ZoomCommand::default();
            zoom_cmd_msg.flag = true;
            match e {
                SideOnly::Left => match zoom_or_focus {
                    ZoomOrFocus::Zoom(value) => {
                        zoom_cmd_msg.value = f64::from(value) as u16;
                        req.left_zoom = zoom_cmd_msg;
                    }
                    ZoomOrFocus::Focus(value) => {
                        zoom_cmd_msg.value = f64::from(value) as u16;
                        req.left_focus = zoom_cmd_msg;
                    }
                },
                SideOnly::Right => match zoom_or_focus {
                    ZoomOrFocus::Zoom(value) => {
                        zoom_cmd_msg.value = f64::from(value) as u16;
                        req.right_zoom = zoom_cmd_msg;
                    }
                    ZoomOrFocus::Focus(value) => {
                        zoom_cmd_msg.value = f64::from(value) as u16;
                        req.right_focus = zoom_cmd_msg;
                    }
                },
            }
        }
        let client_set_camera_zoom_focus;
        arc_mutex!(set_camera_zoom_focus_client=>client_set_camera_zoom_focus);
        let result = client_set_camera_zoom_focus.call_async(req);
        return block_on(async { result.await.unwrap() });
    }

    /// Get the focus and zoom of both cameras.
    fn send_request_get_camera_zoom_focus(
        get_camera_zoom_focus_client: &Arc<Mutex<Arc<Client<GetCameraZoomFocus>>>>,
        am_current_eye: &Arc<Mutex<Eye<T>>>,
        side: &SideOnly,
    ) {
        let req = GetCameraZoomFocus_Request::default();

        // let client_get_camera_zoom_focus;
        // arc_mutex!(get_camera_zoom_focus_client=>client_get_camera_zoom_focus);
        // let result = client_get_camera_zoom_focus.call_async(req);
        // let to_return = futures::executor::block_on(async { result.await.unwrap() });
        // return to_return;
        let temp = Arc::clone(&am_current_eye);
        let side_in = side.clone();
        ameis!(get_camera_zoom_focus_client=> gczfc;{
            // println!("supposed to send a request");
            gczfc.async_send_request_with_callback(req, move |response:&GetCameraZoomFocus_Response| {
                // println!("got a response");
                match side_in {
                    SideOnly::Left => {
                        ameis!(temp=>current_eye;{current_eye.current_zoom = T::from(response.left_zoom as f64)});
                    }
                    SideOnly::Right => {
                        ameis!(temp=>current_eye;{current_eye.current_zoom = T::from(response.right_zoom as f64)});
                    }
                };
            }).unwrap();
        });
    }

    /// Perform autofocus on a given camera.
    ///
    /// Args:
    ///     eye: either "left_eye" or "right_eye".
    fn focusing_algorithm(
        am_eye: &Arc<Mutex<Eye<T>>>,
        sczfc: &Arc<Mutex<Arc<Client<SetCameraZoomFocus>>>>,
        gczfc: &Arc<Mutex<Arc<Client<GetCameraZoomFocus>>>>,
        side: SideOnly,
    ) {
        let mut max_res = T::from(0); // Best canny sharpness function result obtained
        let mut p_max = T::from(0); // focus position link to max_res
        let mut low_thresh = T::from(0); // lower noise tolerance threshold
        let mut up_thresh = T::from(0); // upper noise tolerance threshold
        let mut step = T::from(1); // moving step
        {
            let mut current_eye;
            arc_mutex!(am_eye=>current_eye);
            current_eye.init = true;
        }
        let mut first = true;
        let mut stop = T::from(0);
        let mut noise = T::from(0.4);
        let mut step = T::from(1);
        let eye_side = match side {
            SideOnly::Left => "left",
            SideOnly::Right => "right",
        };
        let eye_init = match side {
            SideOnly::Left => "left_eye",
            SideOnly::Right => "right_eye",
        };
        // time to let the everythong to be initialized
        std::thread::sleep(std::time::Duration::from_millis(5000));

        {
            let mut current_eye;
            arc_mutex!(am_eye=>current_eye);

            while current_eye.compressed_img == None {
                printol!("Waiting for an image from /{eye_side}_image...");
                std::thread::sleep(std::time::Duration::from_millis(5000));
            }
        }
        println!("Autofocus node for {eye_init} ready!");
        loop {
            std::thread::sleep(std::time::Duration::from_millis(50));
            if ameis!(am_eye => current_eye; {current_eye.focus_flag}) {
                let res = canny_sharpness_function(
                    ameis!(am_eye => current_eye; {&(&current_eye).compressed_img.clone().unwrap()}),
                );

                if ameis!(am_eye => current_eye; {current_eye.init}) {
                    while ameis!(am_eye => current_eye; {current_eye.current_zoom == T::from(-1)}) {
                        std::thread::sleep(std::time::Duration::from_millis(2000));
                        // match side {
                        //     SideOnly::Left => {
                        //         Self::send_request_get_camera_zoom_focus(&gczfc, &am_eye, &side);
                        //     }
                        //     SideOnly::Right => {
                        Self::send_request_get_camera_zoom_focus(&gczfc, &am_eye, &side);
                        //     }
                        // }
                        // ameis!(am_eye => current_eye; {current_eye.current_zoom = value});
                    }

                    if ameis!(am_eye => current_eye; {current_eye.current_zoom < T::from(100)}) {
                        noise = T::from(5)
                    }

                    first = true;
                    stop = T::from(0);
                    Self::compute_poses_maxima(&am_eye);
                    ameis!(am_eye => current_eye; {current_eye.pos = (&current_eye).min_pos.clone()});
                    max_res = T::from(0);
                    ameis!(am_eye => current_eye; {current_eye.init = false});
                    let order = ameis!(am_eye => current_eye; {current_eye.min_pos});
                    Self::send_request_set_camera_zoom_focus(
                        &sczfc,
                        vec![(side, ZoomOrFocus::Focus(order))],
                    );
                    std::thread::sleep(std::time::Duration::from_millis(2000));
                } else if stop == T::from(0) {
                    if res > max_res {
                        max_res = res;
                        ameis!(am_eye => current_eye; {p_max = current_eye.pos});
                    }

                    if first {
                        first = false;
                        low_thresh = res - noise;
                        up_thresh = res + noise;
                        Self::compute_next_pose(am_eye, step);
                    } else if res < low_thresh
                        || ameis!(am_eye => current_eye; {current_eye.pos == current_eye.max_pos})
                    {
                        // self.eyes_info[eye]["final_pos"] = p_max;
                        ameis!(am_eye => current_eye; {current_eye.final_pos = p_max});
                        stop = T::from(1);
                        let mut temp_pose = Self::compute_next_pose(&am_eye, -30);
                        Self::send_request_set_camera_zoom_focus(
                            &sczfc,
                            vec![(side, ZoomOrFocus::Focus(temp_pose))],
                        );

                        std::thread::sleep(std::time::Duration::from_millis(500));
                        let order = ameis!(am_eye => current_eye; {current_eye.final_pos});
                        Self::send_request_set_camera_zoom_focus(
                            &sczfc,
                            vec![(side, ZoomOrFocus::Focus(order))],
                        );
                        std::thread::sleep(std::time::Duration::from_millis(500));
                        ameis!(am_eye => current_eye; {
                            current_eye.pos = current_eye.final_pos;
                            current_eye.final_pos = T::from(-1);
                            current_eye.focus_flag = false;
                        });
                        println!("Finished autofocus on {eye_init}.");
                    } else if res > up_thresh {
                        low_thresh = res - noise;
                        up_thresh = res + noise;
                        step = T::from(1);
                        Self::compute_next_pose(am_eye, step);
                    } else {
                        if step == T::from(1) {
                            step = T::from(5)
                        }
                        let temp = Self::compute_next_pose(am_eye, step);
                        ameis!(am_eye => current_eye; {current_eye.pos = temp});
                    }
                    let order_zoom = ameis!(am_eye => current_eye; {current_eye.current_zoom});
                    let order_focus = ameis!(am_eye => current_eye; {current_eye.pos});
                    Self::send_request_set_camera_zoom_focus(
                        &sczfc,
                        vec![
                            (side, ZoomOrFocus::Zoom(order_zoom)),
                            (side, ZoomOrFocus::Focus(order_focus)),
                        ],
                    );
                    std::thread::sleep(std::time::Duration::from_millis(150))
                }
            } else {
                std::thread::sleep(std::time::Duration::from_millis(40));
            }
        }
    }
}

/// Create and launch CameraFocus Node.
///
/// If ctrl+c is pressed, node is destroyed.
fn main() {
    let camera_focus = CameraFocus::<f64>::new("camera_focus");

    rclrs::spin(&camera_focus.node)
        .map_err(|err| -> anyhow::Error { err.into() })
        .unwrap();
}
