//! Service node to manage zoom for cameras.;
// import time;
use std::{
    sync::{Arc, Mutex},
    time,
};

// import rclpy;
// from rclpy.node import Node;
use rclrs::{self, Client, Node, Service, Subscription};

// from reachy_msgs.srv import GetCameraZoomLevel, SetCameraZoomLevel;
// from reachy_msgs.srv import GetCameraZoomSpeed, SetCameraZoomSpeed;
// from reachy_msgs.srv import GetCameraZoomFocus, SetCameraZoomFocus;
// from reachy_msgs.srv import SetFocusState;
use reachy_msgs::srv::{
    GetCameraZoomFocus, GetCameraZoomFocus_Request, GetCameraZoomFocus_Response,
    GetCameraZoomLevel, GetCameraZoomLevel_Request, GetCameraZoomLevel_Response,
    GetCameraZoomSpeed, GetCameraZoomSpeed_Request, GetCameraZoomSpeed_Response,
    SetCameraZoomFocus, SetCameraZoomFocus_Request, SetCameraZoomFocus_Response,
    SetCameraZoomLevel, SetCameraZoomLevel_Request, SetCameraZoomLevel_Response,
    SetCameraZoomSpeed, SetCameraZoomSpeed_Request, SetCameraZoomSpeed_Response, SetFocusState,
    SetFocusState_Request, SetFocusState_Response,
};

// from zoom_kurokesu import ZoomController;
// use zoom_kurokesu::ZoomController;

// #[macro_use]
// #[path = "zoom_kurokesu.rs"]
// mod zoom_kurokesu;
use zoom_kurokesu_rust::*;

use utils::*;
// #[macro_use]
// mod personal_macro;
// use personal_macro::*;

// mod side_zoom_focus;
// use side_zoom_focus::*;

///Run main loop.
fn main() {
    let zoom_controller_service = ZoomControllerService::default();
    zoom_controller_service.start_autofocus();
    println!("starting spinning");
    rclrs::spin(&zoom_controller_service.node)
        .map_err(|err| -> anyhow::Error { err.into() })
        .unwrap();
}

/// Represent a camera.  
#[derive(Debug, Default, Clone)]
struct Zoom<T> {
    zoom: T,
    focus: T,
    speed: T,
    zoom_level: String,
}

impl<T: Copy> Zoom<T> {
    fn new(zae: Side<ZoomFocus<T>>, default_zoom_speed: T, default_zoom_level: String) -> Self {
        Zoom {
            zoom: zae.zoom_value().clone(),
            focus: zae.focus_value().clone(),
            speed: default_zoom_speed,
            zoom_level: default_zoom_level,
        }
    }
}

struct ZoomControllerService {
    node: Node,
    get_camera_zoom_level: Arc<Mutex<Arc<Service<GetCameraZoomLevel>>>>,
    set_camera_zoom_level: Arc<Mutex<Arc<Service<SetCameraZoomLevel>>>>,
    get_camera_zoom_speed: Arc<Mutex<Arc<Service<GetCameraZoomSpeed>>>>,
    set_camera_zoom_speed: Arc<Mutex<Arc<Service<SetCameraZoomSpeed>>>>,
    get_camera_zoom_focus: Arc<Mutex<Arc<Service<GetCameraZoomFocus>>>>,
    set_camera_zoom_focus: Arc<Mutex<Arc<Service<SetCameraZoomFocus>>>>,
    set_focus_state: Arc<Mutex<Arc<Client<SetFocusState>>>>,
}

impl Default for ZoomControllerService {
    fn default() -> Self {
        Self::new(
            10000,
            "inter".to_string(),
            "/dev/ttyACM0",
            115200,
            10,
            10_000,
        )
    }
}

///Main node creating the zoom services for cameras.;
impl ZoomControllerService {
    ///Set up the node and create the services.;
    fn new(
        default_zoom_speed: isize,
        default_zoom_level: String,
        port: &str,
        baudrate: isize,
        timeout: isize,
        speed: isize,
    ) -> Self {
        // super().__init__("camera_zoom_controller_service");
        // logger = get_logger();

        let context = rclrs::Context::new(std::env::args()).unwrap();
        let mut node = context
            .create_node("camera_zoom_controller_service")
            .unwrap();

        let def_zoom_lvl: ZoomLevel = ZoomLevel::try_from(default_zoom_level.clone()).unwrap();
        let controller = am!(ZoomController::new(port, baudrate, timeout, speed));

        ameis!(controller=>c;{c.set_speed(default_zoom_speed).unwrap()});

        for side in vec![SideOnly::Left, SideOnly::Right] {
            ameis!(controller=>c;{c.homing(side)});
            ameis!(controller=>c;{c.set_zoom_level(&side, &def_zoom_lvl)});
        }

        let current_zoom_info = vec![
            Side::Left(am!(Zoom::new(
                Side::Left(def_zoom_lvl).into(),
                default_zoom_speed,
                default_zoom_level.clone(),
            ))),
            Side::Right(am!(Zoom::new(
                Side::Right(def_zoom_lvl).into(),
                default_zoom_speed,
                default_zoom_level.clone(),
            ))),
        ];

        let am_left_zoom = Arc::clone(&current_zoom_info[0].inner());
        let am_right_zoom = Arc::clone(&current_zoom_info[1].inner());
        let get_camera_zoom_level = am!(node
            .create_service::<GetCameraZoomLevel, _>(
                "get_camera_zoom_level",
                move |_request_header, request| {
                    Self::get_zoom_level_callback(&am_left_zoom, &am_right_zoom, request)
                },
            )
            .unwrap());
        // self.logger.info(f"Launching "{get_command_service.srv_name}" service.");
        println!("Launching \"{}\" service.", "get_camera_zoom_level");

        let am_controller = Arc::clone(&controller);
        let am_left_zoom = Arc::clone(&current_zoom_info[0].inner());
        let am_right_zoom = Arc::clone(&current_zoom_info[1].inner());
        let set_camera_zoom_level = am!(node
            .create_service::<SetCameraZoomLevel, _>(
                "set_camera_zoom_level",
                move |_request_header, request| {
                    Self::set_zoom_command_callback(
                        &am_controller,
                        &am_left_zoom,
                        &am_right_zoom,
                        request,
                        Default::default(),
                    )
                },
            )
            .unwrap());
        // self.logger.info(f"Launching "{set_command_service.srv_name}" service.");
        println!("Launching \"{}\" service.", "set_camera_zoom_level");

        let am_left_zoom = Arc::clone(&current_zoom_info[0].inner());
        let am_right_zoom = Arc::clone(&current_zoom_info[1].inner());
        let get_camera_zoom_speed = am!(node
            .create_service::<GetCameraZoomSpeed, _>(
                "get_camera_zoom_speed",
                move |_request_header, request| {
                    Self::get_zoom_speed_callback(
                        &am_left_zoom,
                        &am_right_zoom,
                        request,
                        Default::default(),
                    )
                },
            )
            .unwrap());

        let am_controller = Arc::clone(&controller);
        let am_left_zoom = Arc::clone(&current_zoom_info[0].inner());
        let am_right_zoom = Arc::clone(&current_zoom_info[1].inner());
        let set_camera_zoom_speed = am!(node
            .create_service::<SetCameraZoomSpeed, _>(
                "set_camera_zoom_speed",
                move |_request_header, request| {
                    Self::set_zoom_speed_callback(
                        &am_controller,
                        &am_left_zoom,
                        &am_right_zoom,
                        request,
                        Default::default(),
                    )
                },
            )
            .unwrap());
        // let mut logger.info(f"Launching "{set_speed_service.srv_name}" service.");
        println!("Launching \"{}\" service.", "set_camera_zoom_speed");

        let am_left_zoom = Arc::clone(&current_zoom_info[0].inner());
        let am_right_zoom = Arc::clone(&current_zoom_info[1].inner());
        let get_camera_zoom_focus = am!(node
            .create_service::<GetCameraZoomFocus, _>(
                "get_camera_zoom_focus",
                move |_request_header, request| {
                    // println!("received a request to send of get_camera_zoom_focus");
                    Self::get_camera_zoom_focus_callback(
                        &am_left_zoom,
                        &am_right_zoom,
                        request,
                        Default::default(),
                    )
                },
            )
            .unwrap());

        let am_controller = Arc::clone(&controller);
        let am_left_zoom = Arc::clone(&current_zoom_info[0].inner());
        let am_right_zoom = Arc::clone(&current_zoom_info[1].inner());
        let set_camera_zoom_focus = am!(node
            .create_service::<SetCameraZoomFocus, _>(
                "set_camera_zoom_focus",
                move |_request_header, request| {
                    Self::set_camera_zoom_focus_callback(
                        &am_controller,
                        &am_left_zoom,
                        &am_right_zoom,
                        request,
                        Default::default(),
                    )
                },
            )
            .unwrap());

        let set_focus_state = am!(node
            .create_client::<SetFocusState>("set_focus_state")
            .unwrap());

        // let mut logger.info("Node ready!");
        println!("Node ready!");
        ZoomControllerService {
            node,
            get_camera_zoom_level,
            set_camera_zoom_level,
            get_camera_zoom_speed,
            set_camera_zoom_speed,
            get_camera_zoom_focus,
            set_camera_zoom_focus,
            set_focus_state,
        }
    }
    /// Get the current camera zoom level.;
    fn get_zoom_level_callback(
        am_left_zoom: &Arc<Mutex<Zoom<isize>>>,
        am_right_zoom: &Arc<Mutex<Zoom<isize>>>,
        request: GetCameraZoomLevel_Request,
    ) -> GetCameraZoomLevel_Response {
        let mut response = GetCameraZoomLevel_Response::default();
        if !inside(&request.name, stvec!["left_eye", "right_eye"]) {
            eprintln!(
                r#"Invalid name sent to zoom controller (must be in ("left_eye", "right_eye"))."#
            );
            return response;
        }
        let eye_side = match &request.name as &str {
            "left_eye" => SideOnly::Left,
            "right_eye" => SideOnly::Right,
            &_ => panic!("wrong set_zoom_command"),
        };
        match eye_side {
            SideOnly::Left => {
                ameis!(am_left_zoom=>current_zoom_info;{response.zoom_level = current_zoom_info.zoom_level.clone()});
            }
            SideOnly::Right => {
                ameis!(am_right_zoom=>current_zoom_info;{response.zoom_level = current_zoom_info.zoom_level.clone()});
            }
        };
        return response;
    }

    /// Handle set_camera_zoom_level request.
    fn set_zoom_command_callback(
        // self,
        am_controller: &Arc<Mutex<ZoomController>>,
        am_left_zoom: &Arc<Mutex<Zoom<isize>>>,
        am_right_zoom: &Arc<Mutex<Zoom<isize>>>,
        request: SetCameraZoomLevel_Request,
        mut response: SetCameraZoomLevel_Response,
    ) -> SetCameraZoomLevel_Response {
        // try{
        //     eye_side = {
        //         "left_eye": "left",
        //         "right_eye": "right",
        //     }[request.name]};
        // except KeyError{
        //     eprintln!(r#"Invalid name sent to zoom controller (must be in ("left_eye", "right_eye"))."#);
        //     response.success = false;
        //     return response};
        let eye_side = match &request.name as &str {
            "left_eye" => SideOnly::Left,
            "right_eye" => SideOnly::Right,
            &_ => panic!("wrong set_zoom_command"),
        };
        if request.zoom_level == "homing" {
            let mut controller;
            arc_mutex!(am_controller=>controller);
            controller.homing(eye_side);
            match eye_side {
                SideOnly::Left => {
                    ameis!(am_left_zoom=>current_zoom; {
                        current_zoom.zoom_level = st!("zero")});
                }
                SideOnly::Right => {
                    ameis!(am_right_zoom=>current_zoom; {                        current_zoom.zoom_level = st!("zero")});
                }
            };
        } else if inside(&request.zoom_level, stvec!("in", "out", "inter")) {
            let zoom_level = ZoomLevel::try_from(request.zoom_level.clone()).unwrap();
            let temp_from: Side<ZoomFocus<isize>> = match eye_side {
                SideOnly::Left => Side::Left(zoom_level).into(),
                SideOnly::Right => Side::Right(zoom_level).into(),
            };

            let mut controller;
            arc_mutex!(am_controller=>controller);

            controller.set_zoom_level(&eye_side, &zoom_level);

            match eye_side {
                SideOnly::Left => {
                    ameis!(am_left_zoom=>current_zoom; {
                        current_zoom.zoom_level = request.zoom_level;
                        current_zoom.zoom = *temp_from.zoom_value();
                    });
                }
                SideOnly::Right => {
                    ameis!(am_right_zoom=>current_zoom; {
                        current_zoom.zoom_level = request.zoom_level;
                        current_zoom.zoom = *temp_from.zoom_value();
                    });
                }
            };
            // current_zoom.zoom_level = request.zoom_level;
            // current_zoom.zoom =
            //     (controller.zoom_pos[eye_side][request.zoom_level]["zoom"]) as isize;
            // current_zoom.zoom = zoom_level.zoom();
            // .into();
            // current_zoom.zoom = *temp_from.zoom_value();
        } else {
            eprintln!(
                r#"Invalid command sent to zoom controller (must be in ("homing", "in", "out" or "inter"))."#
            );
            response.success = false;
            return response;
        };

        response.success = true;
        return response;
    }
    /// Get the current camera zoom speed.
    fn get_zoom_speed_callback(
        // self,
        am_left_zoom: &Arc<Mutex<Zoom<isize>>>,
        am_right_zoom: &Arc<Mutex<Zoom<isize>>>,
        request: GetCameraZoomSpeed_Request,
        mut response: GetCameraZoomSpeed_Response,
    ) -> GetCameraZoomSpeed_Response {
        if !inside(&request.name, stvec!["left_eye", "right_eye"]) {
            eprintln!(
                r#"Invalid name sent to zoom controller (must be in ("left_eye", "right_eye"))."#
            );
            return response;
        };

        let eye_side = match &request.name as &str {
            "left_eye" => SideOnly::Left,
            "right_eye" => SideOnly::Right,
            &_ => panic!("wrong set_zoom_command"),
        };
        match eye_side {
            SideOnly::Left => {
                ameis!(am_left_zoom=>current_zoom;{response.speed = current_zoom.speed as u16;});
            }
            SideOnly::Right => {
                ameis!(am_right_zoom=>current_zoom;{response.speed = current_zoom.speed as u16;});
            }
        }
        // let mut current_zoom;
        // arc_mutex!(am_current_zoom_info=>current_zoom);
        // response.speed = current_zoom.speed as u16;
        // response.speed = current_zoom_info[request.name]["speed"];
        return response;
    }

    /// Handle set_camera_zoom_speed request.
    fn set_zoom_speed_callback(
        // self,
        am_controller: &Arc<Mutex<ZoomController>>,
        am_left_zoom: &Arc<Mutex<Zoom<isize>>>,
        am_right_zoom: &Arc<Mutex<Zoom<isize>>>,
        request: SetCameraZoomSpeed_Request,
        mut response: SetCameraZoomSpeed_Response,
    ) -> SetCameraZoomSpeed_Response {
        if !inside(&request.name, stvec!["left_eye", "right_eye"]) {
            eprintln!(
                r#"Invalid name sent to zoom controller (must be in ("left_eye", "right_eye"))."#
            );
            response.success = false;
            return response;
        }
        {
            let mut controller;
            arc_mutex!(am_controller=>controller);
            controller.set_speed(request.speed as isize).unwrap();
        }

        let eye_side = match &request.name as &str {
            "left_eye" => SideOnly::Left,
            "right_eye" => SideOnly::Right,
            &_ => panic!("wrong set_zoom_command"),
        };
        match eye_side {
            SideOnly::Left => {
                ameis!(am_left_zoom=>current_zoom;{current_zoom.speed = request.speed as isize;});
            }
            SideOnly::Right => {
                ameis!(am_right_zoom=>current_zoom;{current_zoom.speed = request.speed as isize;});
            }
        }

        response.success = true;
        return response;
    }

    /// Handle get_camera_zoom_focus callback.
    fn get_camera_zoom_focus_callback(
        // self,
        am_left_zoom: &Arc<Mutex<Zoom<isize>>>,
        am_right_zoom: &Arc<Mutex<Zoom<isize>>>,
        _request: GetCameraZoomFocus_Request,
        mut response: GetCameraZoomFocus_Response,
    ) -> GetCameraZoomFocus_Response {
        // println!("creating response");
        {
            let left_zoom;
            arc_mutex!(am_left_zoom=>left_zoom);
            response.left_focus = left_zoom.focus as u16;
            response.left_zoom = left_zoom.zoom as u16;
        }
        {
            let right_zoom;
            arc_mutex!(am_right_zoom=>right_zoom);
            response.right_focus = right_zoom.focus as u16;
            response.right_zoom = right_zoom.zoom as u16;
        }
        // println!("sending response");
        return response;
    }
    ///Handle set_camera_zoom_focus callback.;
    fn set_camera_zoom_focus_callback(
        // self,
        am_controller: &Arc<Mutex<ZoomController>>,
        am_left_zoom: &Arc<Mutex<Zoom<isize>>>,
        am_right_zoom: &Arc<Mutex<Zoom<isize>>>,
        request: SetCameraZoomFocus_Request,
        mut response: SetCameraZoomFocus_Response,
    ) -> SetCameraZoomFocus_Response {
        // command = {"left": {}, "right": {}};
        let mut vec_cmds = vec![];
        {
            let cmd = request.left_zoom;
            if cmd.flag {
                // let side = "left";
                // let cmd_type = "zoom";
                vec_cmds.push(Side::Left(ZoomOrFocus::Zoom(cmd.value)));
                let mut left_zoom;
                arc_mutex!(am_left_zoom=>left_zoom);
                left_zoom.zoom = cmd.value as isize
            }
        }
        {
            let cmd = request.left_focus;
            if cmd.flag {
                // let side = "left";
                // let cmd_type = "focus";
                vec_cmds.push(Side::Left(ZoomOrFocus::Focus(cmd.value)));
                let mut left_zoom;
                arc_mutex!(am_left_zoom=>left_zoom);
                left_zoom.focus = cmd.value as isize
            }
        }
        {
            let cmd = request.right_zoom;
            if cmd.flag {
                // let side = "right";
                // let cmd_type = "zoom";
                vec_cmds.push(Side::Right(ZoomOrFocus::Zoom(cmd.value)));
                let mut right_zoom;
                arc_mutex!(am_right_zoom=>right_zoom);
                right_zoom.zoom = cmd.value as isize
            }
        }
        {
            let cmd = request.right_focus;
            if cmd.flag {
                // let side = "right";
                // let cmd_type = "focus";
                vec_cmds.push(Side::Right(ZoomOrFocus::Focus(cmd.value)));
                let mut right_zoom;
                arc_mutex!(am_right_zoom=>right_zoom);
                right_zoom.focus = cmd.value as isize
            }
        }
        // for cmd_name in list(request.get_fields_and_field_types().keys()){
        //     cmd = getattr(request, cmd_name);
        //     if cmd.flag{
        //         side, cmd_type = cmd_name.split("_");
        //         command[side][cmd_type] = cmd.value;
        //         current_zoom_info[side+"_eye"][cmd_type] = cmd.value}};

        let mut controller;
        arc_mutex!(am_controller=>controller);
        controller._send_custom_command(vec_cmds);
        response.success = true;
        return response;
    }
    ///Call set_focus_state service.
    fn start_autofocus(&self) {
        let am_set_focus_state: &Arc<Mutex<Arc<Client<SetFocusState>>>> = &self.set_focus_state;
        let mut req = SetFocusState_Request::default();
        req.eye = stvec!["left_eye", "right_eye"];
        req.state = vec![true, true];

        let set_focus_state;
        arc_mutex!(am_set_focus_state=>set_focus_state);
        // let mut changed_focus = false;
        // futures::executor::block_on(async {
        //     changed_focus = set_focus_state.call_async(req).await.unwrap().success
        // });
        // if changed_focus {
        //     println!("The focus state was changed")
        // } else {
        //     println!("!!!The focus state didn't changed!!!")
        // }
        set_focus_state
            .async_send_request_with_callback(req, |response: &SetFocusState_Response| {
                if response.success {
                    println!("The focus state was changed")
                } else {
                    println!("!!!The focus state didn't changed!!!")
                }
            })
            .unwrap();
        println!("Supposed to start autofocus");
        // std::thread::sleep(std::time::Duration::from_secs(1));
    }
}
