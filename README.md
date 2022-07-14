# rust-reachy_camera_autofocus

Transcription of python packages to Rust using ros2-rust crate
==============

| Target | Status |
|----------|--------|
| **Ubuntu 20.04** | [![Build Status](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust.yml/badge.svg?branch=main)](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust.yml?branch=main) |


This project needs OpenCV to be built. If you don't have it run `install-opencv.sh`

```bash
sh install-opencv.sh
```

If you haven't already got the dependencies for ros2-rust package:

```bash
sudo apt install -y git libclang-dev python3-pip python3-vcstool
cargo install cargo-ament-build
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
```

With ROS2 Foxy installed, you can run those comands to get and build the workspace: 

```bash
mkdir -p workspace/src && cd workspace
git clone https://github.com/CoAuToSe/rust-reachy_camera_autofocus src/ros2_rust
vcs import src < src/ros2_rust/ros2_rust_foxy.repos
source /opt/ros/foxy/setup.bash
colcon build
source ./install/setup.sh
ros2 launch auto_focus camera_auto_focus.launch.xml

```