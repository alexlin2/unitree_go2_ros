# unitree_go2_ros
Unitree go2 ROS 1 Drivers implemented using Python WebRTC interface

This project enables a ROS interface that talks to the go2 through WebRTC and sends sport commands, and receives the camera and lidar data and publishes it out as ROS topics

## Features

joy_cmd_vel :white_check_mark: \
urdf + state publisher :white_check_mark: \
robot odom :white_check_mark: \
lidar pointcloud :white_check_mark: \
camera_stream :white_check_mark:


## Getting started

install ROS 1 with http://wiki.ros.org/noetic/Installation/Ubuntu

create a catkin workspace
```
mkdir -p ~/go2_catkin_ws/src
cd ~/go2_catkin_ws/src
git clone --recursive https://github.com/alexlin2/unitree_go2_ros.git
```

```
export GO2_TOKEN="your token"
export GO2_IP="robot ip"
```

## Usage
launch go2_base_node

```
roslaunch go2_driver go2.launch
```

In another terminal:

```
cd go2webrtc-rs
cargo build
./target/debug/go2webrtc-rc  --robot 192.168.12.1
python go2cv.py
```

Connect XBOX controller and have fun

# Thanks

Special thanks to tfoldi for his work on the WebRTC interface

# License 
