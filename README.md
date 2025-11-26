# Indy ROS2

## Introduction

**Indy** is Neuromeka’s flagship cobot model we designed and manufactured. Guaranteeing workers’ safety based on innovative collision detection algorithms, Indy supports more intuitive direct teaching by impedance control as well as online and offline programming with the teach pendant app running on android tablets.

<center><img src=".img/intro_img.png" width="400" heigh="400"/></center> 


This repository contains ROS2 drivers for Indy7, Indy7V2, IndyRP2, IndyRP2V2, Indy12 and Indy12V2.


## Preparation

The following software needs to be installed:
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

## Installation

### Install dependencies
```
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
sudo apt update

sudo apt install -y ros-jazzy-ament-cmake
ros-jazzy-xacro \
ros-jazzy-ros-base \
ros-jazzy-moveit \
ros-jazzy-moveit-servo \
ros-jazzy-moveit-visual-tools \
ros-jazzy-moveit-resources \
ros-jazzy-moveit-ros-move-group \
ros-jazzy-moveit-planners-ompl \
ros-jazzy-moveit-kinematics \
ros-jazzy-moveit-ros-perception \
ros-jazzy-ros2-control \
ros-jazzy-ros2-controllers \
ros-jazzy-controller-manager \
ros-jazzy-joint-state-broadcaster \
ros-jazzy-joint-state-publisher-gui \
ros-jazzy-joint-trajectory-controller \
ros-jazzy-rviz-visual-tools \
ros-jazzy-geometric-shapes \
ros-jazzy-gz-ros2-control \
ros-jazzy-ros-gz \
ros-jazzy-realsense2-camera \
ros-jazzy-realsense2-description \
ros-jazzy-librealsense2*
```

**Switch to Cyclone DDS**
```
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```
Add this to ~/.bashrc to source it automatically
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Download the source code

```
source /opt/ros/jazzy/setup.bash
git clone https://github.com/neuromeka-robotics/indy-ros2 -b jazzy-indyDCP3
cd ~/indy-ros2/
```

### Setup [Neuromeka Package](https://github.com/neuromeka-robotics/neuromeka-package) and build

```
sudo apt install python3-venv
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install catkin_pkg empy lark-parser
pip install pyyaml jinja2 typeguard neuromeka
rosdep install --from-paths src --ignore-src -r -y 
colcon build
```

To deactivate the venv

```
deactivate
```

### Source the setup file
```
. install/setup.bash
```

### Setup when open new terminal
```
cd ~/indy-ros2/
source .venv/bin/activate
. install/setup.bash
```

## Usage

Use **indy_type** to choose specific robot **(indy7, indy7_v2, indy12, indyrp2, indyrp2_v2)**.
Use **indy_eye** to enable Indy Eye model **(support indy7, indyrp2)**.
To enable Indy Eye, add **indy_eye:=true** to the end of command

If not specified, the default value will be indy7.

When used with a real robot, you need to provide an **indy_ip** value.

**Servoing mode with Joy Controller (tested with XBOX ONE S gamepad)**
Using Dpad to control joint 1 and joint 2.
B and X control joint 4
Y and A control joint 3
Left joystick, right joystick, LB, RB, LT, RT to control TCP.
*On Real Robot*
Use 'LEFT_STICK_CLICK' to move Home, 'RIGHT_STICK_CLICK' to move Zero, 'XBOX' to Recover, 'HOME' to Start/Stop Teleop


**Servoing mode with Keyboard**
*Common Use*
Use arrow keys and the '.' and ';' keys to Cartesian jog
Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame
Use 'N' 'M' ',' for the Task move UVW
Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.
Use 'J' to select joint jog
Use 'T' to select twist
Use '-' '+' to adjust joint speed
Use '9' '0' to adjust task speed
'Q' to quit.
*On Real Robot*
Use 'H' to move Home, 'Z' to move Zero, 'S' to Recover, 'P' to stop Teleop


### Start Indy description

```
ros2 launch indy_description indy_display.launch.py indy_type:=indy7
```

![](.img/description_indy7.gif)


### Simulation Robot

**Start Indy Robot**

```
ros2 launch indy_gazebo indy_gazebo.launch.py indy_type:=indy7
```

**Start Indy with MoveIt**

```
ros2 launch indy_moveit indy_moveit_gazebo.launch.py indy_type:=indy7
```

**Start Indy with Servoing**

```
ros2 launch indy_moveit indy_moveit_gazebo.launch.py indy_type:=indy7 servo_mode:=true
```

Start keyboard or controller

```
ros2 run indy_driver servo_keyboard_input.py --ros-args -p is_sim:=true
```
```
ros2 run indy_driver servo_joy_input.py --ros-args -p is_sim:=true
```

### Real Robot

**Start Indy Robot**

```
ros2 launch indy_driver indy_bringup.launch.py indy_type:=indy7 indy_ip:=192.168.xxx.xxx
```

**Start Indy with MoveIt**

```
ros2 launch indy_moveit indy_moveit_real_robot.launch.py indy_type:=indy7 indy_ip:=192.168.xxx.xxx
```

Enable the RealSense pipeline:

```
ros2 launch indy_moveit indy_moveit_real_robot.launch.py \
	indy_type:=indy7 indy_ip:=192.168.xxx.xxx enable_realsense:=true
```

Use the optional arguments `realsense_namespace`, `camera_parent_frame`, `camera_link_frame`, and `camera_pose_*` to match your mount point.

**Start Indy with Servoing**

```
ros2 launch indy_moveit indy_moveit_real_robot.launch.py indy_type:=indy7 indy_ip:=192.168.xxx.xxx servo_mode:=true
```

Start keyboard or controller

```
ros2 run indy_driver servo_keyboard_input.py --ros-args -p is_sim:=false
```
```
ros2 run indy_driver servo_joy_input.py --ros-args -p is_sim:=false
```
