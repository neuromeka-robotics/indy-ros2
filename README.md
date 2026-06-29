# Indy ROS2

## Introduction

**Indy** is Neuromeka’s flagship cobot model we designed and manufactured. Guaranteeing workers’ safety based on innovative collision detection algorithms, Indy supports more intuitive direct teaching by impedance control as well as online and offline programming with the teach pendant app running on android tablets.

![Indy robot](.img/intro_img.png)

This repository contains ROS2 drivers for Indy7, Indy7V2, IndyRP2, IndyRP2V2, Indy12 and Indy12V2.

## Preparation

The following software needs to be installed:

- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

## Installation

### Install dependencies

```bash
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

### Switch to Cyclone DDS

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

Add this to ~/.bashrc to source it automatically

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Download the source code

```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/neuromeka-robotics/indy-ros2 -b jazzy-indyDCP3
cd ~/indy-ros2/
```

### Setup [Neuromeka Package](https://github.com/neuromeka-robotics/neuromeka-package) and build

```bash
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

```bash
deactivate
```

### Source the setup file

```bash
. install/setup.bash
```

### Setup when open new terminal

```bash
cd ~/indy-ros2/
source .venv/bin/activate
. install/setup.bash
```

## Usage

Use **indy_type** to choose the robot model.

Supported **indy_type** values:

```text
indy7, indy7_v2, indy7_v3, indy12, indy12_v2, indy12_v3,
indyrp2, indyrp2_v2, icon7l, icon3,
nuri3s, nuri4s, nuri7c, nuri12c, nuri20c, nuri30,
opti5, eir
```

Use **indy_eye** to enable the Indy Eye model.

Supported **indy_eye** values:

```text
indy7, indy7_v2, indy7_v3, indy12_v3, indyrp2, indyrp2_v2
```

To enable Indy Eye, add **indy_eye:=true** to the end of command.

Use **gripper_dh_ag95** to attach DH_AG95 grippers to EIR.

```bash
ros2 launch indy_description indy_display.launch.py indy_type:=eir gripper_dh_ag95:=true
```

If not specified, the default value will be indy7.

When used with a real robot, you need to provide an **indy_ip** value.

### Servoing Mode With Joy Controller

Tested with XBOX ONE S gamepad.

- Use Dpad to control joint 1 and joint 2.
- B and X control joint 4.
- Y and A control joint 3.
- Left joystick, right joystick, LB, RB, LT, RT control TCP.

#### Joy Controller On Real Robot

- Use 'LEFT_STICK_CLICK' to move Home, 'RIGHT_STICK_CLICK' to move Zero, 'XBOX' to Recover, 'HOME' to Start/Stop Teleop.

### Servoing Mode With Keyboard

#### Keyboard Common Use

- Use arrow keys and the '.' and ';' keys to Cartesian jog.
- Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame.
- Use 'N' 'M' ',' for the Task move UVW.
- Use 1|2|3|4|5|6|7 keys to joint jog. 'R' reverses the jogging direction.
- Use 'J' to select joint jog.
- Use 'T' to select twist.
- Use '-' '+' to adjust joint speed.
- Use '9' '0' to adjust task speed.
- 'Q' to quit.

#### Keyboard On Real Robot

- Use 'H' to move Home, 'Z' to move Zero, 'S' to Recover, 'P' to stop Teleop.

### Start Indy description

```bash
ros2 launch indy_description indy_display.launch.py indy_type:=indy7
```

![Indy7 description view](.img/description_indy7.gif)

### Simulation Robot

#### Start Indy Robot In Simulation

```bash
ros2 launch indy_gazebo indy_gazebo.launch.py indy_type:=indy7
```

For EIR:

```bash
ros2 launch indy_gazebo indy_gazebo.launch.py indy_type:=eir
```

#### Start MoveIt In Simulation

```bash
ros2 launch indy_moveit indy_moveit_gazebo.launch.py indy_type:=indy7
```

For EIR dual-arm MoveIt, select either `left_arm` or `right_arm` in the MotionPlanning panel:

```bash
ros2 launch indy_moveit indy_moveit_gazebo.launch.py indy_type:=eir
```

#### Start Servoing In Simulation

```bash
ros2 launch indy_moveit indy_moveit_gazebo.launch.py indy_type:=indy7 servo_mode:=true
```

Start keyboard or controller

```bash
ros2 run indy_driver servo_keyboard_input.py --ros-args -p is_sim:=true
```

```bash
ros2 run indy_driver servo_joy_input.py --ros-args -p is_sim:=true
```

### Real Robot

#### Start Real Robot

```bash
ros2 launch indy_driver indy_bringup.launch.py indy_type:=indy7 indy_ip:=192.168.xxx.xxx
```

#### Start MoveIt With Real Robot

```bash
ros2 launch indy_moveit indy_moveit_real_robot.launch.py indy_type:=indy7 indy_ip:=192.168.xxx.xxx
```

Enable the RealSense pipeline:

```bash
ros2 launch indy_moveit indy_moveit_real_robot.launch.py \
    indy_type:=indy7 indy_ip:=192.168.xxx.xxx enable_realsense:=true
```

Use the optional arguments `realsense_namespace`, `camera_parent_frame`, `camera_link_frame`, and `camera_pose_*` to match your mount point.

#### Start Servoing With Real Robot

```bash
ros2 launch indy_moveit indy_moveit_real_robot.launch.py indy_type:=indy7 indy_ip:=192.168.xxx.xxx servo_mode:=true
```

Start keyboard or controller

```bash
ros2 run indy_driver servo_keyboard_input.py --ros-args -p is_sim:=false
```

```bash
ros2 run indy_driver servo_joy_input.py --ros-args -p is_sim:=false
```
