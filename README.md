# Indy ROS2

## Introduction

**Indy** is Neuromeka’s flagship cobot model we designed and manufactured. Guaranteeing workers’ safety based on innovative collision detection algorithms, Indy supports more intuitive direct teaching by impedance control as well as online and offline programming with the teach pendant app running on android tablets.

<center><img src=".img/intro_img.png" width="400" heigh="400"/></center> 


This repository contains ROS2 drivers for Indy7, Indy7V2, IndyRP2, IndyRP2V2 and Indy12.


## Preparation

The following software needs to be installed:
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Neuromeka Package](https://github.com/neuromeka-robotics/neuromeka-package)
    ```
    pip3 install neuromeka
    pip3 install --upgrade neuromeka
    ```

## Installation

### Install dependencies
```
sudo apt install python3-rosdep
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update
sudo apt update
sudo apt install rospack-tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
sudo apt install python3-vcstool

sudo apt install -y ros-humble-xacro \
ros-humble-moveit \
ros-humble-moveit-servo \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-moveit-ros-move-group \
ros-humble-moveit-planners-ompl \
ros-humble-moveit-kinematics \
ros-humble-gazebo-ros \
ros-humble-gazebo-ros2-control \
ros-humble-controller-manager \
ros-humble-joint-state-broadcaster \
ros-humble-joint-state-publisher-gui \
ros-humble-joint-trajectory-controller \
ros-humble-moveit-ros-perception \
ros-humble-rviz-visual-tools \
ros-humble-moveit-visual-tools \
ros-humble-moveit-resources
```

**Switch to Cyclone DDS**
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
Add this to ~/.bashrc to source it automatically
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Create workspace

Create workspace and download the source code

```
cd
mkdir -p indy-ros2/src
cd ~/indy-ros2/src
git clone <this repository url>
```

Build the source code

```
cd ~/indy-ros2/
colcon build
```

### Source the setup file
```
. install/setup.bash
```

## Usage

Use **indy_type** to choose specific robot **(indy7, indy7_v2, indy12, indyrp2, indyrp2_v2)**.\
Use **indy_eye** to enable Indy Eye model **(support indy7, indyrp2, indy7_v2, indyrp2_v2)**.\
To enable Indy Eye, add **indy_eye:=true** to the end of command

If not specified, the default value will be indy7.\
When used with a real robot, you need to provide an **indy_ip** value.

**Servoing mode with Joy Controller (tested with XBOX ONE S gamepad)**\
Using Dpad to control joint 1 and joint 2.\
B and X control joint 4\
Y and A control joint 3\
Left joystick, right joystick, LB, RB, LT, RT to control TCP.\
*On Real Robot*\
Use 'LEFT_STICK_CLICK' to move Home, 'RIGHT_STICK_CLICK' to move Zero, 'XBOX' to Recover, 'HOME' to Start/Stop Teleop


**Servoing mode with Keyboard**\
*Common Use*\
Use arrow keys and the '.' and ';' keys to Cartesian jog\
Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame\
Use 'N' 'M' ',' for the Task move UVW\
Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.\
Use '-' '+' to adjust joint speed\
Use '9' '0' to adjust task speed\
'Q' to quit.\
*On Real Robot*\
Use 'H' to move Home, 'Z' to move Zero, 'S' to Recover, 'P' to stop Teleop\

**Generate your URDF files**\
You can generate your URDF files using **generate_all_urdfs.sh** file in **indy_description/urdf** folder
```bash
cd </..path../..to../indy_description/urdf/>
sudo chmod +x generate_all_urdfs.sh
./generate_all_urdfs.sh
```

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


## Docker Setup Instructions for ROS2 Humble

### Install Docker
If Docker is not installed on your system, follow these steps:
```
sudo apt update \
&& sudo apt install -y docker.io \
&& sudo systemctl start docker \
&& sudo systemctl enable docker
```
Navigate to the Dockerfile Directory
```
cd ~/indy-ros2/docker
```
Build the Docker Image
```
sudo docker build -t ros2_humble_neuromeka:humble-indyDCP3 .
```
Verify the docker Image, if docker is successfully updated, you can find the docker image.
```
sudo docker images
```

Prepare for GUI Applications (Rviz, Gazebo) using your monitor:
```
xhost +local:docker
```
Create and start a Docker container named ros2_humble_neuromeka_container, enabling GUI support:
```
sudo docker run -it --name \
ros2_humble_neuromeka_container \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
ros2_humble_neuromeka:humble-indyDCP3
```
Access the Docker Container in Another Terminal
```
sudo docker exec -it ros2_humble_neuromeka_container bash
```
Your Docker environment is now configured.
