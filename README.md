## Installation

### Install Ros Foxy

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

sudo apt install -y ros-foxy-xacro \
ros-foxy-moveit \
ros-foxy-moveit-servo \
ros-foxy-ros2-control \
ros-foxy-ros2-controllers \
ros-foxy-moveit-ros-move-group \
ros-foxy-moveit-planners-ompl \
ros-foxy-moveit-kinematics \
ros-foxy-gazebo-ros \
ros-foxy-gazebo-ros2-control \
ros-foxy-controller-manager \
ros-foxy-joint-state-broadcaster \
ros-foxy-joint-state-publisher-gui \
ros-foxy-joint-trajectory-controller \
ros-foxy-moveit-ros-perception \
ros-foxy-rviz-visual-tools \
ros-foxy-moveit-resources
```


Switch to Cyclone DDS


```
sudo apt install ros-foxy-rmw-cyclonedds-cpp
```

#### Add this to ~/.bashrc to source it automatically

```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```


### Connect Mycobot and NUC to same network layer.

Run following cmd on terminal of ROS PC and Mycobot, the ID can follow your settings.

On Mycobot

```
source /opt/ros/galactic/setup.bash
```

On both Mycobot and PC

```
export ROS_DOMAIN_ID=30
```


### MYCOBOT SETUP

Copy control_node.py in mycobot_280pi_utils folder to /mycobot_ros2/mycobot_280/mycobot_280pi/mycobot_280pi.

Add 'control_node = mycobot_280pi.control_node:main' to mycobot_ros2/mycobot_280/mycobot_280pi/setup.py.

Test control_node on Mycobot.

```
ros2 run mycobot_280pi control_node --ros-args -p robot:=robot0
```

You can check topic on Mycobot and ROS PC.

CMD line to publish data using terminal

```
ros2 topic pub /robot0/control_radians std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

```
ros2 topic pub /robot0/control_coords std_msgs/msg/Float32MultiArray "{data: [149.8, -63.7, 175.1, -170.68, -0.47, -89.81]}"
```


### Sigma SETUP (On ROS PC)

For 64bit

```
wget https://www.forcedimension.com/downloads/sdk/sdk-3.15.0-linux-x86_64-gcc.tar.gz
tar -xzvf sdk-3.15.0-linux-x86_64-gcc.tar.gz
```

For 32bit

```
wget https://www.forcedimension.com/downloads/sdk/sdk-3.15.0-linux-i686-gcc.tar.gz
tar -xzvf sdk-3.15.0-linux-i686-gcc.tar.gz
```

check ld -ldhd --verbos / ld -ldrd --verbos
copy libdhd.a, libdrd.a to library folder
(32bit and 64bit version will show different library directory)

Example for 64bit version

```
sudo mkdir -p /usr/local/lib/x86_64-linux-gnu
cd /sdk-3.15.0/lib/release/lin-x86_64-gcc
sudo cp libdhd.a /usr/local/lib/x86_64-linux-gnu/libdhd.a
sudo cp libdrd.a /usr/local/lib/x86_64-linux-gnu/libdrd.a
```

Config UDEV

```
sudo nano /etc/udev/rules.d/sigma.rules
```
copy ATTRS{idProduct}=="0403", ATTRS{idVendor}=="1451",MODE="666",GROUP="plugdev"
```
sudo udevadm trigger
```


## Usage

### To use one Sigma with one Mycobot

**On Mycobot (raspberry pi)**

Run control node

```
ros2 run mycobot_280pi control_node --ros-args -p robot:=robot0
```

**On ROS PC (connect Sigma)**

Run Sigma node

```
ros2 launch sigma sigma.launch.py
```

Run Sigma and Mycobot bridge node

```
ros2 launch mycobot_280pi one_sigma_mycobot.launch.py robot_name:=robot0 sigma_name:=sigma0
```

### To use two Sigma with two Mycobot

**On Mycobot_0 (raspberry pi)**

Run control node

```
ros2 run mycobot_280pi control_node --ros-args -p robot:=robot0
```

**On Mycobot_1 (raspberry pi)**

Run control node

```
ros2 run mycobot_280pi control_node --ros-args -p robot:=robot1
```

**On ROS PC (connect Sigma)**

Run Sigma node

```
ros2 launch sigma sigma.launch.py
```

Run Sigma and Mycobot bridge node

```
ros2 launch mycobot_280pi two_sigma_mycobot.launch.py robot0_name:=robot0 sigma0_name:=sigma0 robot1_name:=robot1 sigma1_name:=sigma1
```


### To use one Mycobot with Moveit


**On Mycobot (raspberry pi)**

Run control node

```
ros2 run mycobot_280pi control_node --ros-args -p robot:=robot0
```

**On ROS PC (connect Sigma)**

Run moveit launch (this default robot name is robot0)

```
ros2 launch mycobot_280pi_moveit mycobot_280pi_moveit_launch.py
```


### To use two Mycobot with Moveit


**On Mycobot_0 (raspberry pi)**

Run control node

```
ros2 run mycobot_280pi control_node --ros-args -p robot:=robot0
```

**On Mycobot_1 (raspberry pi)**

Run control node

```
ros2 run mycobot_280pi control_node --ros-args -p robot:=robot1
```

**On ROS PC (connect Sigma)**

Run moveit launch (this default robot name is robot0 and robot1 init pos is 0 0 0 and 0 0.5 0)

```
ros2 launch mycobot_280pi_moveit two_mycobot_280pi_moveit_launch.py
```

### Motion Record

To use motion record, please start following cmd

```
ros2 run mycobot_280pi mycobot_motion_record.py --ros-args -p robot:=robot0
```


### Other Use

To set the workspace for robot, please modify the **workspace_params.yaml** in **mycobot_280pi/config** and rebuild the software.


This pakage provide dummy robot.
The robot can be controlled by /control_radians topic.
If you want to test with dummy robot.

The **quantity:=1** is number of robot. The robot name will start from 0.

```
ros2 launch mycobot_280pi mycobot_dummies.launch.py quantity:=1
```

Description for robot.

Use following cmd to launch robot description. (Need dummy robot or real robot for joint_states).

```
ros2 launch mycobot_280pi_description one_mycobot_280pi.launch.py robot_name:=robot0 robot_xyz:='"0 0 0"' robot_rpy:='"0 0 0"'
```

```
ros2 launch mycobot_280pi_description two_mycobot_280pi.launch.py robot0_name:=robot0 robot0_xyz:='"0 0 0"' robot0_rpy:='"0 0 0"' robot1_name:=robot1 robot1_xyz:='"0 0.5 0"' robot1_rpy:='"0 0 0"'
```

