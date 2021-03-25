# kobuki_micro_ros

Tested on ROS2 Foxy, Ubuntu 20.04

# ROS2 Installation
## Setup Locale

```
sudo locale-gen en_US en_US.UTF-8

sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8
```

## Setup Sources

```
sudo apt update && sudo apt install curl gnupg2 lsb-release

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```


## Install ROS2 Packages

```
sudo apt update
sudo apt install ros-foxy-desktop
source /opt/ros/foxy/setup.bash
sudo apt install python3-colcon-common-extensions
```


## Test the installation
```
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```
![ros2_output](img/ros2_output1.png
)

# Micro ROS Installation

## Update dependencies using rosdep

```
sudo apt update && rosdep update

sudo apt-get install python3-pip
```


## Source the ROS 2 installation
```
source /opt/ros/foxy/setup.bash

mkdir ~/microros_ws

cd ~/microros_ws

git clone -b foxy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep install --from-path src --ignore-src -y
```


