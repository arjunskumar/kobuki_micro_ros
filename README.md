# kobuki_micro_ros

# ROS2 Installation
## Setup Locale

```
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## Setup Sources

```
sudo apt update &amp;&amp; sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c &#39;echo &quot;deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu
$(lsb_release -cs) main&quot; &gt; /etc/apt/sources.list.d/ros2-latest.list&#39;
```


## Install ROS2 Packages

```sudo apt update
sudo apt install ros-dashing-desktop
source /opt/ros/dashing/setup.bash
sudo apt install python3-colcon-common-extensions
```


## Test the installation
```
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```