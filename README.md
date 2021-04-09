# kobuki_micro_ros

Tested on ROS2 Foxy, Ubuntu 20.04, 

Hardware Requirements
* [Olimex-stm32-e407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)
* [Segger -LINK-EDU](https://www.digikey.in/product-detail/en/segger-microcontroller-systems/8-08-90-J-LINK-EDU/899-1008-ND/2263130)
* [usb-to-ttl converter](https://www.electronicscomp.com/cp2102-usb-to-ttl-serial-converter-module)

# ROS2 Installation
<details>
<summary> click to expand </summary>
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

</details>


# Micro ROS Installation

<details>
<summary> click to expand </summary>

## Update dependencies using rosdep

```
sudo apt update && rosdep update

sudo apt-get install python3-pip
```

## Build micro-ROS tools 

```
# Source the ROS 2 installation
sudo apt install python3-rosdep

source /opt/ros/foxy/setup.bash

mkdir ~/microros_ws

cd ~/microros_ws

git clone -b foxy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep install --from-path src --ignore-src -y

colcon build
source install/local_setup.bash
```
The build system’s workflow is a four-step procedure:

* Create step: This step is in charge of downloading all the required code repositories and cross-compilation toolchains for the specific hardware platform. Among these repositories, it will also download a collection of ready to use micro-ROS apps.

* Configure step: In this step, the user can select which app is going to be cross-compiled by the toolchain. Some other options, such as transport, agent’s IP address/port (for UDP transport) or device ID (for serial connections) will be also selected in this step.

* Build step: Here is where the cross-compilation takes place and the platform-specific binaries are generated.

* Flash step: The binaries generated in the previous step are flashed onto the hardware platform memory, in order to allow the execution of the micro-ROS app.

</details>

# olimex-stm32-e407 Board configuration 

<details>
<summary> click to expand </summary>

![olimex_board_layout_front](img/olimex_board_layout_front.png
)


| Pin Number      | Power Selection Input|
| ----------- | ----------- |
| 1 - 2      | +5V External DC Input       |
| 3 - 4   | +5V JTAG        |
| 5 - 6    | +5V USB OTG2     |
| 7 - 8    | +5V USB OTG1     |

![olimex_board_layout_back](img/olimex_board_layout_back.png
)


## Creating a new firmware workspace

```
ros2 run micro_ros_setup create_firmware_ws.sh <RTOS> <Platform>
```
In our case, use Seggar JTAG for flashing and powering the board( insert jumper to pin 3-4) of Olimex and power the JTAG by connecting the USB to power source. The Power LED (Red) will blink if connection is successful.

```
cd ~/microros_ws/
source install/local_setup.bash
ros2 run micro_ros_setup create_firmware_ws.sh freertos olimex-stm32-e407 
```
![micro_ros_creating_firmware](img/micro_ros_creating_firmware.png
)


Once the command is executed, a folder named firmware must be created in your workspace.
This step is in charge, among other things, of downloading a set of micro-ROS apps for the specific platform you are addressing. In the case of FreeRTOS, these are located at firmware/freertos_apps/apps. Each app is represented by a folder containing the following files:

* app.c: This file contains the logic of the application.

* app-colcon.meta: This file contains the micro-ROS app specific colcon configuration. 

!!! To check 
```
For the user to create its custom application, a folder <my_app> will need to be registered in this location, containing the two files just described.
```

## Configuring the firmware

The configuration step will set up the main micro-ROS options and select the desired application. It can be executed with the following command:

```
ros2 run micro_ros_setup configure_firmware.sh [APP] [OPTIONS]
```


The options available for this configuration step are:

* --transport or -t: udp, serial or any hardware-specific transport label

* --dev or -d: agent string descriptor in a serial-like transport

* --ip or -i: agent IP in a network-like transport

* --port or -p: agent port in a network-like transport

In this case, we will use a Serial transport (labeled as serial) and focus on the out-of-the-box int32_publisher application located at firmware/freertos_apps/apps/int32_publisher. To execute this application with the chosen transport, run the configuration command above by specifying the [APP] and [OPTIONS] parameters as below:

```
ros2 run micro_ros_setup configure_firmware.sh int32_publisher --transport serial
```

![micro_ros_configuring_firmware](img/micro_ros_configuring_firmware.png
)

## Building the firmware

```
ros2 run micro_ros_setup build_firmware.sh
```
Ignore the sdterr packages: it is due to cross compilations


![micro_ros_builiding_firmware](img/micro_ros_builiding_firmware.png
)

This will create bin and elf files.

## Flashing the firmware

Locate `/firmware/freertos_apps/microros_olimex_e407_extensions/build/micro-ROS.elf`

Then flash the firmware using Segger Jlink


|  Mode |  Jumper Position |
| ----------- | ----------- |
| Default Mode   | B0_0/B1_0     |
| Boot Mode    |    B0_1/B1_0  |

If the board is in boot mode (flashing), user GReen LED will not blink.

```
openocd -f ./interface/jlink.cfg -f ./target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase ~/home/koem-vinu/uros_ws/firmware/freertos_apps/microros_olimex_e407_extensions/build/micro-ROS.elf" -c "verify" -c "reset" -c "exit"
```

![micro_ros_flashing_firmware](img/micro_ros_flashing_firmware.png
)

Once flashing is done, change the jumpers to default mode and then press Reset button. You can see Green LED starts blinking.

## Creating the micro-ROS agent

Inorder to communicate with microROS client in the MCU we need a  micro-ROS agent to start talking with the rest of the ROS 2 world. To do that, create a micro-ROS agent:

Create the microROS agent workspace by command

```
ros2 run micro_ros_setup create_agent_ws.sh
```

Build  the microROS agent workspace by command and source the setup

```
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

Then, depending on the selected transport and RTOS, the board connection to the agent may differ.

* For serial communication -Use USB-TTL convertor
    
* For USB-SERIAL communication - Use onboard USB-OTG2 port to connect

![usb_ttl_olimex](/img/usb_ttl_olimex.png)

## Running the micro-ROS Agent

To give micro-ROS access to the ROS 2 dataspace, you just need to run the agent:

```
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
```


### Granting permission to device

```
sudo su
cd /dev
chown username ttyUSB0
```

![micro_ros_agent](img/micro_ros_agent.png
)

```
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 topic list
```

![micro_ros_topic](img/micro_ros_topic.png
)
</details>
