# hero_chassis_controller

## Overview
This is cqc's last homework of DynamicX.

### Keywords: ros_control
### License
The source cod is released under a [BSD-3Clause license](license).

**Author : cqc <br>
Affiliationg :  Individual <br>
Maintainer:cqc,205455603@qq.com**

Hero_chassis_controller package has been test under Ros noetic on respectively 20.04.It is just only my homework.

## Installation
### Buildding from Source 

- Robot Operating System(ros)
- rm_description
- controller_interface
- forward_command_controller
- hardware_interface
- pluginlib
- teleop_twist_keyboard
- control_toolbox
- dynamic_reconfigure

### Building 

To build from source,using

	cd catkin_workspace/src
	git clone https://github.com/xuncheng-001/hero_chassis_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build

### Usage
Run the simulation and controller with:

    roslaunch hero_chassis_controller hero_chassis_controller.launch

### Config files
Config file config 
- **controllers.yaml** Params of hero_chassis_controller , joint_state_controller and pid parameter.

### Launch files
- **hero_chassis_controller.launch:** Hero chassis simulation and hero_chassis_controller and teleop_twist_keyboard.