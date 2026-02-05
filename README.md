# luh_youbot_os

The luh_youbot_os repository provides a software development kit for applications with the KUKA youBot. 
 
The image shows an overview of the core components.

![alt tag](https://raw.githubusercontent.com/LUHbots/luh_youbot_os/master/luh_youbot_os/luh_youbot_os_overview.png)


## Tutorials

https://github.com/LUHbots/luh_youbot_os/wiki


## Package Overview

Package | Description
------------- | -------------
luh_laser_watchdog        | The Laser Watchdog has two functions: 1. It filters velocity commands for the base to avoid collisions. 2. It reduces the laser scan data to four distance values (left, front, right and back). 
luh_youbot_calibration    | The calibration node can be used to identify the static parameters of the youbot manipulator. The static parameters are e.g. needed for gravity compensation. 
luh_youbot_controller     | The Controller Node is the central component of the SDK. It is responsible for trajectory planning and motion execution. It provides various subscribers, service servers and action servers for different kinds of motion commands for the manipulator, the gripper and the base.
luh_youbot_controller_api | The Controller API provides a user friendly programming interface for the motion commands of the Controller Node. This is the most important package for application developers.
luh_youbot_gui            | The GUI can be used to manually control the manipulator and to manage predefined poses. 
luh_youbot_description    | This package contains the URDF robot model definition.
luh_youbot_driver_api     | The Youbot Driver API is a wrapper for the youbot driver. It communicates with the motor controllers.
luh_youbot_gazebo         | Launchfiles and plugins needed for simulation with Gazebo
luh_youbot_gripper        | This Node is for the Gripper v2 with external Arduino connected via USB
luh_youbot_joy_teleop     | The Joy Teleop Node allows manual control of the youbot with a game controller.
luh_youbot_kinematics     | The Youbot Kinematics provides tools to calculate forward kinematics, inverse kinematics and statics of the youbot manipulator. 
luh_youbot_msgs           | This package contains all message, service and action definitions.
luh_youbot_poses          | This package contains some predefined poses and a library with which pose definition files can be read.
luh_youbot_vrep_api       | This is a fake driver API that sends commands to v-rep simulation.
youbot_driver             | A catkinized version of the youbot driver (https://github.com/youbot/youbot_driver).

For this controller, you need ROS Kinetic with Ubuntu 16.04

You need to install additional libraries and programs.
Install control-toolbox:

    sudo apt-get install ros-kinetic-control-toolbox

Install nlopt:

    sudo apt-get install ros-kinetic-nlopt

	sudo apt-get install libnlopt-dev

Create and initialize catkin workspace (i. e. youbot):

    mkdir -p ~/youbot/src
    cd ~/youbot/src
    catkin_init_workspace
    cd ..
    catkin_make

Clone the youBot GUI sources to /src folder:
    
    cd ~/youbot/src
    git clone https://github.com/CamiloEsc/luh_youbot_os.git

Add the repository folder to the ROS_PACKAGE_PATH environment variable.

Compile the youbot GUI by typing:

    cd ~/youbot
    catkin_make

Usage
------------

The youBot GUI give you  graphical user interface app to communicate with youBot arm via youbot_driver. You can move youBot arm with joints or in the global coordinate system.

To run youbot_gui you need to follow this commands:

    cd ~/youbot
    source devel/setup.bash
    rosrun luh_youbot_gui luh_youbot_gui

