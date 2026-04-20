# BlueBoat ROS2

This repository contains the robot description and necessary launch files to describe and simulate the BlueBoat (Uncrewed Surface Vessel) with Gazebo and its hydrodynamics plugins under ROS 2.

Additionnal steps are included to make sure this can be used starting from a fresh Ubuntu install.

NOTE: This package is a modified version of the original [BlueROV2](https://github.com/CentraleNantesROV/bluerov2/tree/main), most of the physical and hydrodynamical properties are currently set on the bluerov2.

# Requirements

## ROS2
The current recommended ROS2 version is Jazzy. All the related info can be found [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## Gazebo
- The recommended gazebo version is GZ Harmonic (LTS). More info and step-by-step installation guide are found [here](https://gazebosim.org/docs/latest/ros_installation/)
- [pose_to_tf](https://github.com/oKermorgant/pose_to_tf), to get the ground truth from Gazebo if needed.

## For the description

- [Xacro](https://github.com/ros/xacro/tree/ros2) , installable through `apt install ros-${ROS_DISTRO}-xacro`
- [simple_launch](https://github.com/oKermorgant/simple_launch), installable through `apt install ros-${ROS_DISTRO}-simple-launch`

## For the control part

- [slider_publisher](https://github.com/oKermorgant/slider_publisher), installable through `apt install ros-${ROS_DISTRO}-slider-publisher`
- [auv_control](https://github.com/CentraleNantesROV/auv_control) for basic control laws
- [urdf_parser](https://github.com/ros/urdf_parser_py) intended to have the controller work with any robot description
#TODO add acados 

# Installation

- Clone the package and its dependencies (if from source) in your ROS 2 workspace `src` and compile with `colcon build`, make sure you are in the parent folder of `src` when compiling.

# Running 
- Make sure to source the terminal if you did not modify the bashrc file.

    `source /opt/ros/jazzy/setup.bash`

    `source install/setup.bash`

- To run a demonstration with the vehicle, you can run a Gazebo scenario, and spawn the robot with a GUI to control the thrusters:

    `ros2 launch blueboat_description world_launch.py sliders:=true`

# Input / output

Gazebo will:

- Subscribe to /blueboat/cmd_thruster[i] and expect std_msgs/Float64 messages (thrust in Newton).
- NOT YET IMPLEMENTED: ~~Publish sensor data to various topics (image, mpu+lsm for IMU, cloud for the sonar, odom)~~
- Publish the ground truth on /blueboat/pose_gt. This pose is forwarded to /tf if pose_to_tf is used.

# High-level control
Both control schemes are handled by the same node that takes the desired control and trajectory as a launch argument:

`ros2 launch blueboat_control Sim_launch.py controller_type:='PID' trajectory:='sin'`

The full list of trajectories is found in blueboat_control/src/_custom_libraries/path_generation.py

## Real robot
This code is meant to interact with the BlueRobotics BlueBoat: https://bluerobotics.com/store/boat/blueboat/blueboat/

Interaction with ROS2 uses the blue-os ROS2 app (can be directly installed through BlueOS app tab): https://github.com/itskalvik/blueos-ros2

The launch file that handles every robot interaction and control can be run with:

`ros2 launch blueboat_control BlueBoat_launch.py`

Different parameters can be handled through a single instruction:

`ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: [value]"`

The list of available [value] is as follows:
 - 'enable': no input will be sent to the thrusters until this has been called
 - 'stop': opposite of previous command, stops the robot and disable thrusters
 - 'override': disables default thruster mapping of the robot, used to send input directly to the motors (used for various controllers and terminal control, makes control through the xbox controller mpossible)
 - 'default': restores default mapping, to be used before closing the terminal
 - 'move': typical instruction follows the shape 'move [float_left] [float_right] [float_time]', it will set the float_left and float_right inputs to the thrusters and apply it for the float_time duration (seconds)

# License
blueboat package is open-sourced under the MIT License. See the LICENSE file for details.
