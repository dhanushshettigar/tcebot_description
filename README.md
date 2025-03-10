# tcebot_description

This package contains the description files for the tcebot robot in ROS 2.

## Package Overview

The `tcebot_description` package provides the necessary files to define the physical structure and properties of the robot using URDF/Xacro.

## Installation

To install the package, navigate to your ROS 2 workspace and clone the repository:

```bash
cd ~/ros2_ws/src
git clone https://github.com/dhanushshettigar/tcebot_description.git
cd ~/ros2_ws
colcon build --packages-select tcebot_description
```
## Usage

To launch the robot description, use:

```bash
ros2 launch tcebot_description robot_description.launch.py
```

## Dependencies

Ensure the following dependencies are installed:

```bash
sudo apt update && sudo apt install -y \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-rviz2
```
