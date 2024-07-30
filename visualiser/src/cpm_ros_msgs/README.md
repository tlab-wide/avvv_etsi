# cpm_ros_msgs

A ROS2 interface to CPMs

## About

This package provides a CPM-ROS interface. The provided messages have fields equivalent to those of the [Collective Perception Messages](https://www.etsi.org/deliver/etsi_tr/103500_103599/103562/02.01.01_60/tr_103562v020101p.pdf), as required by the Autonomous Vehicle Network Visualisation project.

## Prerequisites

- [ROS2 Humble](http://docs.ros.org/en/humble/Installation.html)
- [colcon](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

## Build and Use

1. Source your ROS2 Humble:
```console
source /opt/ros/humble/setup.bash
```
2. Clone into the **src** subdirectory of your ROS2 workspace and use **colcon** to build:
```console
colcon build --packages-select cpm_ros_msgs
```
3. **#include** or **import** the message types to your source code