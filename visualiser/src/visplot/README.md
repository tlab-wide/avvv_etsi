# visplot

Visually extension for a real time statistics plotter subscribing on ROSBAG topics

## Dependencies

### ROS dependency

- [ROS2 humble](https://docs.ros.org/en/humble/)

### Manageable using `rosdep`

You can resolve the following dependencies using [`rosdep`](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html). Simply navigate to the root of your workspace and run ```rosdep install --from-paths src/visplot -y --ignore-src```.
- [Python 3](https://www.python.org/)
- [matplotlib](https://pypi.org/project/matplotlib/)

### Manual

You need to manually clone the following package beside `visplot`.
- [cpm_ros_msgs](https://github.com/AmirInt/cpm_ros_msgs)

## Build

- Clone the repository into your ROS2 workspace's `src` folder:
```console
git clone git@github.com:AmirInt/visplot.git
```
- Source your ROS2 Humble:
```console
source /opt/ros/humble/setup.bash
```
- From the root of your wokspace, use `colcon` to build the package:
```console
colcon build --packages-select visplot
```

## Run

This is not a standalone application. It depends on [`visually`](https://github.com/AmirInt/visually) to launch with the correct parameters.
