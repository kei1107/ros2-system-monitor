# ros2-system-monitor

Fork version of [ros-system-monitor(bb594ff)](https://github.com/ethz-asl/ros-system-monitor/tree/bb594ffd6fb4aa9b31ff9d887bdd8ff720ac8772)

## Synopsis

System monitoring tools for ROS2.

**License:** BSD-3 License (BSD-3)

## Description

This project provides system monitoring tools for ROS2 in the form of the following ROS2 nodes:

* CPU monitor
* HDD monitor
* Memory monitor
* Network monitor
* NTP monitor

Each node publishes ROS diagnostics which can conveniently be visualized in the runtime monitor.

## Installation

**install depend packages**

```shell
git clone https://github.com/kei1107/ros2-system-monitor
rosdep install -i --from-paths ros2-system-monitor
```

**build**

```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Releae
```
