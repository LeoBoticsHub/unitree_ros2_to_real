# unitree_ros2_to_real

This package can send control command to real robot from ROS 2. You can do low-level control(namely control all joints on robot) and high-level control(namely control the walking direction and speed of robot).

This version is suitable for unitree_legged_sdk v3.8.3, namely B1 robot.

## Packages

Basic message function: `ros2_unitree_legged_msgs`

The interface between ROS and real robot: `ros2_unitree_legged_real`

## Environment

We recommand users to run this package in Ubuntu 22.04 and ROS humble environment.

## Dependency

make sure to install the following sdk on your computer before using this package.

* [unitree_legged_sdk](https://github.com/LeoBoticsHub/unitree_ros_to_real/tree/B1_devel) (v3.8.3)

### Notice

The release v3.8.3 only supports for robot: B1.

## Build

You can use colcon to build the ROS2 packages.  \
Clone the package in your workspace (e.g., `~/ros_ws/src`) and run `colcon build` from the workspace folder.

## Run

To run the high level B1 controller, bash the workspace and run the following:

```bash
    ros2 run ros2_unitree_legged_real unitree_high_ros2_control
```

## Client service handler interface

To simplify interaction with the services spawn by `unitree_high_ros2_control` node, we provided a library containing the class `UnitreeRos2Client` which provide a easy to use interface with ros services. \
See [unitree_ros2_client.hpp](ros2_unitree_legged_real/include/ros2_unitree_legged_real/unitree_ros2_client.hpp) and [unitree_services_handler.cpp](ros2_unitree_legged_real/src/unitree_ros2_client.cpp) to have an hint on how to use it.

## OLD README VERSION

The original README.md version can be found here: [OLD_README](old_readme.md)

## Authors

The package is provided by:

* [Federico Rollo](https://github.com/FedericoRollo) [Mantainer]
