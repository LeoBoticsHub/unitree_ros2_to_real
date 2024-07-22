# unitree_ros2_to_real

This package can send control command to real robot from ROS 2. You can do low-level control(namely control all joints on robot) and high-level control(namely control the walking direction and speed of robot).

This version is suitable for unitree_legged_sdk v3.8.3, for B1 robot and v3.8.6 fo Go1.

## Packages

Basic message function: `ros2_unitree_legged_msgs`

The interface between ROS and real robot: `ros2_unitree_legged_real`

## Environment

We recommand users to run this package in Ubuntu 22.04 and ROS humble environment.

## Dependency

make sure to install the following sdk on your computer before using this package.

* B1 robot: [unitree_legged_sdk](https://github.com/LeoBoticsHub/unitree_legged_sdk/tree/B1) (v3.8.3)

* Go1 robot: [unitree_legged_sdk](https://github.com/LeoBoticsHub/unitree_legged_sdk/tree/go1) (v3.8.6)

### Notice

The release v3.8.3 only supports for robot: B1.
The release v3.8.6 only supports for robot: Go1.

## Build

You can use colcon to build the ROS2 packages.  \
Clone the package in your workspace (e.g., `~/ros_ws/src`) and run from `ros_ws` directory:

* B1 robot:  `colcon build colcon build --cmake-args " -DUSE_STATIC_LIBRARY=OFF"`
* Go1 robot:  `colcon build --cmake-args " -DUSE_STATIC_LIBRARY=ON"`

## Run

To run the high level controller, bash the workspace and run the following:

* B1 robot:
  
```bash
    ros2 launch ros2_unitree_legged_real ros2_control.launch.py robot_type:=b1
```

* Go1 robot:
  
```bash
    ros2 launch ros2_unitree_legged_real ros2_control.launch.py robot_type:=go1
```

### Docker image

If you want to run the controller directly on the robots main computer, use connect to main computer and run:

* B1 robot:

```bash
docker_factory_run -i b1/ros2_control
```

* Go1 robot:

```bash
docker_factory_run -i go1/ros2_control
```

or, if you use [robot_setup](https://github.com/LeoBoticsHub/robots_setup) bashrcs, from your pilot laptop,

```bash
launch_ros2_control
```

## Unitree ROS 2 client interface

To simplify interaction with the services spawn by `unitree_high_ros2_control` node, we provided c++ and python libraries containing the class `UnitreeRos2Client` which provide a easy to use interface with ros services.

* See [unitree_ros2_client.hpp](ros2_unitree_legged_real/include/ros2_unitree_legged_real/unitree_ros2_client.hpp) and [unitree_services_handler.cpp](ros2_unitree_legged_real/src/unitree_ros2_client.cpp) to have an hint on how to use the c++ version.

* See [unitreeRos2Client.py](ros2_unitree_legged_real/ros2_unitree_legged_real/unitreeRos2Client.py) for the python version

## Testing

To test the functionalities of ```unitree_high_ros2_control``` node we provided a quick reference test to run.
Start by running the ROS2 controllor on the robot or your PC (Be sure that robot and host pc are on the same network or correctly connected):

```bash
    ros2 launch ros2_unitree_legged_real ros2_control.launch.py robot_type:={ROBOT_TYPE}
```

Once the controller is started, move the robot in stand down position (L2+A on controller) and start the test procedure on your host pc. Open a terminal in ```ros2_unitree_legged_real/test``` and run the test:

```bash
    python test_ros2control.py
```

It will produce a recap of the test named ```test_recap_DATE_TIME``` where the succesfull and unsuccesful tests are reported.

## OLD README VERSION

The original README.md version can be found here: [OLD_README](old_readme.md)

## Authors

The package is provided by:

* [Federico Rollo](https://github.com/FedericoRollo) [Mantainer]
