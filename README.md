# Inverse kinematics solver
<img src="https://img.shields.io/badge/ros--version-humble-green"/>  <img src="https://img.shields.io/badge/platform%20-Ubuntu%2022.04-orange"/>

## Description

The `inverse_kinematics` package contains functions for solving the inverse kinematics problem for redundant manipulators. The solution is computed using the pseudoinverse of the manipulator's Jacobian.

A work in progress in early stages.

## Installation

It is recommended to use Ubuntu 22.04 with [**ROS 2 Humble**](https://docs.ros.org/en/humble/index.html).

### Required packages

[**rrlib_interfaces**](https://github.com/EleganckaPorazka/rrlib_interfaces.git)

[**trajectory_generator**](https://github.com/EleganckaPorazka/trajectory_generator.git)

A package with the forward kinematics solver for your manipulator that computes end effector pose (PoseStamped) and the Jacobian of the manipulator (JacobianStamped), for example [**lwr_forward_kinematics**](https://github.com/EleganckaPorazka/lwr_forward_kinematics.git).

### Building from source

```
mkdir -p ~/ros2_ws/src/inverse_kinematics
cd ~/ros2_ws/src/inverse_kinematics
git clone https://github.com/EleganckaPorazka/inverse_kinematics.git .
source /opt/ros/humble/setup.bash
colcon build
. install/setup.bash
```

## Running

```
ros2 run inverse_kinematics inverse_kinematics_basic
```
Example:
```
ros2 param set /inverse_kinematics_basic DOF '7'
ros2 param set /inverse_kinematics_basic dt '0.005'
ros2 param set /inverse_kinematics_basic '[100.0, 1.0]'
```

## Notes

TODO: Can't get parameters from the yaml file: 'ros2 launch inverse_kinematics inv_kin.launch.py' or 'ros2 run inverse_kinematics inverse_kinematics_basic --ros-args --params-file ~/ros2_kuka/src/inverse_kinematics/config/IK_parameters.yaml' don't seem to work. Even 'ros2 run inverse_kinematics inverse_kinematics_basic --ros-args -p DOF:=7 -p dt:=0.01 -p clik_gains:="[100.0, 0.0]"' is not giving a desired effect. The command 'ros2 param dump /inverse_kinematics_basic' returns the parameters with proper values, but they don't seem to propagate to 'inverse_kinematics_basic_node.cpp'. Possibly, the whole parameter callback thing there is wrong. Frustrating.

This code is also uploaded to [**my other repository**](https://gitlab.com/lwolinski/inverse_kinematics.git).
