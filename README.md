Hardware driver and supporting components of the OmniVeyor robot.

Contains:
- pcv_base: Mobile base driver
- nimbro_network: Network communication interface for multi-robot setup
- robot_localization: Extended Kalman Filter for odometry fusion

The packages are set up with the provided rosinstall files.
Usage:
```
cd ros_ws
mkdir ./src
rosinstall ./src https://raw.githubusercontent.com/HaoguangYang/omniveyor_hardware/master/omniveyor_hardware.rosinstall
```
