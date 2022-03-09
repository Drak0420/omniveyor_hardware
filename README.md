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
# dirty-fix: nimbro_network cmake file formating
sed -i 's/cmake_minimum_required(VERSION 3.10)/cmake_minimum_required(VERSION 3.10.0)/g' \
    ./src/omniveyor_hardware/nimbro_network/nimbro_network/CMakeLists.txt
# now it's safe to build.
catkin_make -DCMAKE_BUILD_TYPE=Release
```
