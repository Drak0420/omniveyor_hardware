Hardware driver and supporting components of the OmniVeyor robot.

Contains:
- pcv_base: Mobile base driver
- nimbro_network: Network communication interface for multi-robot setup
- robot_localization: Extended Kalman Filter for odometry fusion

The packages are set up with the provided rosinstall files.
Usage:
```sh
cd ros_ws
mkdir ./src
rosinstall ./src https://raw.githubusercontent.com/HaoguangYang/omniveyor/master/omniveyor.rosinstall
rosinstall ./src https://raw.githubusercontent.com/HaoguangYang/omniveyor_hardware/master/omniveyor_hardware.rosinstall
# dirty-fix: nimbro_network cmake file formating
sed -i 's/cmake_minimum_required(VERSION 3.10)/cmake_minimum_required(VERSION 3.10.0)/g' \
    ./src/omniveyor_hardware/nimbro_network/nimbro_network/CMakeLists.txt
# now it's safe to build.
catkin_make -DCMAKE_BUILD_TYPE=Release
# arduino-based payload interface
if compgen -G "/dev/ttyACM*" > /dev/null; then
    PORT=`ls /dev/ttyACM* | head -1`
    arduino --upload --board arduino:avr:uno --port /dev/ttyACM ${PORT} \
        ./src/omniveyor_hardware/pcv_base/resources/payload-Generic/payload-Generic.ino
fi
```

Hardware setup:
```sh
# setup canbus
sudo apt install -y can-utils
sudo cp ./src/omniveyor_hardware/pcv_base/resources/setup/can.conf /etc/modules-load.d/
sudo chmod +x /etc/modules-load.d/can.conf
sudo cp ./src/omniveyor_hardware/pcv_base/resources/setup/canbus.service /etc/systemd/system/
sudo chmod +x /etc/systemd/system/canbus.service
sudo systemctl enable canbus

# setup x11vnc
sudo apt install -y x11vnc
sudo cp ./src/omniveyor_hardware/pcv_base/resources/setup/x11vnc.service /etc/systemd/system/
sudo chmod +x /etc/systemd/system/x11vnc.service
sudo systemctl enable x11vnc

# setup Robot ID
nodeNumber=$(hostname | tr -dc '0-9' | sed 's/^0*//')
echo "export NODE_NO=$nodeNumber" | sudo tee /etc/profile.d/robot_name.sh

# setup limits
echo "# increase message queue size
fs.mqueue.msg_max = 100" | sudo tee -a /etc/sysctl.conf
sudo sed '$i`whoami`         hard    memlock         524288' /etc/security/limits.conf
sudo sed '$i`whoami`         soft    memlock         524288' /etc/security/limits.conf
sudo sed '$i`whoami`         hard    priority        85' /etc/security/limits.conf
sudo sed '$i`whoami`         hard    rtprio          85' /etc/security/limits.conf
sudo sed '$i`whoami`         soft    rtprio          85' /etc/security/limits.conf

# restart sysctl to take effect
sudo sysctl -p
```
