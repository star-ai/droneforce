
# To get it running in ROS

## Install ROS Kinetic first
http://wiki.ros.org/kinetic/Installation

## Download the FlightGoggles simulator
https://github.com/AgileDrones/FlightGoggles
I had to download 1.5.6 to run on Ubuntu


## be inside the ROS dir (the directory this README.md lives in)
```bash
sudo apt install libzmqpp-dev libeigen3-dev libopencv-dev
git clone https://github.com/AgileDrones/FlightGogglesClientBindings.git src/FlightGogglesClientBindings

catkin_make
. devel/setup.sh
roslaunch droneforce test.launch
```


