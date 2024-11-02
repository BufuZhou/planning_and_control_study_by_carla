# code style (C++ google)
sudo apt install clang-format
sudo pip3 install cpplint

clangd

# Ubuntu 20.04

# ros2 tutorials
http://dev.ros2.fishros.com/doc/Tutorials/Workspace/Creating-A-Workspace.html

https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html

# Carla 0.9.13
https://github.com/carla-simulator/carla/releases
https://carla.readthedocs.io/en/0.9.13/start_quickstart/

# Carla-ros-bridge 
https://github.com/carla-simulator/ros-bridge
Update CARLA version to 0.9.13 (#630) e9063d97ff5a724f76adbb1b852dc71da1dcfeec

https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/

mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge
source /opt/ros/foxy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r
colcon build

# glog
intstall:
sudo apt-get install libgoogle-glog-dev

version:
 apt show libgoogle-glog-dev

# gflag
sudo apt install libgflags-dev

version:
apt show libgflags-dev

# protobuf libprotoc 3.6.1
sudo apt-get install protobuf-compiler
sudo apt-get install libprotobuf-dev
protoc --version

# eigen
sudo apt install libeigen3-dev

查看版本
pkg-config --modversion eigen3
apt show libeigen3-dev

https://eigen.tuxfamily.org/index.php?title=Main_Page

https://github.com/ros2/eigen3_cmake_module

# tbb parallel 
sudo apt install libtbb-dev

# gtest

## foxglove
https://foxglove.dev/download

https://docs.foxglove.dev/docs/connecting-to-data/ros-foxglove-bridge


ros2 launch rosbridge_server rosbridge_websocket_launch.xml
