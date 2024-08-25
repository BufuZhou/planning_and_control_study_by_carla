# planning_and_control_study_by_carla
# Environmental requirements
## Ubuntu 20.04

## Carla 0.9.13
https://github.com/carla-simulator/carla/releases

## Carla-ros-bridge 
https://github.com/carla-simulator/ros-bridge
Update CARLA version to 0.9.13 (#630) e9063d97ff5a724f76adbb1b852dc71da1dcfeec

# install
## ubuntu20.04

## Carla 0.9.13
https://carla.readthedocs.io/en/0.9.13/start_quickstart/

## Carla-ros-bridge
https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/

mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge
source /opt/ros/foxy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r
colcon build

## glog
intstall:
sudo apt-get install libgoogle-glog-dev

version:
 apt show libgoogle-glog-dev


## gflag
sudo apt install libgflags-dev

version:
apt show libgflags-dev

## protobuf libprotoc 3.6.1
sudo apt-get install protobuf-compiler
sudo apt-get install libprotobuf-dev
protoc --version

## eigen
sudo apt install libeigen3-dev

查看版本
pkg-config --modversion eigen3

教程：
https://eigen.tuxfamily.org/index.php?title=Main_Page

https://github.com/ros2/eigen3_cmake_module

# code style (C++ google)
sudo apt install clang-format
sudo pip3 install cpplint

clangd

# ROS tutorials
http://dev.ros2.fishros.com/doc/Tutorials/Workspace/Creating-A-Workspace.html

https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html

## create package
colcon build  --cmake-args
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

# Module
# carla 0.9.13colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

cd ~/CARLA_0.9.13/
./CarlaUE4.sh

or
sh ~/CARLA_0.9.13/CarlaUE4.sh

## ros2
colcon build --packages-select vehicle control

## carla-ros-bridge
carla ros中存在bug，需要进行一定修改
（1）不支持高版本的carla，需要修改version
（2）ros指令无法指定出生点以及设置车辆类型
目前carla中获取轨迹有两种方法：
（1）通过get_route，利用carla内部规划的路径；
（2）通过carla-ros-bridge，运行内置的自动驾驶规划模式或者手动操作，获得路径。路径保存在运行节点的当前路径
（3）通过（2）中获得的路径名称为actual_driving_path_of_vehicle.txt，替换planning中data文件下txt内容

###
cd ~/carla-ros-bridge/
source ./install/setup.bash
or
source ~/carla-ros-bridge/install/setup.bash


ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py role_name:="ego_vehicle" vehicle_filter:="vehicle.lincoln.mkz_2017" spawn_point:="49.91,-7.778184,0.28,0,0,0"  town:="town03"
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py  town:="town03"


### spawn_point
~/carla-ros-bridge/src/ros-bridge/carla_spawn_objects/src/carla_spawn_objects/carla_spawn_objects.py
check_spawn_point_param
spawn_point:="49.91,-7.778184,0.28,0,0,0"
spawn_point:="-6.446170,-79.055023,0.275307,0,0,92.004189"
spawn_point定义为：
"x,y,z,roll,pitch,yaw"
carla example中生成spawn point为：
Transform(Location(x=-6.446170, y=-79.055023, z=0.275307), Rotation(pitch=0.000000, yaw=92.004189, roll=0.000000))
Yaw的位置不一样，需要进行调整

### ego_vehicle
~/lifanjie/carla-ros-bridge/src/ros-bridge/carla_spawn_objects/config/objects.json
"type": "vehicle.lincoln.mkz_2017",



##
cd ~/planning_and_control_study_by_carla 
source install/setup.bash
ros2 launch control control_with_planning.launch.py 

### modify parameter need rebuild 

![alt text](image.png)

## vehicle 
control comand to carla vehicle and get vehicle info from carla

## control
generate control command to control the vehicle

## planning




