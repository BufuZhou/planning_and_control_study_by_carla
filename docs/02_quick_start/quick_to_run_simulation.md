# Follow these steps to perform simulation
## Step1: Run carla simulator:
Make sure calar is in the home folder, run:
```
sh ~/CARLA_0.9.13/CarlaUE4.sh
```

## Step2: Run carla-ros-bridge:
Add the source path for the carla-ros-bridge workspace. Make sure calar-ros-bridge is in the home folder. In another terminal, run:
```
source ~/carla-ros-bridge/install/setup.bash
```
Then, start the carla-ros-bridge:
```
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py role_name:="ego_vehicle" vehicle_filter:="vehicle.lincoln.mkz_2017" spawn_point:="49.91,-7.778184,0.28,0,0,0"  town:="town03"
```

## Step3: Run pnc node:
Add the source path for the pnc workspace. Make sure pnc is in the home folder. In another terminal, run:
```
source ~/planning_and_control_study_by_carla/install/setup.bash
```
Then, start the pnc node:
```
ros2 launch control control_with_planning.launch.py
```