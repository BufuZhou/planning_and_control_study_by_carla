carla ros中存在bug，需要进行一定修改，具体如下：
（1）不支持高版本的carla，需要修改version
（2）ros指令无法指定出生点以及设置车辆类型,源自于一些参数传递存在问题


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