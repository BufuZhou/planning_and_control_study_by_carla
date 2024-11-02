


目前carla中获取轨迹有两种方法：
（1）通过get_route，利用carla内部规划的路径；
（2）通过carla-ros-bridge，运行内置的自动驾驶规划模式或者手动操作，获得路径。路径保存在运行节点的当前路径
（3）通过（2）中获得的路径名称为actual_driving_path_of_vehicle.txt，替换planning中data文件下txt内容