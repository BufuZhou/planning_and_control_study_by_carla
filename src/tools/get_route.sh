current_path=$(pwd)
echo $current_path
cp ${current_path}/src/tools/get_route.py ~/CARLA_0.9.13/PythonAPI/examples/get_route.py 
{
  gnome-terminal -t "carla" -x bash -c "sh ~/CARLA_0.9.13/CarlaUE4.sh;exec bash;"
} & # 只有carla启动以后，后面脚本才能够正常运行
{
  sleep 5s
  python3 ~/CARLA_0.9.13/PythonAPI/examples/get_route.py 
}

