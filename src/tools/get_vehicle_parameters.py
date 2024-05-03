import carla
import random
import math
from geometry_msgs.msg import Pose
from transforms3d.euler import euler2quat

#设置ego生成位置的选择模式，"spectator" OR "random"
SPAWN_POINT = "spectator"

#设置ego的控制模式，"set_transform" OR "ackermann_control" OR "control" OR "autopilot"
CONTROL_MODE = "autopilot"



def main():
    #创建client，并获取world
    client = carla.Client("localhost", 2000)
    world = client.load_world('Town03')

    #设置ego的车型
    ego_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz_2017')
    ego_bp.set_attribute('role_name','ego')

    if SPAWN_POINT == "spectator":
        # 选择当前spectator位置为ego生成位置
        spectator = world.get_spectator()
        spectator_tf = spectator.get_transform()
        spawn_point = spectator_tf
        # 手动设定特定的点
        x, y, z, roll, pitch, yaw = -6.446170,-79.055023,0.275307,0,0,92.004189
        # Transform(Location(x=44.070000, y=-5.840000, z=4.150000), Rotation(pitch=0.000000, yaw=173.198349, roll=0.000000))
        print(f"spawn ego vehicle at: {spawn_point}")
        spawn_point.location.x = x
        spawn_point.location.y = y
        spawn_point.location.z = z
        spawn_point.rotation.pitch = pitch
        spawn_point.rotation.yaw = yaw
        spawn_point.rotation.roll = roll
        print(f"spawn ego vehicle at: {spawn_point}")
    elif SPAWN_POINT == "random":
        #随机选择预定义的生成点为ego生成位置
        spawn_points = world.get_map().get_spawn_points()
        
        # #生成点的可视化
        # for i, spawn_point in enumerate(spawn_points):
        #     world.debug.draw_string(spawn_point.location, str(i), life_time=100)
        #     world.debug.draw_arrow(spawn_point.location, spawn_point.location + spawn_point.get_forward_vector(), life_time=100)

        spawn_point = random.choice(spawn_points)
    
    # 生成ego车
    ego_vehicle = world.try_spawn_actor(ego_bp, spawn_point)
    snap = world.wait_for_tick()
    init_frame = snap.frame
    run_frame = 0
    print(f"spawn ego vehicle at: {spawn_point}")

    #获取和设置车辆属性
    bbx = ego_vehicle.bounding_box
    physics = ego_vehicle.get_physics_control()
    print(f"bounding_box = {bbx}")
    print(f"physics = {physics}")

    torque_curve_list = physics.torque_curve
    print("the torque measured in Nm for a specific RPM of the vehicle's engine:")
    for i in torque_curve_list:
        engine_rpm = i.x     # r/min
        engine_torque = i.y        # Nm
        print(engine_rpm, engine_torque)
    print("The maximum RPM of the vehicle's engine: ", physics.max_rpm)
    print("The moment of inertia of the vehicle's engine: ", physics.moi)

    print("Mass of the vehicle: ", physics.mass)
    print("Center of mass of the vehicle: ", physics.center_of_mass)
    print("Curve that indicates the maximum steering for a specific forward speed:")
    for i in physics.steering_curve:
        speed = i.x
        max_steering_angle = i.y * 180 / math.pi 
        print(speed, max_steering_angle)
    print("Maximum angle that the wheel can steer: ", physics.wheels[0].max_steer_angle)
    print("Radius of the wheel: ", physics.wheels[0].radius)
    print("Radius of the wheel: ", physics.wheels[0].radius)
    print("Maximum brake torque: ", physics.wheels[0].max_brake_torque)
    print("Tire longitudinal stiffness per unit gravitational acceleration. Each vehicle has a custom value: ", physics.wheels[0].long_stiff_value)
    print("Maximum stiffness per unit of lateral slip. Each vehicle has a custom value: ", physics.wheels[0].lat_stiff_value)

    print("position of the front left wheel: ", physics.wheels[0].position.x)
    print("position of the front left wheel: ", physics.wheels[0].position.y)
    print("position of the front left wheel: ", physics.wheels[0].position.z)
    print("position of the front right wheel: ", physics.wheels[1].position.x)
    print("position of the front right wheel: ", physics.wheels[1].position.y)
    print("position of the front right wheel: ", physics.wheels[1].position.z)
    # physics.mass = 2000
    # ego_vehicle.apply_physics_control(physics)
    # ego_vehicle.set_light_state(carla.VehicleLightState.All)

    # #采用默认的异步变步长模式
    # while run_frame < 10000:
    #     snap = world.wait_for_tick()
    #     run_frame = snap.frame - init_frame
    #     print(f"-- run_frame = {run_frame}")

    #     #设置spectator跟随ego车
    #     spectator = world.get_spectator()
    #     ego_tf = ego_vehicle.get_transform()
    #     spectator_tf = carla.Transform(ego_tf.location, ego_tf.rotation)
    #     spectator_tf.location += ego_tf.get_forward_vector() * (-10)
    #     spectator_tf.location += ego_tf.get_up_vector() * 3
    #     spectator.set_transform(spectator_tf)

    #     #控制车辆
    #     if CONTROL_MODE == "set_transform":
    #         new_ego_tf = carla.Transform(ego_tf.location, ego_tf.rotation)
    #         new_ego_tf.location += ego_tf.get_forward_vector() * 0.1
    #         ego_vehicle.set_transform(new_ego_tf)
    #     elif CONTROL_MODE == "ackermann_control":
    #         #ackermann_control
    #         ackermann_control = carla.VehicleAckermannControl()
    #         ackermann_control.speed = 5.0
    #         ego_vehicle.apply_ackermann_control(ackermann_control)
    #     elif CONTROL_MODE == "control":
    #         #control
    #         control = carla.VehicleControl()
    #         control.throttle = 0.3
    #         ego_vehicle.apply_control(control)
    #     else:
    #         #为ego车设置自动驾驶
    #         ego_vehicle.set_autopilot(True)
        
    #     #获取车辆状态
    #     transform = ego_vehicle.get_transform()
    #     velocity = ego_vehicle.get_velocity()
    #     acceleration = ego_vehicle.get_acceleration()
    #     # print(f"-current transform = {transform}")
    #     # print(f"-current velocity = {velocity}")
    #     # print(f"-current acceleration = {acceleration}")

    # #销毁车辆
    # is_destroyed = ego_vehicle.destroy()
    # if is_destroyed:
    #     print(f"ego_vehicle has been destroyed sucessfully")

if __name__ == "__main__":
    main()