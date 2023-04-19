#!/bin/python3

from time import sleep
import rclpy
from as2_python_api.drone_interface_gps import DroneInterfaceGPS
from as2_msgs.msg import YawMode


def drone_run(drone_interface: DroneInterfaceGPS):

    origin = [40.158194, -3.380795, 0]

    speed = 2.0
    takeoff_height = 1.0
    height = 2.0

    sleep_time = 2.0
    yaw_mode = YawMode()
    yaw_mode.mode = YawMode.PATH_FACING

    dim = 0.000001
    path = [
        [47.3977419, 8.5455933, height],
        [47.3977419, 8.5455933 + dim, height],
        [47.3977419 + dim, 8.5455933 + dim, height],
        [47.3977419 + dim, 8.5455933, height],
        [47.3977419, 8.5455933, height]
    ]

    print("Start mission")

    # drone_interface.gps.set_origin(origin)

    ##### ARM OFFBOARD #####
    drone_interface.arm()
    sleep(3.0)
    drone_interface.offboard()
    sleep(10.0)

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(takeoff_height, speed=1.0)
    print("Take Off done")
    sleep(sleep_time)
    # return

    ##### FOLLOW PATH #####
    sleep(sleep_time)
    print(f"Follow path with path facing: [{path}]")
    drone_interface.follow_path()
    print("Follow path done")

    # ##### GO TO #####
    for goal in path:
        print(f"Go to with path facing {goal}")
        drone_interface.go_to.go_to_gps_point(goal, speed)
        print("Go to done")
    sleep(sleep_time)

    drone_interface.go_to.go_to_gps_point([40.158183, -3.380893, 2.0], 1.0)
    drone_interface.go_to.go_to_gps_point([40.158194, -3.380795, 2.0], 1.0)

    ##### LAND #####
    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    drone_interface.disarm()


if __name__ == '__main__':
    rclpy.init()
    uav = DroneInterfaceGPS("drone0", verbose=False, use_sim_time=True)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
