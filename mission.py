#!/usr/bin/env python3

# Copyright 2023 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Simple mission for a single drone."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
from time import sleep

from as2_python_api.drone_interface import DroneInterface
import rclpy


def drone_run(drone_interface: DroneInterface) -> None:
    """
    Run the mission for a single drone.

    :param drone_interface: DroneInterface object
    :return: None
    """
    speed = 0.5
    takeoff_height = 1.0
    height = 1.0

    sleep_time = 2.0

    dim = 1.0
    path = [
        [-dim, dim, height],
        [-dim, -dim, height],
        [dim, -dim, height],
        [dim, dim, height]
    ]
    print('Start mission')

    # ARM OFFBOARD
    print('Arm')
    drone_interface.arm()
    sleep(sleep_time)
    print('Offboard')
    drone_interface.offboard()
    sleep(sleep_time)

    # TAKE OFF
    print('Take Off')
    drone_interface.takeoff(takeoff_height, speed=1.0)
    print('Take Off done')
    sleep(sleep_time)

    # GO TO
    for goal in path:
        print(f'Go to with path facing {goal}')
        drone_interface.go_to.go_to_point_path_facing(goal, speed=speed)
        print('Go to done')
    sleep(sleep_time)

    # LAND
    print('Landing')
    drone_interface.land(speed=0.5)
    print('Land done')

    # DISARM
    drone_interface.disarm()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Single drone mission')

    parser.add_argument('-n', '--namespace',
                        type=str,
                        default='px1',
                        help='ID of the drone to be used in the mission')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')

    args = parser.parse_args()
    drone_namespace = args.namespace
    verbosity = args.verbose

    print(f'Running mission for drone {drone_namespace} in real mode')

    rclpy.init()

    uav = DroneInterface(
        drone_id=drone_namespace,
        verbose=verbosity)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print('Clean exit')
    exit(0)
