from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from disropt.utils.graph_constructor import binomial_random_graph
import numpy as np
import sys
import argparse
import os


def generate_launch_description():
    N=2
    seed=3
    
    ble_devices = [
        {"name": "BLE_Arduino_Pandebono", "address": "CD:EA:14:C3:CB:A5", "uuid": "0000ffe1-0000-1000-8000-00805f9b34fb"},
        {"name": "BLE_Propeller_Mate"   , "address": "3C:71:BF:CF:14:1A", "uuid": "0000ffe1-0000-1000-8000-00805f9b34fb"},
    ]
    
    for arg in sys.argv:
        if arg.startswith("N:="):
            N = int(arg.split(":=")[1])
        if arg.startswith("seed:="):
            seed = int(arg.split(":=")[1])

    # generate communication graph (this function also sets the seed)
    Adj = binomial_random_graph(N, 0.2, seed=seed)

    # generate initial positions in [-3, 3] with z = 0
    P = np.zeros((N, 3))
    P[:, 0:2] = np.random.randint(-3, 3, (N, 2))

    # initialize launch description
    robot_launch = []       # launched after 10 sec (to let Gazebo open)
    launch_description = [] # launched immediately (will contain robot_launch)

    # add task table executable
    robot_launch.append(Node(
            package='choirbot_examples', executable='choirbot_taskassignment_table', output='screen',
            prefix=['xterm -hold -e'],
            parameters=[{'N': N}]))

    # add executables for each robot
    for i in range(N):

        
        ble_device = ble_devices[i]
        launch_description.append(Node(
            package='choirbot_io',
            executable='ble_bridge',
            name=f'ble_bridge_agent_{i}',
            output='screen',
            parameters=[
                {'cmd_topic': f'/agent_{i}/cmd_vel'},
                {'ble_address': ble_device["address"]},
                {'ble_uuid': ble_device["uuid"]}
            ]
        ))

        
    return LaunchDescription(launch_description)
