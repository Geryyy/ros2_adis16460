from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from datetime import datetime
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    imu_node = Node(
                package='imu_cpp',
                executable='imu_publisher',
                name='imu_publisher',
                output='both',
                parameters=[{
                    'use_sim_time': True,
                    'bias0_angvel_x': 5.79e-05,
                    'bias0_angvel_y': 0.00217,
                    'bias0_angvel_z': 0.00202,
                    'bias1_angvel_x': 0.00131,
                    'bias1_angvel_y': 0.00264,
                    'bias1_angvel_z': -0.00050,
                }]
            )

    return LaunchDescription([
        imu_node
    ])
