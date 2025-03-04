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
                }]
            )

    return LaunchDescription([
        imu_node
    ])
