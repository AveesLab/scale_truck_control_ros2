#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

def generate_launch_description():
    ld = LaunchDescription()

    '''
    #같은 패키지에 속한 런치 파일을 불러올 경우
    LogInfo(msg=['Execute three launch files!']),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/xxxxx.launch.py']),
    )
    '''

    ros_param_file = os.path.join(
            get_package_share_directory('scale_truck_control_ros2'), 
            'config', 
            'config.yaml')

    control_node=Node(
            package='scale_truck_control_ros2', 
            namespace='FV1', 
            name='scale_truck_control_node', 
            executable='control_node', 
            parameters = [ros_param_file],
            output='screen')

    lrc_node=Node(
            package='scale_truck_control_ros2', 
            namespace='FV1', 
            name='LRC', 
            executable='lrc_node', 
            parameters = [ros_param_file],
            output='screen')

    opencr_node=Node(
            package='micro_ros_agent', 
            name='opencr', 
            namespace='FV1', 
            executable='micro_ros_agent', 
            arguments = ["serial", "--dev", "/dev/ttyACM0"]
            )

#    ld.add_action(control_node)
    ld.add_action(lrc_node)
#    ld.add_action(opencr_node)

    return ld

