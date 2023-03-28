#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ros_param_file = os.path.join(
            get_package_share_directory('scale_truck_control_ros2'), 
            'config', 
            'config.yaml')

    vehicle_param_file = os.path.join(
            get_package_share_directory('scale_truck_control_ros2'), 
            'config', 
            'LV.yaml')

    lrc_param_file = os.path.join(
            get_package_share_directory('scale_truck_control_ros2'), 
            'config', 
            'lrc_LV.yaml')

    control_node=Node(
            package='scale_truck_control_ros2', 
#            namespace='LV', 
            name='scale_truck_control_node', 
            executable='control_node', 
            parameters = [ros_param_file, vehicle_param_file],
            output='screen')

    lrc_node=Node(
            package='scale_truck_control_ros2', 
 #           namespace='LV', 
            name='LRC', 
            executable='lrc_node', 
            parameters = [ros_param_file, vehicle_param_file, lrc_param_file],
            output='screen')

    ld.add_action(control_node)
    ld.add_action(lrc_node)

    return ld

