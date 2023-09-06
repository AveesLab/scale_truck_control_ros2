#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    ros_param_file = os.path.join(
            get_package_share_directory('scale_truck_control_ros2'), 
            'config', 
            'config.yaml')                 
    
    lane_param_file = os.path.join(
            get_package_share_directory('scale_truck_control_ros2'), 
            'config', 
            'SV1.yaml')                 

    # Node #
    csi_cam_node=Node(
            package='ros2_camera',
            namespace='SV1',
            name='csi_cam',
            executable='picam_streamer',
            parameters=[
                PathJoinSubstitution([
                        get_package_share_directory('ros2_camera'),
                        'config', 'picam_conf.yaml',
                        ])],
            remappings=[('/image_raw', 'usb_cam/image_raw')],
            output='screen',
            emulate_tty=True)

    lane_detection_node=Node(
            package='lane_detection_ros2',
            namespace='SV1',
            name='LaneDetector', # .yaml에 명시.
            executable='lane_detect_node',
            output='screen',
            parameters = [lane_param_file])

    control_node=Node(
            package='scale_truck_control_ros2', 
            namespace='SV1', 
            name='scale_truck_control_node', 
            executable='control_node', 
            output='screen',
            parameters = [ros_param_file])

    lrc_node=Node(
            package='scale_truck_control_ros2', 
            namespace='SV1', 
            name='LRC', 
            executable='lrc_node', 
            parameters = [ros_param_file],
            output='screen')

    opencr_node=Node(
            package='micro_ros_agent', 
            name='opencr', 
            namespace='SV1', 
            executable='micro_ros_agent', 
            arguments = ["serial", "--dev", "/dev/ttyACM0"]
            )

    ld = LaunchDescription()
    
#    ld.add_action(csi_cam_node)
    ld.add_action(lane_detection_node)
    ld.add_action(control_node)
    ld.add_action(lrc_node)
    ld.add_action(opencr_node)

    return ld
