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

    ros_param_file = os.path.join(
            get_package_share_directory('scale_truck_control_ros2'), 
            'config', 
            'config.yaml')
    
    usb_cam_node=Node(
            package='usb_cam',
            namespace='LV',
            name='usb_cam',
            executable='usb_cam_node_exe',
            parameters = [
                {"video_device": "/dev/video0"},
                {"framerate": 30.0},
                {"io_method": "mmap"},
                {"frame_id": "usb_cam"},
                {"pixel_format": "yuyv"},
                {"image_width": 640},
                {"image_height": 480}
            ],
            output='screen')

    lane_detection_node=Node(
            package='lane_detection_ros2',
            namespace='LV',
            name='LaneDetection',
            executable='lane_detect_node',
            parameters = [
                {"image_view/enable_opencv": "true" }
            ]    
#            output='screen'
            )

    control_node=Node(
            package='scale_truck_control_ros2', 
            namespace='LV', 
            name='scale_truck_control_node', 
            executable='control_node', 
#            output='screen',
            parameters = [ros_param_file])

    lrc_node=Node(
            package='scale_truck_control_ros2', 
            namespace='LV', 
            name='LRC', 
            executable='lrc_node', 
            parameters = [ros_param_file]
#            output='screen'
            )

    opencr_node=Node(
            package='micro_ros_agent', 
            name='opencr', 
            namespace='LV', 
            executable='micro_ros_agent', 
            arguments = ["serial", "--dev", "/dev/ttyACM0"]
            )

    ld.add_action(usb_cam_node)
    ld.add_action(lane_detection_node)
    ld.add_action(control_node)
#    ld.add_action(lrc_node)
#    ld.add_action(opencr_node)

    return ld

