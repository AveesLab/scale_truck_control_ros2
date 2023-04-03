#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000') 
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode=LaunchConfiguration('scan_mode', default='DenseBoost')#Standard,DenseBoost


    ld = LaunchDescription()

    ros_param_file = os.path.join(
            get_package_share_directory('scale_truck_control_ros2'), 
            'config', 
            'config.yaml')                 
         
    rplidar_node=Node(
            package='rplidar_ros2',
            namespace='FV1',
            executable='rplidar_scan_publisher',
            name='rplidar_scan_publisher',
            parameters=[{'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate,
                         'scan_mode':scan_mode}],
            output='screen')         
        
    laserfilter_node=Node(
            package="laser_filters",
            namespace='FV1',
            executable="scan_to_scan_filter_chain",
            parameters=[
            	PathJoinSubstitution([
            		get_package_share_directory("laser_filters"),
            		"examples", "laserfilter_angle.yaml",
            		])],
            output='screen',)

    object_node=Node(
	    package='pcl_object_detection',
	    namespace='FV1',	    
	    executable='pcl_object_detection_node',
	    output={
	    	'stdout': 'screen',
	    	'stderr': 'screen',
	    	})

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
    ld.add_action(rplidar_node)
    ld.add_action(laserfilter_node)
    ld.add_action(object_node)
    ld.add_action(control_node)
    ld.add_action(lrc_node)
#    ld.add_action(opencr_node)

    return ld


