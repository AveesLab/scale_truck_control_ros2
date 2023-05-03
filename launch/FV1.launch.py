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

    ###############
    # Lidar param #
    ##############
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000') 
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode=LaunchConfiguration('scan_mode', default='DenseBoost')#Standard,DenseBoost
<<<<<<< HEAD

#    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
#    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000') #for A3 is 256000
#    frame_id = LaunchConfiguration('frame_id', default='laser')
#    inverted = LaunchConfiguration('inverted', default='false')
#    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
#    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
=======
>>>>>>> 6fd89c6bf4f74e63d12834d9d6141e1a0a04ffb2

#    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
#    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000') #for A3 is 256000
#    frame_id = LaunchConfiguration('frame_id', default='laser')
#    inverted = LaunchConfiguration('inverted', default='false')
#    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
#    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
#

    ros_param_file = os.path.join(
            get_package_share_directory('scale_truck_control_ros2'), 
            'config', 
            'config.yaml')                 
    
    lane_param_file = os.path.join(
            get_package_share_directory('scale_truck_control_ros2'), 
            'config', 
            'FV1.yaml')                 

    # Node #
    usb_cam_node=Node(
            package='usb_cam',
            namespace='FV1',
            name='usb_cam',
            executable='usb_cam_node_exe',
            parameters = [ # default value 
                {"video_device": "/dev/video0"},
                {"framerate": 30.0},
                {"io_method": "mmap"},
                {"frame_id": "usb_cam"},
                {"pixel_format": "yuyv"},
                {"image_width": 640},
                {"image_height": 480}
            ],
            output='screen')

    rplidarS2_node=Node(
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

#    rplidarA3_node=Node(
#            package='rplidar_ros2',
#            namespace='FV1',
#            executable='rplidar_scan_publisher',
#            name='rplidar_scan_publisher',
#            parameters=[{'serial_port': serial_port, 
#                         'serial_baudrate': serial_baudrate, 
#                         'frame_id': frame_id,
#                         'inverted': inverted, 
#                         'angle_compensate': angle_compensate, 
#                         'scan_mode': scan_mode}],
#            output='screen')
                
    laserfilter_node=Node(
            package="laser_filters",
            namespace='FV1',
            name='scan_to_scan_filter_chain',
            executable="scan_to_scan_filter_chain",
            parameters=[
            	PathJoinSubstitution([
            		get_package_share_directory("laser_filters"),
            		"examples", "laserfilter_angle.yaml",
            		])],
            output='screen',)

    object_node=Node(
	    package='object_detection_ros2',
	    namespace='FV1',	    
	    executable='object_detection_ros2_node',
	    output={
	    	'stdout': 'screen',
	    	'stderr': 'screen',
	    	})

    lane_detection_node=Node(
            package='lane_detection_ros2',
            namespace='FV1',
            name='LaneDetector', # .yaml에 명시.
            executable='lane_detect_node',
#            output='screen',
            parameters = [lane_param_file])

    control_node=Node(
            package='scale_truck_control_ros2', 
            namespace='FV1', 
            name='scale_truck_control_node', 
            executable='control_node', 
#            output='screen',
            parameters = [ros_param_file])

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

    ld = LaunchDescription()
<<<<<<< HEAD
   
=======
    
>>>>>>> 6fd89c6bf4f74e63d12834d9d6141e1a0a04ffb2
    ld.add_action(rplidarS2_node)
    ld.add_action(usb_cam_node)
    ld.add_action(lane_detection_node)
#    ld.add_action(rplidarA3_node)
    ld.add_action(laserfilter_node)
    ld.add_action(object_node)
    ld.add_action(control_node)
    ld.add_action(lrc_node)
    ld.add_action(opencr_node)

    return ld


