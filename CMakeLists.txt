cmake_minimum_required(VERSION 3.8)
project(scale_truck_control_ros2)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(TRUCK_SDK_PATH "./sdk/")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(scale_truck_control_ros2 REQUIRED)

set(msg_files
  "msg/Ocr2lrc.msg"
  "msg/Lrc2ocr.msg"
  "msg/Xav2lrc.msg"
  "msg/Lrc2xav.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
 ${msg_files}
 DEPENDENCIES geometry_msgs
)

include_directories(
	include 
	include/scale_truck_control
) # Set PATH for custom header file

add_executable(lrc_node src/lrc.cpp)
ament_target_dependencies(lrc_node rclcpp std_msgs scale_truck_control_ros2)

add_executable(control_node src/scale_truck_control.cpp)
ament_target_dependencies(control_node rclcpp std_msgs scale_truck_control_ros2)

install(TARGETS
	lrc_node
	control_node
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
	config
	DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY 
	include/
  	DESTINATION include
)

ament_package()
