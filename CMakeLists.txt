cmake_minimum_required(VERSION 3.8)
project(scale_truck_control_ros2)

set(REQUIRED_QT_VERSION 5.9.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(TRUCK_SDK_PATH "./sdk/")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(scale_truck_control_ros2 REQUIRED)

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(PROJECT_SOURCES
  src/controller/main.cpp
  src/controller/controller.cpp
  src/controller/ros2node.cpp
  src/controller/controller.ui
  src/controller/lvthread.cpp
  src/controller/fv1thread.cpp
  src/controller/fv2thread.cpp
  
  include/controller/controller.h
  include/controller/ros2node.hpp
  include/controller/lvthread.h
  include/controller/fv1thread.h
  include/controller/fv2thread.h
)

set(PROJECT_SOURCES_DEPENDENCIES
  rclcpp 
  std_msgs 
  Qt5Core 
  Qt5Widgets 
  ament_index_cpp 
  scale_truck_control_ros2
)

set(msg_files
  "msg/Ocr2lrc.msg"
  "msg/Lrc2ocr.msg"
  "msg/Lrc2xav.msg"
  "msg/Xav2lrc.msg"
  "msg/CmdData.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
 ${msg_files}
 DEPENDENCIES geometry_msgs
)

include_directories(
	include 
) # Set PATH for custom header file

add_executable(controller ${PROJECT_SOURCES})
ament_target_dependencies(controller ${PROJECT_SOURCES_DEPENDENCIES})

add_executable(lrc_node src/lrc.cpp)
ament_target_dependencies(lrc_node rclcpp std_msgs scale_truck_control_ros2)

add_executable(control_node src/scale_truck_control.cpp)
ament_target_dependencies(control_node rclcpp std_msgs scale_truck_control_ros2)

target_link_libraries(controller
  Qt5::Core
  Qt5::Quick
  Qt5::Widgets
)

install(TARGETS
	lrc_node
	control_node
	controller
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
