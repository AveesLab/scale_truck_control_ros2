# scale_truck_control_ros2
scale_truck_control ros2 version

# Develop History
**2023.05.24**
```
Implemented dual ROI version for normal and lane-changing modes

- Added distinction between normal mode and lane change mode using two versions of ROI.
- Established lane change process: CMD -> FV2 lane change command -> FV2 lane change complete -> CMD -> FV1 lane change command -> FV1 lane change complete -> CMD -> LV lane change command -> LV lane change complete -> CMD.
- Implemented controller button toggle upon completion of lane change.
```

# Install Micro-ros-Aruduino
- 본인의 ROS2 워크스페이스에서 작업하면 됩니다.
```
source /opt/ros/galactic/setup.bash
cd ~/ros2_ws/src 
git clone https://github.com/micro-ROS/micro_ros_setup.git -b galactic
```

- rosdep으로 의존 패키지 업데이트
```
sudo apt update && rosdep update
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
```

- pip 설치
```
sudo apt-get install python3-pip
```

- micro-ROS tools 과 소스 빌드
```
cd ~/ros2_ws
colcon build --packages-select micro_ros_setup
source install/local_setup.bash
```

- micro-ROS agent 패키지 설치
```
cd ~/ros2_ws
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

- Test
```
sudo chmod 777 /dev/ttyACM0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

# ROS2 Packages Install
- ros2_msg
```
cd ~/ros2_ws/src
git clone https://github.com/AveesLab/ros2_msg.git
cd ~/ros2_ws && colcon build --symlink-install && . install/setup.bash
```

- usb_cam
```
sudo apt-get install ros-galactic-usb-cam
cd ~/ros2_ws/src
git clone https://github.com/ros-drivers/usb_cam.git -b ros2
```

- rplidar_ros2
```
cd ~/ros2_ws/src
git clone https://github.com/CarlDegio/rplidar_ros.git -b ros2
```

- laser_filter
```
sudo apt-get install ros-galactic-filters
sudo apt-get install ros-galactic-angles
sudo apt-get install ros-galactic-laser-geometry
sudo ln -s /usr/include/eigen3/Eigen  /usr/include/Eigen
cd ~/ros2_ws/src
git clone https://github.com/wonseokkkk/laser_filters.git
```

- object_detection
```
sudo apt-get install ros-galactic-perception-pcl
cd ~/ros2_ws/src
git clone https://github.com/AveesLab/object_detection_ros2.git
```

- lane_detection
```
cd ~/ros2_ws/src
git clone https://github.com/AveesLab/lane_detection_ros2.git
```
- scale_truck_control_ros2
```
cd ~/ros2_ws/src
git clone https://github.com/AveesLab/scale_truck_control_ros2.git
```

# alias
```
sudo vim ~/.bashrc
```
```
alias cw='cd ~/ros2_ws/src'
alias cb='source ~/ros2_ws/install/setup.bash'
alias sb='source ~/.bashrc'
alias cm='cd ~/ros2_ws && colcon build --symlink-install && . install/setup.bash'
#alias cm='cd ~/ros2_ws && colcon build --packages-select scale_truck_control_ros2  --symlink-install && . install/setup.bash'
alias eb='sudo vim ~/.bashrc'
```
