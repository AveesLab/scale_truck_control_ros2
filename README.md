# scale_truck_control_ros2
scale_truck_control ros2 version


# Install Micro-ros 
- 본인의 ROS2 워크스페이스에서 작업하면 됩니다.
```
cd ~/ros2_ws/src 
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git
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
