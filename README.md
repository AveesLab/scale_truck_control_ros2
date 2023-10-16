# scale_truck_control_ros2
ROS2 based scale_truck_control system

# Develop History
**2023.05.24**
```
Implemented dual ROI version for normal and lane-changing modes

- Added distinction between normal mode and lane change mode using two versions of ROI.
- Established lane change process: CMD -> FV2 lane change command -> FV2 lane change complete -> CMD -> FV1 lane change command -> FV1 lane change complete -> CMD -> LV lane change command -> LV lane change complete -> CMD.
- Implemented controller button toggle upon completion of lane change.
```

# I. Hardware
```
High-level Controller - NVIDIA Jetson AGX Orin 64GB
Low-level Controller  - OpenCR 1.0 (ARM Cortex-M7)
two USB Camera        - ELP-USBFHD04H-BL180
Lidar                 - RPLidar A3 & S2
```

# II. Software
> Hige-level Controller
```
Jetpack : 5.1.1 version - Ubuntu 20.04 focal
OpenCV  : 4.4.0 version - include options (GPU, CUDA, CUDNN)
ROS 2   : galactic version
```
> Low-level Controller
```
ros2 libarary
```

# 0. Set OpenCV 4.4.0
> If your device is laptop, click below link and install Opencv 4.4.0.
> 
> https://velog.io/@minukiki/Ubuntu-20.04%EC%97%90-OpenCV-4.4.0-%EC%84%A4%EC%B9%98
> 
- Uninstall old version of OpenCV
```
sudo apt-get purge  libopencv* python-opencv
# sudo apt-get autoremove
sudo find /usr/local/ -name "*opencv*" -exec rm -i {} \;
```
- Install 4.4.0 version of OpenCV
```
sudo apt-get update
sudo apt-get upgrade

sudo apt-get -y install build-essential cmake
sudo apt-get -y install pkg-config
sudo apt-get -y install libjpeg-dev libtiff5-dev libpng-dev
sudo apt-get -y install ffmpeg libavcodec-dev libavformat-dev libswscale-dev libxvidcore-dev libx264-dev libxine2-dev
sudo apt-get -y install libv4l-dev v4l-utils
sudo apt-get -y install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev 
sudo apt-get -y install libgtk-3-dev
sudo apt-get -y install mesa-utils libgl1-mesa-dri libgtkgl2.0-dev libgtkglext1-dev
sudo apt-get -y install libatlas-base-dev gfortran libeigen3-dev
sudo apt-get -y install python3-dev python3-numpy
```
- Download OpenCV 4.4.0 source file
```
mkdir OpenCV && cd OpenCV
git clone -b 4.4.0 https://github.com/opencv/opencv
git clone -b 4.4.0 https://github.com/opencv/opencv_contrib
cd opencv && mkdir build && cd build
```
- Build OpenCV 4.4.0
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
-D WITH_OPENCL=OFF \
-D WITH_CUDA=ON \
-D CUDA_ARCH_BIN=8.7 \
-D CUDA_ARCH_PTX="" \
-D WITH_CUDNN=ON \
-D WITH_CUBLAS=ON \
-D ENABLE_FAST_MATH=ON \
-D CUDA_FAST_MATH=ON \
-D OPENCV_DNN_CUDA=ON \
-D ENABLE_NEON=ON \
-D WITH_QT=OFF \
-D WITH_OPENMP=ON \
-D WITH_OPENGL=ON \
-D BUILD_TIFF=ON \
-D WITH_FFMPEG=ON \
-D WITH_GSTREAMER=ON \
-D WITH_TBB=ON \
-D BUILD_TBB=ON \
-D BUILD_TESTS=OFF \
-D WITH_V4L=ON \
-D WITH_LIBV4L=ON \
-D OPENCV_ENABLE_NONFREE=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D BUILD_NEW_PYTHON_SUPPORT=ON \
-D BUILD_opencv_python3=TRUE \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D BUILD_EXAMPLES=OFF \
..
```
```
sudo make install -j12
```

- Jetson Stats
```
sudo apt-get install python3-pip
sudo -H pip3 install jetson-stats
jetson_release
```


# 1. Install ROS2 (Galactic)
https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
## Set locale
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
## Setup Sources
```
sudo apt install software-properties-common
sudo add-apt-repository universe

#Now add the ROS 2 GPG key with apt.
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

#Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
## Install ROS2 galactic
```
sudo apt update
sudo apt upgrade
sudo apt install ros-galactic-desktop
```
## colcon install
```
sudo apt install python3-colcon-common-extensions
```
## Environment setup
```
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/galactic/setup.bash
mkdir -p ~/ros2_ws/src
```

# 2. Install vision_opencv, cv_bridge
>- Create libopencv_imgproc.so.4.4.0
>- No --allow-overriding 'packages'
 ```
 cd ~/ros2_ws/src
 git clone -b galactic https://github.com/ros-perception/vision_opencv.git
 ```
 ```
 cd vision_opencv/cv_bridge
 vim CMakeLists.txt

 # fix the below
 --find_package(OpenCV 4 QUIET
 ++find_package(OpenCV 4.4 QUIET
 ```
 ```
 cd ~/ros2_ws
 colcon build --symlink-install && . install/setup.bash
 #Then move package to other directory
 cd ~/ros2_ws/src
 mv vision_opencv ~/
 ```

# 3. Install Micro-ros-Aruduino
> On your ros2 workspace
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

# 4. Install ROS2 Packages
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

- yolo_object_detection_ros2
 ```
 cd ~/ros2_ws/src
 git clone https://github.com/AveesLab/yolo_object_detection_ros2.git
 cd yolo_object_detection_ros2/darknet
 make -j12
 ```

- scale_truck_control_ros2
 ```
 cd ~/ros2_ws/src
 git clone https://github.com/AveesLab/scale_truck_control_ros2.git
 ```

# 5. Run
- LV(Leading Vehicle) rosbag file download (onedrive-2.66GB)

[LV rosbag file download](https://kookmin-my.sharepoint.com/:f:/g/personal/dksxodnr234_kookmin_kr/EhZXPmfiH51EpsCuw_sWRSwByLEAOql4ZRPCfCS3T7Z2Fw?e=9phbm1)

- FV(Following Vehicle) rosbag file download (onedrive-2.71GB)

[FV rosbag file download](https://kookmin-my.sharepoint.com/:f:/g/personal/dksxodnr234_kookmin_kr/EnnvaWPxMitGsiyUjjfGKUkBy0j9dY6ukF5VIpipvbgtMg?e=aWr8D9)

- Rosbag run
```
ros2 bag play -l [rosbag file name].bag
```
- Ros launch
```
ros2 launch scale_truck_control_ros2 [launch file name].py
```


# alias command Setup
> ```
> sudo vim ~/.bashrc
> ```
>- add the below
> ```
> alias cw='cd ~/ros2_ws/src'
> alias cb='source ~/ros2_ws/install/setup.bash'
> alias sb='source ~/.bashrc'
> alias cm='cd ~/ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug && . install/setup.bash'
> alias eb='sudo vim ~/.bashrc'
> ```
> ```
> source ~/.bashrc
> ```
# build ROS2 packages
> ```
> cm
> ```
