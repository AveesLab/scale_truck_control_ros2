#pragma once

//C++
#include <iostream>
#include <pthread.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <boost/thread/thread.hpp>
#include <vector>
#include <sys/time.h>
#include <condition_variable>
#include <functional>
#include <memory>
#include <fstream>
#include <string>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

//custom msgs
#include "scale_truck_control_ros2/msg/lrc2ocr.hpp"
#include "scale_truck_control_ros2/msg/ocr2lrc.hpp"
#include "scale_truck_control_ros2/msg/xav2lrc.hpp"
#include "scale_truck_control_ros2/msg/lrc2xav.hpp"
#include "scale_truck_control_ros2/msg/lv2_fv.hpp"

using namespace std::chrono_literals;

namespace LocalResiliencyCoordinator{

class LocalRC : public rclcpp::Node
{
public:
    LocalRC();

    ~LocalRC();

private:
    void init();

    rclcpp::Subscription<scale_truck_control_ros2::msg::Ocr2lrc>::SharedPtr OcrSubscriber_;
    rclcpp::Subscription<scale_truck_control_ros2::msg::Xav2lrc>::SharedPtr XavSubscriber_;
    rclcpp::Subscription<scale_truck_control_ros2::msg::LV2FV>::SharedPtr LVSubscriber_;

    rclcpp::Publisher<scale_truck_control_ros2::msg::Lrc2ocr>::SharedPtr OcrPublisher_;
    rclcpp::Publisher<scale_truck_control_ros2::msg::Lrc2xav>::SharedPtr XavPublisher_;
    rclcpp::TimerBase::SharedPtr OcrPublishTimer_;
    rclcpp::Publisher<scale_truck_control_ros2::msg::LV2FV>::SharedPtr FVPublisher_;

    void Lrc2ocrCallback(void);
    void OcrCallback(const scale_truck_control_ros2::msg::Ocr2lrc::SharedPtr msg);
    void XavCallback(const scale_truck_control_ros2::msg::Xav2lrc::SharedPtr msg);
    void LVCallback(const scale_truck_control_ros2::msg::LV2FV::SharedPtr msg);

    bool isNodeRunning();
    void rosPub();
    void recordData(struct timeval *startTime);
    void printStatus();
    void communicate();
    void radio();

    int index_;
    bool isNodeRunning_;
    bool EnableConsoleOutput_;
    std::string log_path_;
    float a_, b_, l_;
    float epsilon_;
    float rotation_angle_ = 0.0f;
    float lateral_offset_ = 0.0f;

    float angle_degree_ = 0;
    float cur_dist_ = 0.8f;
    float tar_dist_ = 0.8f;
    float ref_vel_ = 0;
    float cur_vel_ = 0;
    float tar_vel_ = 0;
    float est_vel_ = 0;
    float hat_vel_ = 0;
    float sat_vel_ = 0;
    double time_ = 0;
    double req_time_ = 0;

    std::thread lrcThread_;
    std::thread udpThread_;
    std::thread tcpThread_;
    std::mutex data_mutex_;
    std::mutex time_mutex_;
};

}
