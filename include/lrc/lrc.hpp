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
#include "ros2_msg/msg/lrc2ocr.hpp"
#include "ros2_msg/msg/xav2lrc.hpp"

using namespace std::chrono_literals;

namespace LocalResiliencyCoordinator{

class LocalRC : public rclcpp::Node
{
public:
    LocalRC();

    ~LocalRC();

private:
    void init();

    //Subscriber
    rclcpp::Subscription<ros2_msg::msg::Xav2lrc>::SharedPtr XavSubscriber_;

    //Publisher
    rclcpp::Publisher<ros2_msg::msg::Lrc2ocr>::SharedPtr OcrPublisher_;

    //Callback
    void XavCallback(const ros2_msg::msg::Xav2lrc::SharedPtr msg);

    bool isNodeRunning();
    void rosPub();
//    void recordData(struct timeval *startTime);
//    void printStatus();
    void communicate();

    int index_;
    bool isNodeRunning_;
    bool EnableConsoleOutput_;
    std::string log_path_;

    float angle_degree_ = 0;
    double time_ = 0;
    double req_time_ = 0;

    std::thread lrcThread_;

    std::mutex data_mutex_;
};

}
