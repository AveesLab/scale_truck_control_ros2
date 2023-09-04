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
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

//custom msgs
#include "ros2_msg/msg/xav2lrc.hpp"
#include "ros2_msg/msg/xav2lane.hpp"
#include "ros2_msg/msg/xav2cmd.hpp"
#include "ros2_msg/msg/lrc2ocr.hpp"
#include "ros2_msg/msg/ocr2lrc.hpp"
#include "ros2_msg/msg/lane2xav.hpp"
#include "ros2_msg/msg/lane.hpp"

using namespace std;
using namespace std::chrono_literals;

namespace scale_truck_control 
{

class ScaleTruckController : public rclcpp::Node 
{
public:
    ScaleTruckController();
    
    ~ScaleTruckController();

private:
    bool readParameters();

    void init();
 
    //Publisher 
    rclcpp::Publisher<ros2_msg::msg::Xav2lrc>::SharedPtr LrcPublisher_;
    rclcpp::Publisher<ros2_msg::msg::Xav2lane>::SharedPtr LanePublisher_;

    //Subscriber 
    rclcpp::Subscription<ros2_msg::msg::Lane2xav>::SharedPtr LaneSubscriber_;
    
    //Callback Func
    void LaneSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg);
    
    void spin();
    bool getImageStatus(void);
    void displayConsole();
//    void recordData(struct timeval startTime);

    //.cpp config
    double CycleTime_ = 0.0;
    int index_;
    bool isNodeRunning_ = true;
    bool controlDone_ = false;
    bool droi_ready_ = false;
    std::string log_path_;
    struct timeval init_;

    //image
    bool enableConsoleOutput_;
    bool imageStatus_ = false;
    float AngleDegree_ = 0.0f; 

    float distance_;

    //Thread
    void objectdetectInThread();
    std::thread controlThread_;
    std::thread objectDetectThread_;
    
    //mutex
    std::mutex image_mutex_;
    std::mutex lane_mutex_;
    std::mutex dist_mutex_;

}; /*end of class*/

} /* namespace scale_truck_control */

