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
#include "scale_truck_control_ros2/msg/lrc2ocr.hpp"
#include "scale_truck_control_ros2/msg/ocr2lrc.hpp"
#include "scale_truck_control_ros2/msg/xav2lrc.hpp"
#include "scale_truck_control_ros2/msg/lrc2xav.hpp"
#include "scale_truck_control_ros2/msg/cmd_data.hpp"
//#include "scale_truck_control_ros2/msg/obj2xav.hpp"

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
    rclcpp::Publisher<scale_truck_control_ros2::msg::Xav2lrc>::SharedPtr LrcPublisher_;
    rclcpp::Publisher<scale_truck_control_ros2::msg::CmdData>::SharedPtr CmdPublisher_;

    //Subscriber 
    rclcpp::Subscription<scale_truck_control_ros2::msg::Lrc2xav>::SharedPtr LrcSubscriber_;
    rclcpp::Subscription<scale_truck_control_ros2::msg::CmdData>::SharedPtr CmdSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr objectSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr laneSubscriber_;
    
    //Callback Func
    void Lrc2ocrCallback(void);
    void LrcSubCallback(const scale_truck_control_ros2::msg::Lrc2xav::SharedPtr msg);  
    void CmdSubCallback(const scale_truck_control_ros2::msg::CmdData::SharedPtr msg);  
    void objectCallback(const std_msgs::msg::Float32MultiArray &msg);
    void LaneSubCallback(const std_msgs::msg::Float32MultiArray &msg);
    
    void spin();
    bool getImageStatus(void);
    void displayConsole();
    void recordData(struct timeval startTime);
    void reply(scale_truck_control_ros2::msg::CmdData* cmd);

    //CmdData
    scale_truck_control_ros2::msg::CmdData* cmd_data_;

    //.cpp config
    double CycleTime_ = 0.0;
    int index_;
    float CurVel_ = 0.0f;
    float RefVel_ = 0.0f;
    bool isNodeRunning_ = true;
    bool controlDone_ = false;
    bool droi_ready_ = false;
    std::string log_path_;

    //image
    bool viewImage_;
    int waitKeyDelay_;
    bool enableConsoleOutput_;
    int sync_flag_; // ?
    bool imageStatus_ = false;

    float AngleDegree_ = 0.0f; // -1 ~ 1  - Twist msg angular.z
    float TargetVel_ = 0.0f; // -1 ~ 1  - Twist msg linear.x
    float SafetyVel_;
    float ResultVel_;
    float FVmaxVel_;

    //object
    int ObjSegments_;
    int ObjCircles_;
    float distance_ = 0.8f;
    float distAngle_ = 0.0f;
    float LVstopDist_;
    float FVstopDist_;
    float TargetDist_;
    float SafetyDist_;
    uint32_t LdrErrMsg_;
    float Ld_offset_ = 0.0f;
    float actDist_ = 0.8f;
    std_msgs::msg::Float32MultiArray Obstacle_;

    void* lanedetectInThread();
    void* objectdetectInThread();
    
    //Thread
    std::thread controlThread_;
    std::thread laneDetectThread_;
    std::thread objectDetectThread_;
    std::thread tcpThread_;
    std::thread tcpImgReqThread_;
    std::thread tcpImgRepThread_;
    
    //mutex
    std::mutex image_mutex_;
    std::mutex rear_image_mutex_;
    std::mutex object_mutex_;
    std::mutex lane_mutex_;
    std::mutex vel_mutex_;
    std::mutex dist_mutex_;
    std::mutex rep_mutex_;
    std::mutex mode_mutex_;
    std::mutex bbox_mutex_;

    std::condition_variable cv_; // cv_.wait() 용도-> ROI거리 파악 후 Lane 그리기


}; /*end of class*/

} /* namespace scale_truck_control */

