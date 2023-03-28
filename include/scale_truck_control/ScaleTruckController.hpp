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
#include "scale_truck_control_ros2/msg/cmd_data.hpp"

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
    rclcpp::Subscription<scale_truck_control_ros2::msg::Lrc2xav>::SharedPtr XavSubscriber_;
    rclcpp::Subscription<scale_truck_control_ros2::msg::CmdData>::SharedPtr CmdSubscriber_;
//    rclcpp::Subscription<scale_truck_control_ros2::msg::Ocr2lrc>::SharedPtr OcrSubscriber_;
    
    //Callback Func
    void Lrc2ocrCallback(void);
    void XavSubCallback(const scale_truck_control_ros2::msg::Lrc2xav::SharedPtr msg);  
    void CmdSubCallback(const scale_truck_control_ros2::msg::CmdData::SharedPtr msg);  
//  void objectCallback(const obstacle_detector::Obstacles &msg);
    
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

    //image
    bool viewImage_;
    bool rear_camera_;
    int waitKeyDelay_;
    bool enableConsoleOutput_;
    int sync_flag_;
    bool fi_camera_ = false;
    bool beta_ = false;
    bool imageStatus_ = false;

    float AngleDegree_; // -1 ~ 1  - Twist msg angular.z
    float TargetVel_ = 0.0f; // -1 ~ 1  - Twist msg linear.x
    float SafetyVel_;
    float ResultVel_;
    float FVmaxVel_;

    //object
    int ObjSegments_;
    int ObjCircles_;
    float distance_ = 0.8f;
    float distAngle_ = 0.0f;
    float ampersand_ = 0.0f;
    float ampersand2_ = 0.0f;
    float LVstopDist_;
    float FVstopDist_;
    float TargetDist_;
    float SafetyDist_;
    uint32_t LdrErrMsg_;
    bool fi_lidar_ = false;
    bool gamma_ = false;
    float log_est_dist_ = 0.0f;
    float AngleDegree2_ = 0.0f;
    float actAngleDegree_ = 0.0f;
    float Lw_ = 0.34f;
    float Ld_offset_ = 0.0f;
    float Ld_offset2_ = 0.0f;
    float actDist_ = 0.8f;
    float estimatedDist_;
    float ppAngle_ = 0.0f;
    float x_coord_ = 0.0f;
    float y_coord_ = 0.0f;


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

