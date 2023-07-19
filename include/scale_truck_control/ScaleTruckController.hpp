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
#include "ros2_msg/msg/lrc2xav.hpp"
#include "ros2_msg/msg/ocr2lrc.hpp"
#include "ros2_msg/msg/lane2xav.hpp"
#include "ros2_msg/msg/cmd2xav.hpp"
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
    rclcpp::Publisher<ros2_msg::msg::Xav2cmd>::SharedPtr CmdPublisher_;
    rclcpp::Publisher<ros2_msg::msg::Xav2lane>::SharedPtr LanePublisher_;

    //Subscriber 
    rclcpp::Subscription<ros2_msg::msg::Lrc2xav>::SharedPtr LrcSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Cmd2xav>::SharedPtr CmdSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr objectSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Lane2xav>::SharedPtr LaneSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Lane2xav>::SharedPtr RearSubscriber_;
    
    //Callback Func
    void Lrc2ocrCallback(void);
    void LrcSubCallback(const ros2_msg::msg::Lrc2xav::SharedPtr msg);  
    void CmdSubCallback(const ros2_msg::msg::Cmd2xav::SharedPtr msg);  
    void objectCallback(const std_msgs::msg::Float32MultiArray &msg);
    void LaneSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg);
    void RearSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg);
    
    void spin();
    bool getImageStatus(void);
    void displayConsole();
    void recordData(struct timeval startTime);
    void reply(ros2_msg::msg::Xav2cmd* cmd);

    // xav->cmd
    ros2_msg::msg::Xav2cmd* cmd_data_;

    //.cpp config
    double CycleTime_ = 0.0;
    int index_;
    float CurVel_ = 0.0f;
    float RefVel_ = 0.0f;
    bool isNodeRunning_ = true;
    bool controlDone_ = false;
    bool droi_ready_ = false;
    std::string log_path_;
    struct timeval init_;

    //LaneChange  
    float laneChange();
    void checkState();
    bool lc_right_flag_ = false;
    bool lc_left_flag_ = false;
    bool lv_lc_right_ = false;
    bool lv_lc_left_ = false;
    bool fv1_lc_right_ = false;
    bool fv1_lc_left_ = false;
    bool fv2_lc_right_ = false;
    bool fv2_lc_left_ = false;
    int lane_diff_cnt_ = 150;
    int lane_diff_ = 0;
    bool lc_center_follow_ = true;
    
    //Pure Puresuit
    float purePuresuit(float tx_, float ty_);
    float Lw_ = 0.0f;
    float Ld_offset_ = 0.0f;
    float target_x_ = 0.0f;
    float target_y_ = 0.0f;

    //image
    bool enableConsoleOutput_;
    bool imageStatus_ = false;
    ros2_msg::msg::Lane2xav lane_coef_, rear_coef_;
    ros2_msg::msg::Lane2xav prev_lane_coef_;

    float AngleDegree_ = 0.0f; 
    float AngleDegree = 0.0f; 
    float AngleDegree2 = 0.0f; 
    float ppAngle_ = 0.0f; 
    int center_select_ = 0; 
    int rear_center_select_ = 0; 
    float TargetVel_ = 0.0f; 
    float SafetyVel_;
    float ResultVel_;
    float FVmaxVel_;
    vector<float> e_values_;
    float K1_ = 0.0f, K2_ = 0.0f;

    //object
    int ObjSegments_;
    int ObjCircles_;
    float distance_ = 0.8f;
    float distAngle_ = 0.0f;
    float LVstopDist_;
    float FVstopDist_;
    float TargetDist_;
    float SafetyDist_;
    std_msgs::msg::Float32MultiArray Obstacle_;

    void lanedetectInThread();
    void objectdetectInThread();
    
    //Thread
    std::thread controlThread_;
    std::thread laneDetectThread_;
    std::thread objectDetectThread_;
    std::thread tcpThread_;
    
    //mutex
    std::mutex image_mutex_;
    std::mutex object_mutex_;
    std::mutex lane_mutex_;
    std::mutex vel_mutex_;
    std::mutex dist_mutex_;
    std::mutex rep_mutex_;

    std::condition_variable cv_; // cv_.wait() 용도-> ROI거리 파악 후 Lane 그리기


}; /*end of class*/

} /* namespace scale_truck_control */

