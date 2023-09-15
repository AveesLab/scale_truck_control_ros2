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
#include "ros2_msg/msg/boundingbox.hpp"
#include "ros2_msg/msg/yoloflag.hpp"

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
    rclcpp::Publisher<ros2_msg::msg::Yoloflag>::SharedPtr runYoloPublisher_;

    //Subscriber 
    rclcpp::Subscription<ros2_msg::msg::Lrc2xav>::SharedPtr LrcSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Cmd2xav>::SharedPtr CmdSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr objectSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Lane2xav>::SharedPtr LaneSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Lane2xav>::SharedPtr RearSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Boundingbox>::SharedPtr YoloSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Boundingbox>::SharedPtr RearYoloSubscriber_;
    
    //Callback Func
    void Lrc2ocrCallback(void);
    void LrcSubCallback(const ros2_msg::msg::Lrc2xav::SharedPtr msg);  
    void CmdSubCallback(const ros2_msg::msg::Cmd2xav::SharedPtr msg);  
    void objectCallback(const std_msgs::msg::Float32MultiArray &msg);
    void LaneSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg);
    void RearSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg);
    void YoloSubCallback(const ros2_msg::msg::Boundingbox::SharedPtr msg);
    void RearYoloSubCallback(const ros2_msg::msg::Boundingbox::SharedPtr msg);
    
    void spin();
    bool getImageStatus(void);
    void displayConsole();
    void recordData(struct timeval startTime);
    void reply(ros2_msg::msg::Xav2cmd* cmd);
    void isLaneChangeCommandReceived();
    void isFV2Detected();
    void isFV1Detected();
    void isLVDetected();
    void isAreaSafe(int indexArea);
    void adjustTargetVelocity();
    void setLaneChangeFlags(bool no_object = false);
    float lowPassFilter(double sampling_time, float est_value, float prev_res);


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
    bool cmd_lv_lc_right_ = false;
    bool cmd_lv_lc_left_ = false;
    bool cmd_fv1_lc_right_ = false;
    bool cmd_fv1_lc_left_ = false;
    bool cmd_fv2_lc_right_ = false;
    bool cmd_fv2_lc_left_ = false;
    int lane_diff_cnt_ = 150;
    int lane_diff_ = 0;
    bool lc_center_follow_ = true;
    double origin_lateral_err = 0;
    double lateral_err = 0;

    // FOR ICRA
    float ICRA_dist = 10.1;
    bool  wroi_flag_ = false;
    float cur_vel = 0.0;

    //RSS
    void RSS();
    float a_max_accel = 0.0f;
    float a_min_brake = 0.0f; 
    float a_max_brake = 0.0f;
    float p_ = 0.0f;
    float rss_min_dist_= 0.0f;
    float rrss_min_dist_= 0.0f;

    float est_vel_ = 0.0f;
    float r_est_vel_ = 0.0f;

    float lv_est_vel_ = 0.0f;
    float fv1_est_vel_ = 0.0f;
    float fv2_est_vel_ = 0.0f;

    float lv_r_est_vel_ = 0.0f;
    float fv1_r_est_vel_ = 0.0f;
    float fv2_r_est_vel_ = 0.0f;

    float est_dist_ = 0.0f; 
    float r_est_dist_ = 0.0f;

    float lv_cur_dist_ = 0.0f;
    float fv1_cur_dist_ = 0.0f;
    float fv2_cur_dist_ = 0.0f;

    float lv_est_dist_ = 0.0f;
    float fv1_est_dist_ = 0.0f;
    float fv2_est_dist_ = 0.0f;

    float lv_rss_dist_ = 0.0f;
    float fv1_rss_dist_ = 0.0f;
    float fv2_rss_dist_ = 0.0f;

    float lv_r_est_dist_ = 0.0f;
    float fv1_r_est_dist_ = 0.0f;
    float fv2_r_est_dist_ = 0.0f;

    float lv_r_rss_dist_ = 0.0f;
    float fv1_r_rss_dist_ = 0.0f;
    float fv2_r_rss_dist_ = 0.0f;

    bool req_flag_ = false;

    //bbox
    std::string name_;
    std::string r_name_;
    uint32_t x_ = 0;
    uint32_t y_ = 0;
    uint32_t w_ = 0;
    uint32_t h_ = 0;
    uint32_t rx_ = 0;
    uint32_t ry_ = 0;
    uint32_t rw_ = 0;
    uint32_t rh_ = 0;

    int isbboxReady_ = 3;   //isbboxObject? 1:Yes,  2:No, 3:No_Msg 
    int r_isbboxReady_ = 3;

    int lv_bbox_ready_ = 3;
    int fv1_bbox_ready_ = 3;
    int fv2_bbox_ready_ = 3;

    int lv_r_bbox_ready_ = 3;
    int fv1_r_bbox_ready_ = 3;
    int fv2_r_bbox_ready_ = 3;

    bool f_run_yolo_flag_ = false;
    bool r_run_yolo_flag_ = false;

    //image
    bool enableConsoleOutput_;
    bool imageStatus_ = false;
    ros2_msg::msg::Lane2xav lane_coef_, r_lane_coef_;
    ros2_msg::msg::Lane2xav prev_lane_coef_;

    float AngleDegree_ = 0.0f; 
    float AngleDegree = 0.0f; 
    float AngleDegree2 = 0.0f; 
    float ppAngle_ = 0.0f; 
    int center_select_ = 0; 
    int r_center_select_ = 0; 
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
    std::mutex rlane_mutex_;
    std::mutex vel_mutex_;
    std::mutex dist_mutex_;
    std::mutex rep_mutex_;
    std::mutex bbox_mutex_;
    std::mutex rbbox_mutex_;
    std::mutex rss_mutex_;

    std::condition_variable cv_; // cv_.wait() 용도-> ROI거리 파악 후 Lane 그리기


}; /*end of class*/

} /* namespace scale_truck_control */

