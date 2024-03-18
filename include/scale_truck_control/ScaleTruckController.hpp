/*
 * ScaleTruckController.h
 *
 *  Created on: June 2, 2020
 *      Author: Hyeongyu Lee
 *   Institute: KMU, Avees Lab
 */

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
#include <string>
#include <condition_variable>

//ROS
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <obstacle_detector/Obstacles.h>
#include <image_transport/image_transport.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

#include "lane_detect/lane_detect.hpp"
#include "zmq_class/zmq_class.h"

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>

//custom msgs
#include <scale_truck_control/lrc2xav.h>
#include <scale_truck_control/xav2lrc.h>
#include <scale_truck_control/yolo_flag.h>
#include <yolo_object_detection/bounding_box.h>

namespace scale_truck_control {

class ScaleTruckController {
  public:
    explicit ScaleTruckController(ros::NodeHandle nh);

    ~ScaleTruckController();

  private:
    bool readParameters();

    void init();

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void rearImageCallback(const sensor_msgs::ImageConstPtr &msg);
    void objectCallback(const obstacle_detector::Obstacles &msg);
    void XavSubCallback(const scale_truck_control::lrc2xav &msg);
    void ScanErrorCallback(const std_msgs::UInt32::ConstPtr &msg);
    void bboxCallback(const yolo_object_detection::bounding_box &msg);
    void recordData(struct timeval startTime);
    void imageCompress(cv::Mat camImage, std::vector<uchar> *compImage);
    void reply(ZmqData* zmq_data);
    void requestImage(ImgData* img_data);
    void replyImage(); 
    void displayConsole();
    void spin();
    bool getImageStatus(void);

    void clusterCallback(const sensor_msgs::PointCloud &msg);
    ros::Subscriber clusterSubscriber_;
    sensor_msgs::PointCloud preceding_truck_point_;
    float pc_distance_ = 0.0f;

    ros::NodeHandle nodeHandle_;
    ros::Subscriber imageSubscriber_;
    ros::Subscriber rearImageSubscriber_;
    ros::Subscriber objectSubscriber_;
    ros::Subscriber XavSubscriber_;
    ros::Subscriber ScanSubError;	
    ros::Subscriber bboxSubscriber_;	
    ros::Publisher XavPublisher_;
    ros::Publisher runYoloPublisher_;

    image_transport::ImageTransport imageTransport_;
    image_transport::Publisher imgPublisher_;

    double CycleTime_ = 0.0;
    int index_;
    float RCMVel_;
    float RCMDist_;
    bool fi_encoder_ = false;
    bool alpha_ = false;
    bool send_rear_camera_image_ = false;
    uint8_t lrc_mode_ = 0;
    uint8_t crc_mode_ = 0;

    //image
    LaneDetect::LaneDetector laneDetector_;
    bool viewImage_;
    bool rear_camera_;
    int waitKeyDelay_;
    bool enableConsoleOutput_;
    int sync_flag_;
    bool fi_camera_ = false;
    bool beta_ = false;

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

    //bbox
    std::string name_;
    uint32_t x_ = 0;
    uint32_t y_ = 0;
    uint32_t w_ = 0;
    uint32_t h_ = 0;
    
    //ZMQ
    ZMQ_CLASS ZMQ_SOCKET_;
    ZmqData* zmq_data_;
    ImgData* img_data_;
    ImgData* backup_data_;

    //Thread
    std::thread controlThread_;
    std::thread laneDetectThread_;
    std::thread objectDetectThread_;
    std::thread tcpThread_;
    std::thread tcpImgReqThread_;
    std::thread tcpImgRepThread_;

    std::mutex image_mutex_;
    std::mutex rear_image_mutex_;
    std::mutex object_mutex_;
    std::mutex lane_mutex_;
    std::mutex vel_mutex_;
    std::mutex dist_mutex_;
    std::mutex rep_mutex_;
    std::mutex mode_mutex_;
    std::mutex bbox_mutex_;

    std::condition_variable cv_;

    obstacle_detector::Obstacles Obstacle_;
    boost::shared_mutex mutexObjectCallback_;

    bool imageStatus_ = false;
    std_msgs::Header imageHeader_;
    cv::Mat camImageCopy_, camImageTmp_;
    cv::Mat rearImageCopy_, rearImageTmp_, rearImageJPEG_, rearImageBackup_;
    bool droi_ready_ = false;

    bool isNodeRunning_ = true;
    bool controlDone_ = false;

    float CurVel_ = 0.0f;
    float RefVel_ = 0.0f;
     
    void* lanedetectInThread();
    void* objectdetectInThread();

    bool req_lv_ = false;
    bool run_yolo_ = false;
    bool tcp_img_req_ = false;
    bool tcp_img_rep_ = false;
    int req_check_ = 0;
    int rep_check_ = 0;
    double time_ = 0.0;
    double DelayTime_ = 0.0;
    std::vector<uchar> compImageSend_;
    std::vector<uchar> compImageRecv_;
    std::vector<uchar> compImageBackup_;
};

} /* namespace scale_truck_control */
