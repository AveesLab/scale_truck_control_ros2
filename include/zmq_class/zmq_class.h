#pragma once

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <ifaddrs.h>

#include <iostream>
#include <sstream>
#include <boost/format.hpp>
#include <thread>
#include <chrono>
#include <mutex>

#include <zmq.hpp>

#include <ros/ros.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

#define DATASIZE 84  // size of ZmqData 
#define REQUEST_TIMEOUT 150 // milliseconds

typedef struct LaneCoef{
	float a = 0.0f;
	float b = 0.0f;
	float c = 0.0f;
}LaneCoef;

typedef struct ImgData{
	uint8_t src_index = 255;
	uint8_t tar_index = 255;

	struct timeval startTime;

	u_char comp_image[100000] = {0,};
	size_t size = 0;
}ImgData;

typedef struct ZmqData{
	//Control center = 15, 20, CRC = 30, LRC = 10, 11, 12, LV = 0, FV1 = 1, FV2 = 2
	uint8_t src_index = 255;
	uint8_t tar_index = 255;
	
	//sensor failure
	bool fi_encoder = false;
	bool fi_camera = false;
	bool fi_lidar = false;
	bool alpha = false;
	bool beta = false;
	bool gamma = false;

	//flag to send rear camera sensor image
	bool send_rear_camera_image = false;

	float ref_vel = 0.0f;
	float cur_vel = 0.0f;
	float cur_dist = 0.0f;
	float cur_angle = 0.0f;
	float tar_vel = 0.0f;
	float tar_dist = 0.0f;
	float est_vel = 0.0f;  //estimated velocity
	float preceding_truck_vel = 0.0f;

	//TM = 0, RCM = 1, GDM = 2
	uint8_t lrc_mode = 0;
	uint8_t crc_mode = 0;

	LaneCoef coef[3];
}ZmqData;

class ZMQ_CLASS{
public:
  explicit ZMQ_CLASS(ros::NodeHandle nh);
  ~ZMQ_CLASS();
  
  void* requestZMQ(ZmqData *send_data);
  void* replyZMQ(ZmqData *send_data);
  void* radioZMQ(ZmqData *send_data);
  void* dishZMQ();
  void* requestImageZMQ(ImgData *send_data, ImgData *backup_data);
  void* replyImageZMQ();
  std::string getIPAddress();

  std::string zipcode_;
  std::string rad_group_, dsh_group_;
  std::string udp_ip_, tcpreq_ip_, tcprep_ip_, tcpreq_img_ip_, tcprep_img_ip_;

  bool controlDone_;
  bool rad_flag_, dsh_flag_, req_flag_, rep_flag_;
  bool req_img_flag_, rep_img_flag_;
  ZmqData *dsh_recv_, *req_send_, *req_recv_, *rep_send_, *rep_recv_;
  ImgData *img_recv_;
  int img_socket_change_count_ = 0;

private:
  ros::NodeHandle nodeHandle_;
  void init();
  bool readParameters();
  void* subscribeZMQ();
  void* publishZMQ();
  
  std::string interface_name_;
  zmq::context_t context_;
  zmq::socket_t req_socket_, rep_socket_, rad_socket_, dsh_socket_;
  zmq::socket_t req_img_socket_, rep_img_socket_;
};
