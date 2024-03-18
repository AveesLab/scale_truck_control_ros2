#pragma once

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include <arpa/inet.h>
#include <ifaddrs.h>

#include <iostream>
#include <sstream>
#include <boost/format.hpp>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>

#include <zmq.hpp>

#define DATASIZE 84

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
	//Control center = 20, CRC = 30, LRC = 10, 11, 12, LV = 0, FV1 = 1, FV2 = 2
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
  ZMQ_CLASS();
  ~ZMQ_CLASS();
  
  void* replyZMQ(ZmqData* send_data);
  std::string getIPAddress();

  std::string zipcode_;
  std::string rad_group_, dsh_group_;
  std::string udp_ip_, tcpreq_ip_, tcprep_ip0_, tcprep_ip1_, tcprep_ip2_;

  bool controlDone_;
  bool rad_flag_, dsh_flag_, req_flag_, rep_flag0_, rep_flag1_, rep_flag2_;
  ZmqData *rad_send_, *dsh_recv_, *req_send_, *req_recv_, *rep_send_, *rep_recv0_, *rep_recv1_, *rep_recv2_;
  
private:
  void init();
  bool readParameters();
  void* requestZMQ(ZmqData *send_data);
  void* radioZMQ(ZmqData *send_data);
  void* dishZMQ();
  
  std::string interface_name_;
  zmq::socket_t rad_socket_, dsh_socket_, req_socket_, rep_socket0_, rep_socket1_, rep_socket2_;
  zmq::context_t context_;
};
