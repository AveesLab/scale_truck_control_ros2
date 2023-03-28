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

//OpenCV
#include <cv_bridge/cv_bridge.h>

#define DATASIZE 84  // size of ZmqData 
#define REQUEST_TIMEOUT 150 // milliseconds

typedef struct LaneCoef{
	float a = 0.0f;
	float b = 0.0f;
	float c = 0.0f;
}LaneCoef;

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

