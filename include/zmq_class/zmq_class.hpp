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

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

//OpenCV
//#include <cv_bridge/cv_bridge.h>

#define DATASIZE 84  // size of ZmqData 
#define REQUEST_TIMEOUT 150 // milliseconds

typedef struct LaneCoef{
	float a = 0.0f;
	float b = 0.0f;
	float c = 0.0f;
}LaneCoef;

typedef struct ZmqData{
	//Control center = 15, 20, LRC = 10, 11, 12, LV = 0, FV1 = 1, FV2 = 2
	uint8_t src_index = 255;
	uint8_t tar_index = 255;
	
	float ref_vel = 0.0f;
	float cur_vel = 0.0f;
	float cur_dist = 0.0f;
	float cur_angle = 0.0f;
	float tar_vel = 0.0f;
	float tar_dist = 0.0f;
	float est_vel = 0.0f;  //estimated velocity

	LaneCoef coef[3];
}ZmqData;

