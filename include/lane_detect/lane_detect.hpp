#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudafilters.hpp>
#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <ros/ros.h>
#include <scale_truck_control/lane_coef.h>
#include <time.h>


using namespace cv;
using namespace std;

namespace LaneDetect {

class LaneDetector{
public:
	LaneDetector(ros::NodeHandle nh);
	~LaneDetector(void);

	//Timer
	struct timeval start_, end_;

	float display_img(Mat _frame, int _delay, bool _view);
	void get_steer_coef(float vel);
	float K1_, K2_, K3_, K4_;
	int distance_ = 0;
	float est_dist_ = 0.0f;
	float est_pose_ = 0.0f;
	scale_truck_control::lane_coef lane_coef_;
	Mat frame_;
	float rotation_angle_ = 0.0f;
	float lateral_offset_ = 0.0f;
	Point left_, right_;
	float y_offset_ = 0.0f;

	/********** bbox *********/
	std::string name_;
	unsigned int x_ = 0, y_ = 0, w_ = 0, h_ = 0;
	Point center_, warp_center_;
	float SteerAngle2_ = 0.0f;
	float log_e1_ = 0.0f;
	float log_el_ = 0.0f;
	float vel_ = 0.0f;

	/***** fault signal *****/
	bool beta_ = false, gamma_ = false;

private:
	void LoadParams(void);
	int arrMaxIdx(int hist[], int start, int end, int Max);
	Mat polyfit(vector<int> x_val, vector<int> y_val);
	Mat detect_lines_sliding_window(Mat _frame, bool _view);
	Point warpPoint(Point center, Mat trans);
	float lowPassFilter(double sampling_time, float est_value, float prev_res);
	Mat estimateDistance(Mat frame, Mat trans, double cycle_time, bool _view);
	Mat estimatePose(Mat frame, double cycle_time, bool _view);
	Mat draw_lane(Mat _sliding_frame, Mat _frame);
	Mat drawBox(Mat frame);
	void controlSteer();
	void clear_release();

	ros::NodeHandle nodeHandle_;

	/********** Camera calibration **********/
	Mat map1_, map2_, f_map1_, f_map2_, r_map1_, r_map2_;
	int canny_thresh1_, canny_thresh2_;

	/********** Lane_detect ***********/
	vector<Point2f> corners_, fROIcorners_, rROIcorners_;
	vector<Point2f> warpCorners_, fROIwarpCorners_, rROIwarpCorners_;
	float wide_extra_upside_[2], wide_extra_downside_[2];

	int last_Llane_base_;
	int last_Rlane_base_;

	vector<int> left_lane_inds_;
	vector<int> right_lane_inds_;
	vector<int> left_x_;
	vector<int> left_y_;
	vector<int> right_x_;
	vector<int> right_y_;
	vector<int> center_x_;
	vector<int> center_y_;
	
	vector<Point> left_lane_, right_lane_;

	Mat left_coef_;
	Mat right_coef_;
	Mat center_coef_;
	float left_curve_radius_;
	float right_curve_radius_;
	float center_position_;
	float SteerAngle_;
	float eL_height_, trust_height_, e1_height_, lp_;
	float eL_height2_;
	float K_;
	double a_[5], b_[5];
	vector<float> e_values_;

	/********** PID control ***********/
	int prev_lane_, prev_pid_;
	double Kp_term_, Ki_term_, Kd_term_, err_, prev_err_, I_err_, D_err_, result_;

	int width_, height_;
	bool option_; // dynamic ROI
	int threshold_;
	double diff_;

	int crop_x_, crop_y_, crop_width_, crop_height_;
};

}
