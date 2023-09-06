#include "scale_truck_control/ScaleTruckController.hpp"

using namespace std::chrono_literals;

namespace scale_truck_control{

ScaleTruckController::ScaleTruckController()
	: Node("scale_truck_control_node", rclcpp::NodeOptions()
			.allow_undeclared_parameters(true)
			.automatically_declare_parameters_from_overrides(true)) 	
{
	if (!readParameters()) {
		rclcpp::shutdown();  
	}

	//RCLCPP_INFO(this->get_logger(), "waitKeyDelay1 : %d\n", waitKeyDelay_);
	init();
}

ScaleTruckController::~ScaleTruckController() {
	isNodeRunning_ = false;

	ros2_msg::msg::Xav2lrc msg;
	msg.tar_vel = ResultVel_;
	{
		std::scoped_lock lock(dist_mutex_);
		msg.steer_angle = AngleDegree_;
		msg.cur_dist = distance_;
	}
	{
		std::scoped_lock lock(rep_mutex_);
		msg.tar_dist = TargetDist_; 
	}

	LrcPublisher_->publish(msg);
	controlThread_.join();
	tcpThread_.join();

	delete cmd_data_;

	RCLCPP_INFO(this->get_logger(), "[ScaleTruckController] Stop.");
	//printf("[ScaleTruckController] Stop.");
}

bool ScaleTruckController::readParameters() {
	/***********/
	/* LV Index*/
	/***********/
	this->get_parameter_or("params/index", index_, 0);
	this->get_parameter_or("params/xav_log_path", log_path_, std::string("/home/avees/ros2_ws/logfiles/"));

	/*******************/
	/* Velocity Option */
	/*******************/
	this->get_parameter_or("params/target_vel", TargetVel_, 0.0f); // m/s
	this->get_parameter_or("params/safety_vel", SafetyVel_, 0.45f); // m/s
	this->get_parameter_or("params/fv_max_vel", FVmaxVel_, 1.20f); // m/s
	this->get_parameter_or("params/ref_vel", RefVel_, 0.0f); // m/s

	/*******************/
	/* Distance Option */
	/*******************/
	this->get_parameter_or("params/target_dist", TargetDist_, 0.8f); // m
	this->get_parameter_or("params/safety_dist", SafetyDist_, 1.5f); // m
	this->get_parameter_or("params/lv_stop_dist", LVstopDist_, 0.5f); // m
	this->get_parameter_or("params/fv_stop_dist", FVstopDist_, 0.3f); // m

	/***************/
	/* View Option */
	/***************/
	this->get_parameter_or("image_view/enable_console_output", enableConsoleOutput_, true);
	this->get_parameter_or("image_view/lane_change_right", lc_right_flag_, false); // cmd
	this->get_parameter_or("image_view/lane_change_left", lc_left_flag_, false); // cmd

	/**************/
	/* RSS Option */
	/**************/
	this->get_parameter_or("rss/a_max_accel", a_max_accel, 0.0f); 
	this->get_parameter_or("rss/a_max_brake", a_max_brake, 0.0f); 
	this->get_parameter_or("rss/a_min_brake", a_min_brake, 0.0f); 
	this->get_parameter_or("rss/response_time", p_, 0.0f); 

	return true;
}

void ScaleTruckController::init() 
{
	RCLCPP_INFO(this->get_logger(), "[ScaleTruckController] init()");
	gettimeofday(&init_, NULL);

	std::string LaneTopicName;
	int LaneQueueSize;
	std::string RearTopicName;
	int RearQueueSize;
	std::string objectTopicName;
	int objectQueueSize;
	std::string LrcSubTopicName;
	int LrcSubQueueSize;
	std::string CmdSubTopicName;
	int CmdSubQueueSize;
	std::string YoloSubTopicName;
	int YoloSubQueueSize;
	std::string RearYoloSubTopicName;
	int RearYoloSubQueueSize;

	std::string LrcPubTopicName;
	int LrcPubQueueSize;
	std::string CmdPubTopicName;
	int CmdPubQueueSize;
	std::string LanePubTopicName;
	int LanePubQueueSize;
	std::string runYoloPubTopicName;
	int runYoloPubQueueSize;

	/******************************/
	/* Ros Topic Subscribe Option */
	/******************************/
	this->get_parameter_or("subscribers/lane_to_xavier/topic", LaneTopicName, std::string("lane2xav_msg"));
	this->get_parameter_or("subscribers/lane_to_xavier/queue_size", LaneQueueSize, 1);
	this->get_parameter_or("subscribers/rearlane_to_xavier/topic", RearTopicName, std::string("rear2xav_msg"));
	this->get_parameter_or("subscribers/rearlane_to_xavier/queue_size", RearQueueSize, 1);
	this->get_parameter_or("subscribers/obstacle_reading/topic", objectTopicName, std::string("raw_obstacles"));
	this->get_parameter_or("subscribers/obstacle_reading/queue_size", objectQueueSize, 100);
	this->get_parameter_or("subscribers/lrc_to_xavier/topic", LrcSubTopicName, std::string("lrc2xav_msg"));
	this->get_parameter_or("subscribers/lrc_to_xavier/queue_size", LrcSubQueueSize, 1);
	this->get_parameter_or("subscribers/cmd_to_xavier/topic", CmdSubTopicName, std::string("/cmd2xav_msg"));
	this->get_parameter_or("subscribers/cmd_to_xavier/queue_size", CmdSubQueueSize, 1);
	this->get_parameter_or("subscribers/yolo_to_xavier/topic", YoloSubTopicName, std::string("yolo_object_detection/Boundingbox"));
	this->get_parameter_or("subscribers/yolo_to_xavier/queue_size", YoloSubQueueSize, 1);
	this->get_parameter_or("subscribers/rearyolo_to_xavier/topic", RearYoloSubTopicName, std::string("rear_yolo_object_detection/Boundingbox"));
	this->get_parameter_or("subscribers/rearyolo_to_xavier/queue_size", RearYoloSubQueueSize, 1);

	/****************************/
	/* Ros Topic Publish Option */
	/****************************/
	this->get_parameter_or("publishers/xavier_to_lrc/topic", LrcPubTopicName, std::string("xav2lrc_msg"));
	this->get_parameter_or("publishers/xavier_to_lrc/queue_size", LrcPubQueueSize, 1);
	this->get_parameter_or("publishers/xavier_to_cmd/topic", CmdPubTopicName, std::string("xav2cmd_msg"));
	this->get_parameter_or("publishers/xavier_to_cmd/queue_size", CmdPubQueueSize, 1);
	this->get_parameter_or("publishers/xavier_to_lane/topic", LanePubTopicName, std::string("xav2lane_msg"));
	this->get_parameter_or("publishers/xavier_to_lane/queue_size", LanePubQueueSize, 1);
	this->get_parameter_or("publishers/xavier_to_Yolo/topic", runYoloPubTopicName, std::string("run_yolo_flag"));
	this->get_parameter_or("publishers/xavier_to_Yolo/queue_size", runYoloPubQueueSize, 1);

	/************************/
	/* Ros Topic Subscriber */
	/************************/
	LrcSubscriber_ = this->create_subscription<ros2_msg::msg::Lrc2xav>(LrcSubTopicName, LrcSubQueueSize, std::bind(&ScaleTruckController::LrcSubCallback, this, std::placeholders::_1));

	CmdSubscriber_ = this->create_subscription<ros2_msg::msg::Cmd2xav>(CmdSubTopicName, CmdSubQueueSize, std::bind(&ScaleTruckController::CmdSubCallback, this, std::placeholders::_1));

	objectSubscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(objectTopicName, objectQueueSize, std::bind(&ScaleTruckController::objectCallback, this, std::placeholders::_1));

	LaneSubscriber_ = this->create_subscription<ros2_msg::msg::Lane2xav>(LaneTopicName, LaneQueueSize, std::bind(&ScaleTruckController::LaneSubCallback, this, std::placeholders::_1));

	RearSubscriber_ = this->create_subscription<ros2_msg::msg::Lane2xav>(RearTopicName, RearQueueSize, std::bind(&ScaleTruckController::RearSubCallback, this, std::placeholders::_1));

	YoloSubscriber_ = this->create_subscription<ros2_msg::msg::Boundingbox>(YoloSubTopicName, YoloSubQueueSize, std::bind(&ScaleTruckController::YoloSubCallback, this, std::placeholders::_1));

	RearYoloSubscriber_ = this->create_subscription<ros2_msg::msg::Boundingbox>(RearYoloSubTopicName, RearYoloSubQueueSize, std::bind(&ScaleTruckController::RearYoloSubCallback, this, std::placeholders::_1));

	/***********************/
	/* Ros Topic Publisher */
	/***********************/
	LrcPublisher_ = this->create_publisher<ros2_msg::msg::Xav2lrc>(LrcPubTopicName, LrcPubQueueSize);  
	CmdPublisher_ = this->create_publisher<ros2_msg::msg::Xav2cmd>(CmdPubTopicName, CmdPubQueueSize);  
	LanePublisher_=this->create_publisher<ros2_msg::msg::Xav2lane>(LanePubTopicName,LanePubQueueSize); 
	runYoloPublisher_=this->create_publisher<ros2_msg::msg::Yoloflag>(runYoloPubTopicName,runYoloPubQueueSize);  

	/**********************/
	/* Safety Start Setup */
	/**********************/
	distance_ = 10.f;
	distAngle_ = 0;

	lane_coef_.coef.resize(3);
	r_lane_coef_.coef.resize(3);
	prev_lane_coef_.coef.resize(3);

	e_values_.resize(3);

	/************/
	/* CMD Data */
	/************/
	cmd_data_ = new ros2_msg::msg::Xav2cmd;
	cmd_data_->src_index = index_;
	cmd_data_->tar_index = 20;  //Control center

	/**********************************/
	/* Control & Communication Thread */
	/**********************************/
	controlThread_ = std::thread(&ScaleTruckController::spin, this);
	tcpThread_ = std::thread(&ScaleTruckController::reply, this, cmd_data_); //send to PC_Commend 
}

/***************/
/* cmd publish */
/***************/
void ScaleTruckController::reply(ros2_msg::msg::Xav2cmd* cmd)
{
	while(isNodeRunning_)
	{
		{
			std::scoped_lock lock(vel_mutex_, dist_mutex_, bbox_mutex_);
			cmd->tar_vel = TargetVel_;
			cmd->cur_vel = CurVel_;
			cmd->cur_dist = distance_;
			cmd->cur_angle = AngleDegree_;
			cmd->est_dist = est_dist_;
			cmd->est_dist = est_vel_;
			cmd->r_est_dist = r_est_dist_;
			cmd->bbox_ready = isbboxReady_;
			cmd->r_bbox_ready = r_isbboxReady_;
			cmd->req_flag = req_flag_;
		}
		{
			std::scoped_lock lock(lane_mutex_, r_lane_mutex_);
			cmd->coef.resize(3);
			cmd->coef[0].a = lane_coef_.coef[0].a;
			cmd->coef[0].b = lane_coef_.coef[0].b;
			cmd->coef[0].c = lane_coef_.coef[0].c;
			cmd->coef[1].a = lane_coef_.coef[1].a;
			cmd->coef[1].b = lane_coef_.coef[1].b;
			cmd->coef[1].c = lane_coef_.coef[1].c;
			cmd->coef[2].a = lane_coef_.coef[2].a;
			cmd->coef[2].b = lane_coef_.coef[2].b;
			cmd->coef[2].c = lane_coef_.coef[2].c;

			cmd->rear_coef.resize(3);
			cmd->rear_coef[0].a = r_lane_coef_.coef[0].a;
			cmd->rear_coef[0].b = r_lane_coef_.coef[0].b;
			cmd->rear_coef[0].c = r_lane_coef_.coef[0].c;
			cmd->rear_coef[1].a = r_lane_coef_.coef[1].a;
			cmd->rear_coef[1].b = r_lane_coef_.coef[1].b;
			cmd->rear_coef[1].c = r_lane_coef_.coef[1].c;
			cmd->rear_coef[2].a = r_lane_coef_.coef[2].a;
			cmd->rear_coef[2].b = r_lane_coef_.coef[2].b;
			cmd->rear_coef[2].c = r_lane_coef_.coef[2].c;
		}
		{
			std::scoped_lock lock(rep_mutex_);
			cmd->lc_right_flag = lc_right_flag_;
			cmd->lc_left_flag = lc_left_flag_;
		}
		{
			std::scoped_lock lock(rep_mutex_);
		}

		CmdPublisher_->publish(*cmd);

		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}
}

void ScaleTruckController::checkState() {
	ros2_msg::msg::Yoloflag yolo_flag_msg;
	int i = 480; //height
	int car_position = 320; // width/2
	double lane_diff = 999.0; 
	double prev_center_base = 0, cur_center_base = 0;

	{
		std::scoped_lock lock(lane_mutex_);
		if(sizeof(prev_lane_coef_.coef) != 0 && sizeof(lane_coef_.coef) != 0) 
		{
			//prev_center_base = (prev_lane_coef_.coef[2].a * pow(i, 2)) + (prev_lane_coef_.coef[2].b * i) + prev_lane_coef_.coef[2].c;
			prev_center_base = car_position;
			cur_center_base =  (lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c;
			lane_diff = abs(cur_center_base - prev_center_base); 
			lane_diff_ = lane_diff;
		}

		if(lane_diff <= 10 && lane_diff >= 0) {
			lane_diff_cnt_ -= 1;
			if(lane_diff_cnt_ <= 0) {
				lane_diff_cnt_ = 150;

				/* right lane change */
				if(lc_right_flag_ == true) {
					if(cmd_fv2_lc_right_ == true) {
						cmd_fv2_lc_right_ = false;
						lc_right_flag_ = false;
//						yolo_flag_msg.f_run_yolo = f_run_yolo_flag_ = true; 
					}
					else if (cmd_fv1_lc_right_ == true) {
						cmd_fv1_lc_right_ = false;
						lc_right_flag_ = false;
//						yolo_flag_msg.f_run_yolo = f_run_yolo_flag_ = true; 
					}
					else {
						cmd_lv_lc_right_ = false;
						lc_right_flag_ = false;
//						yolo_flag_msg.f_run_yolo = f_run_yolo_flag_ = false; 
					}
				}
				/* left lane change */
				else if(lc_left_flag_ == true) {
					if(cmd_fv2_lc_left_ == true) {
						cmd_fv2_lc_left_ = false;
						lc_left_flag_ = false;
//						yolo_flag_msg.f_run_yolo = f_run_yolo_flag_ = true; 
					}
					else if (cmd_fv1_lc_left_ == true) {
						cmd_fv1_lc_left_ = false;
						lc_left_flag_ = false;
//						yolo_flag_msg.f_run_yolo = f_run_yolo_flag_ = true; 
					}
					else {
						cmd_lv_lc_left_ = false;
						lc_left_flag_ = false;
//						yolo_flag_msg.f_run_yolo = f_run_yolo_flag_ = false; 
					}
				}
				runYoloPublisher_->publish(yolo_flag_msg);
			}
		}
	}  
}

void ScaleTruckController::objectdetectInThread() 
{
	float dist, dist_tmp;
	dist_tmp = 10.1f;
	ros2_msg::msg::Xav2lane Lane_;
	/**************/
	/* Lidar Data */
	/**************/
	{
		std::scoped_lock lock(lane_mutex_, object_mutex_);         
		ObjCircles_ = Obstacle_.data.size();    
	}   
	for(int i=0; i < ObjCircles_; i+=3)
	{
		if(Obstacle_.data[i]!=0 || Obstacle_.data[i+1]!=0)
		{
			//dist = sqrt(pow(Obstacle_.data[i], 2)+pow(Obstacle_.data[i+1], 2));
			dist = sqrt(pow(Obstacle_.data[i], 2));
			if(dist_tmp >= dist)
			{
				dist_tmp = dist;
			}
		}	
	}

	if(ObjCircles_ != 0)
	{
		distance_ = dist_tmp;
	}

	/*****************************/
	/* Dynamic ROI Distance Data */
	/*****************************/
	{
		std::scoped_lock lock(rep_mutex_, lane_mutex_, vel_mutex_, bbox_mutex_);
		if(dist_tmp < 1.24f && dist_tmp > 0.30f) // 1.26 ~ 0.28
		{
			Lane_.cur_dist = (int)((1.24f - dist_tmp)*490.0f)+40;
			//Lane_.cur_dist = (int)((1.24f - dist_tmp)*390.0f)+40;
		}
		else {
			Lane_.cur_dist = 0;
		}
		Lane_.cur_vel = CurVel_;
		Lane_.lc_right_flag = lc_right_flag_;
		Lane_.lc_left_flag = lc_left_flag_;

		if(isbboxReady_ == 1){
			Lane_.name = name_;
			Lane_.x = x_;
			Lane_.y = y_;
			Lane_.w = w_;
			Lane_.h = h_;    
		}
		if(r_isbboxReady_ == 1) {
			Lane_.r_name = r_name_;
			Lane_.rx = rx_;
			Lane_.ry = ry_;
			Lane_.rw = rw_;
			Lane_.rh = rh_;    
		}
		LanePublisher_->publish(Lane_);
	}

	if(index_ == 0) //LV
	{  
		std::scoped_lock lock(rep_mutex_, dist_mutex_);
		if(distance_ <= LVstopDist_) {
			// Emergency Brake
			ResultVel_ = 0.0f;
		}
		else if (distance_ <= SafetyDist_){
			float TmpVel_ = (ResultVel_-SafetyVel_)*((distance_-LVstopDist_)/(SafetyDist_-LVstopDist_))+SafetyVel_;
			if (TargetVel_ < TmpVel_){
				ResultVel_ = TargetVel_;
			}
			else{
				ResultVel_ = TmpVel_;
			}
		}
		else{
			ResultVel_ = TargetVel_;
		}
	}
	else //FVs
	{  
		std::scoped_lock lock(rep_mutex_, dist_mutex_);
		if ((distance_ <= FVstopDist_) || (TargetVel_ <= 0.1f)){
			// Emergency Brake
			ResultVel_ = 0.0f;
		}
		else {
			ResultVel_ = TargetVel_;
		}
	}
}

void ScaleTruckController::isLaneChangeCommandReceived() {
	{
		std::scoped_lock lock(rep_mutex_);
		if(index_ == 2 && (cmd_fv2_lc_right_ || cmd_fv2_lc_left_)) isFV2Detected();
		else if(index_ == 1 && (cmd_fv1_lc_right_ || cmd_fv1_lc_left_)) isFV2Detected();
		else if(index_ == 0 && (cmd_lv_lc_right_ || cmd_lv_lc_left_)) isFV1Detected();
	}
}

void ScaleTruckController::isFV2Detected() {
	{
		std::scoped_lock lock(rep_mutex_);
		if (index_ == 2) { //FV2
			if(fv2_bbox_ready_ == 1 || fv2_r_bbox_ready_ == 1) isAreaSafe(2); 
			else if(fv2_bbox_ready_ == 2 && fv2_r_bbox_ready_ == 2) isFV1Detected();
			else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "index:%d, FV2 No front & rearYolo Msg\n", index_);
		}
		else if (index_ == 1) { //FV1
			if(fv2_bbox_ready_ == 1) isAreaSafe(2);
			else if(fv2_bbox_ready_ == 2) isFV1Detected();
			else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "index:%d, FV2 No frontYolo Msg\n", index_);
		}
	}
}

void ScaleTruckController::isFV1Detected() {
	{
		std::scoped_lock lock(rep_mutex_);
		if(index_ == 2) {
			if(fv1_r_bbox_ready_ == 1) isAreaSafe(1);	
			else if(fv1_r_bbox_ready_ == 2) setLaneChangeFlags();
			else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "FV1 No rearYolo Msg\n");
		}
		else if(index_ == 1) {
			if(fv1_bbox_ready_ == 1) isAreaSafe(1);
		  else if(fv1_bbox_ready_ == 2) setLaneChangeFlags();	
			else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "FV1 No frontYolo Msg\n");
		}
		else {
			if(fv1_bbox_ready_ == 1) isAreaSafe(1);
		  else if(fv1_bbox_ready_ == 2) isLVDetected();	
			else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "FV1 No frontYolo Msg\n");
		}
	}
}

void ScaleTruckController::isLVDetected() {
	{
		std::scoped_lock lock(rep_mutex_);
		if(lv_bbox_ready_ == 1) isAreaSafe(0);
		else if (lv_bbox_ready_ == 2) setLaneChangeFlags(); // LV LC
		else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "LV No frontYolo Msg\n");
	}
}

void ScaleTruckController::isAreaSafe(int indexArea) {
	{
		std::scoped_lock lock(rep_mutex_);
		// car_length_ = 1.15
		int d1_ = fv2_cur_dist_ + 1.15f + 0.23f;  
		int d2_ = fv1_cur_dist_ + 1.15f + 0.23f; 

		/*******/
		/* FV2 */
		/*******/
		if (index_ == 2) {
			//FV1 Area is OK?
			if(indexArea == 1) { 
				if(fv1_r_est_dist_ != 0 && fv2_cur_dist_ != 0) {
					if(fv1_r_est_dist_ <= d1_) adjustTargetVelocity();
					else setLaneChangeFlags(); // FV2 LC
				}
				else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No (fv1_r_est, fv2_cur)_dist\n");
			} 
			else if(indexArea == 2) { 
				//FV2 Rear && Front Area is OK?
				if (fv2_r_bbox_ready_ == 1 && fv2_bbox_ready_ == 1) {
					if(fv2_r_est_dist_ != 0 && fv2_r_rss_dist_ != 0 && fv2_est_dist_ != 0 && fv2_rss_dist_ != 0) {
						if(fv2_r_est_dist_ <= fv2_r_rss_dist_) adjustTargetVelocity();
						else if (fv2_est_dist_ <= fv2_rss_dist_) adjustTargetVelocity();
						else isFV1Detected();
					}
					else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No (fv2_r_est, fv2_est, fv2_r_rss, fv2_rss)_dist\n");
				}
				//FV2 Rear Area is OK?
				else if (fv2_r_bbox_ready_ == 1 && fv2_bbox_ready_ != 1) {
					if(fv2_r_est_dist_ != 0 && fv2_r_rss_dist_ != 0) {
						if(fv2_r_est_dist_ <= fv2_r_rss_dist_) adjustTargetVelocity();
						else isFV1Detected();
					}
					else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No (fv2_r_est, fv2_r_rss)_dist\n");
				}
				//FV2 Front Area is OK?
				else if (fv2_bbox_ready_ == 1) {
					if(fv2_est_dist_ != 0 && fv2_rss_dist_ != 0) {
						if(fv2_est_dist_ <= fv2_rss_dist_) adjustTargetVelocity();
						else isFV1Detected();
					}
					else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No (fv2_est, fv2_rss)_dist\n");
				}
			} 
		}
		/*******/
		/* FV1 */
		/*******/
		else if (index_ == 1) {
			if(indexArea == 1) {
				if(fv1_est_dist_ != 0 && fv1_rss_dist_ != 0) {
					if(fv1_est_dist_ <= fv1_rss_dist_) adjustTargetVelocity();
					else setLaneChangeFlags(); // FV1 LC
				}
				else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No (fv1_est, fv1_rss)_dist\n");
			} 
			else if(indexArea == 2) {
				if(fv2_est_dist_ != 0 && fv2_cur_dist_ != 0) {
					if(fv2_est_dist_ <= d1_) adjustTargetVelocity();
					else isFV1Detected();
				}	
				else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No (fv2_est, fv2_cur)_dist\n");
			} 
		}
		/*******/
		/* LV  */
		/*******/
		else if (index_ == 0) {
			if(indexArea == 0) {
				if(lv_est_dist_ != 0 && lv_rss_dist_ != 0) {
					if(lv_est_dist_ <= lv_rss_dist_) adjustTargetVelocity();
					else setLaneChangeFlags(); // LV LC
				}
				else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No (lv_est, lv_rss)_dist\n");
			}
			else if(indexArea == 1) {
				if(fv1_est_dist_ != 0 && fv1_cur_dist_ != 0) {
					if(fv1_est_dist_ <= d2_) adjustTargetVelocity();
					else isLVDetected(); 
				}
				else RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No (fv1_est, fv1_cur)_dist\n");
			} 
		}
	}
}

void ScaleTruckController::adjustTargetVelocity() {
	{
		std::scoped_lock lock(vel_mutex_, rep_mutex_);
		if(index_ == 2) {
			if (fv2_r_bbox_ready_ == 1) {
				if(CurVel_ >= fv2_r_est_vel_) {
					TargetVel_ += 0.01f;
					if(TargetVel_ >= FVmaxVel_) TargetVel_ = FVmaxVel_;
				}
				else {
					TargetVel_ -= 0.01f;
					if(TargetVel_ <= SafetyVel_) TargetVel_ = SafetyVel_;	
				}
			}
			else if (fv2_bbox_ready_ == 1) {
				if(CurVel_ >= fv2_est_vel_) {
					TargetVel_ += 0.01f;
					if(TargetVel_ >= FVmaxVel_) TargetVel_ = FVmaxVel_;
				}
				else {
					TargetVel_ -= 0.01f;
					if(TargetVel_ <= SafetyVel_) TargetVel_ = SafetyVel_;	
				}
			}
			else if (fv1_r_bbox_ready_ == 1) {
				if(CurVel_ >= fv1_r_est_vel_ ) {
					TargetVel_ += 0.01f;
					if(TargetVel_ >= FVmaxVel_) TargetVel_ = FVmaxVel_;
				}
				else {
					TargetVel_ -= 0.01f;
					if(TargetVel_ <= SafetyVel_) TargetVel_ = SafetyVel_;	
				}
			
			}
		}
		else if (index_ == 0 || index_ == 1) {
			TargetVel_ -= 0.01;
			if(TargetVel_ <= SafetyVel_) TargetVel_ = SafetyVel_;	
		}
	}
}

void ScaleTruckController::setLaneChangeFlags(bool no_object) {
	ros2_msg::msg::Yoloflag yolo_flag_msg;
	{
		std::scoped_lock lock(rep_mutex_);
		if(cmd_fv2_lc_right_ || cmd_fv1_lc_right_ || cmd_lv_lc_right_) {
			lc_right_flag_ = true;
			yolo_flag_msg.f_run_yolo = f_run_yolo_flag_ = true; 
		}
		else if(cmd_fv2_lc_left_ || cmd_fv1_lc_left_ || cmd_lv_lc_left_) {
			lc_left_flag_ = true;
			yolo_flag_msg.f_run_yolo = f_run_yolo_flag_ = true; 
		}

		runYoloPublisher_->publish(yolo_flag_msg);
	}
}

void ScaleTruckController::spin() 
{
	double diff_time=0.0;
	int cnt = 0;

	ros2_msg::msg::Xav2lrc msg;
	std::thread objectdetect_thread;

	while(!controlDone_ && rclcpp::ok()) {
		struct timeval start_time, end_time;
		gettimeofday(&start_time, NULL);

		{
			std::scoped_lock lock(lane_mutex_, r_lane_mutex_, vel_mutex_);
  		if((est_vel_ != 0 && est_dist_ != 0)) {
				RSS(est_dist_, CurVel_, est_vel_);
  		}
  		if((r_est_vel_ != 0 && r_est_dist_ != 0)) {
				RSS(r_est_dist_, CurVel_, r_est_vel_);
			}
		}

		objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);
		objectdetect_thread.join();

		/* 데이터들 들어왔는지 확인 후 판단 */
		/* lv,fv1,fv2 | bbox | cur_dist | est_dist | */
		if (lc_right_flag_== false || lc_left_flag_ == false)	isLaneChangeCommandReceived();	

		{
			std::scoped_lock lock(rep_mutex_);
			if((AngleDegree2 != 0) && (lc_right_flag_ || lc_left_flag_)) { 
				AngleDegree_ = AngleDegree2;
				checkState(); // is lane change complete ?	
			}
			else{ 
				AngleDegree_ = AngleDegree;
			}
		}
		msg.tar_vel = ResultVel_;  //Xavier to LRC and LRC to OpenCR
		{
			std::scoped_lock lock(dist_mutex_);
			msg.steer_angle = AngleDegree_; 
			msg.cur_dist = distance_;       
		}
		{
			std::scoped_lock lock(rep_mutex_);
			msg.tar_dist = TargetDist_;
		}    
		LrcPublisher_->publish(msg);   

		std::this_thread::sleep_for(std::chrono::milliseconds(5));

		if(!isNodeRunning_) {
			controlDone_ = true;
			rclcpp::shutdown();
		}

		recordData(init_);

		if(enableConsoleOutput_)
			displayConsole();

		gettimeofday(&end_time, NULL);
		diff_time += ((end_time.tv_sec - start_time.tv_sec) * 1000.0) + ((end_time.tv_usec - start_time.tv_usec) / 1000.0);
		cnt++;

		CycleTime_ = diff_time / (double)cnt;

		if (cnt > 3000){
			diff_time = 0.0;
			cnt = 0;
		}

	}
}

bool ScaleTruckController::RSS(float est_dist, float cf_vel, float cr_vel) {
	if (f_run_yolo_flag_) {
		float d_long_min = 0.0f;
		
		/*이때는 a_max_accel = 0으로 해도 무방*/
		d_long_min = cr_vel*p_ + (a_max_accel*pow(p_,2)/2) + (pow((cr_vel+p_*a_max_accel),2)/(2*a_min_brake)) - (pow(cf_vel,2)/(2*a_max_brake));
		d_long_min = max(d_long_min, 0.0f);

		//	RCLCPP_INFO(this->get_logger(), "est_dist : %.3f", est_dist);
		//	RCLCPP_INFO(this->get_logger(), "cf_vel : %.3f", cf_vel);
		//	RCLCPP_INFO(this->get_logger(), "cr_vel : %.3f", cr_vel);
		//	RCLCPP_INFO(this->get_logger(), "d_long_min : %.3f\n", d_long_min);
		if(index_ == 0) lv_rss_dist_ = d_long_min;
		else if(index_ == 1) fv1_rss_dist_ = d_long_min;
		else if(index_ == 2) fv2_rss_dist_ = d_long_min;
	}

	if (r_run_yolo_flag_) {
		float d_long_min = 0.0f;

		d_long_min = cr_vel*p_ + (a_max_accel*pow(p_,2)/2) + (pow((cr_vel+p_*a_max_accel),2)/(2*a_min_brake)) - (pow(cf_vel,2)/(2*a_max_brake));
		d_long_min = max(d_long_min, 0.0f);

		if(index_ == 0) lv_r_rss_dist_ = d_long_min;
		else if(index_ == 1) fv1_r_rss_dist_ = d_long_min;
		else if(index_ == 2) fv2_r_rss_dist_ = d_long_min;
	}
}

void ScaleTruckController::displayConsole() {
	fflush(stdout);
	printf("\033[2J");
	printf("\033[1;1H");
	printf("Index             : %d", index_);
	printf("\033[2;1H");
	printf("Angle             : %2.3f degree", AngleDegree);
	printf("\033[3;1H");
	printf("lcAngle           : %2.3f degree", AngleDegree2);
	printf("\033[4;1H");
	printf("center_select     : %d", center_select_);
	printf("\033[5;1H");
	printf("E2/E1 lc_flag     : %d / %d", lc_left_flag_, lc_right_flag_);
	printf("\033[6;1H");
	printf("lc_center_follow  : %d", lc_center_follow_);
	printf("\033[7;1H");
	printf("k1/k2             : %3.3f %3.3f", K1_, K2_);
	printf("\033[8;1H");
	printf("e1/eL             : %3.3f %3.3f", e_values_[1], e_values_[0]);
	printf("\033[9;1H");
	printf("Tar/Cur/Est Vel   : %3.3f / %3.3f m/s", TargetVel_, CurVel_);
	printf("\033[10;1H");
	printf("Tar/Cur/Est Dist  : %3.3f / %3.3f m", TargetDist_, distance_);
	printf("\033[11;1H");
	printf("F/R Est_Vel       : %3.3f / %3.3f m/s", est_vel_, r_est_vel_);
	printf("\033[12;1H");
	printf("F/R Est_Dist      : %3.3f / %3.3f m", est_dist_, r_est_dist_);
	printf("\033[13;1H");
	printf("lane diff         : %d", lane_diff_);
	printf("\033[14;1H");
	printf("lane diff cnt     : %d", lane_diff_cnt_);
	printf("\033[15;1H");
	printf("Cycle Time        : %3.3f ms\n", CycleTime_);
}

void ScaleTruckController::recordData(struct timeval startTime){
	struct timeval currentTime;
	char file_name[] = "SCT_log00.csv";
	static char file[128] = {0x00, };
	char buf[256] = {0x00,};
	static bool flag = false;
	double diff_time;
	std::ifstream read_file;
	std::ofstream write_file;
	if(!flag){
		for(int i = 0; i < 100; i++){
			file_name[7] = i/10 + '0';  //ASCII
			file_name[8] = i%10 + '0';
			sprintf(file, "%s%s", log_path_.c_str(), file_name);
			read_file.open(file);
			if(read_file.fail()){  //Check if the file exists
				read_file.close();
				write_file.open(file);
				break;
			}
			read_file.close();
		}
		//write_file << "time,tar_vel,cur_vel,tar_dist,cur_dist,normal_angle,lc_angle,lc_right_flag,lc_left_flag" << std::endl; //seconds
		write_file << "time,tar_vel,cur_vel,est_vel,tar_dist,cur_dist,est_dist,r_run_yolo_flag,y_run_yolo_flag" << std::endl; //seconds
		flag = true;
	}
	if(flag){
		std::scoped_lock lock(dist_mutex_);
		gettimeofday(&currentTime, NULL);
		diff_time = ((currentTime.tv_sec - startTime.tv_sec)) + ((currentTime.tv_usec - startTime.tv_usec)/1000000.0);
		//    sprintf(buf, "%.10e,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d", diff_time, TargetVel_, CurVel_, TargetDist_, distance_, AngleDegree_, AngleDegree2, lc_right_flag_, lc_left_flag_, lc_center_follow_);
		sprintf(buf, "%.10e,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f, %d, %d", diff_time, TargetVel_, CurVel_, est_vel_, TargetDist_, distance_, est_dist_, r_run_yolo_flag_, f_run_yolo_flag_);
		write_file.open(file, std::ios::out | std::ios::app);
		write_file << buf << std::endl;
	}
	write_file.close();
}

void ScaleTruckController::LaneSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg)
{
	{
		std::scoped_lock lock(lane_mutex_);
		lane_coef_.coef = msg->coef;
		AngleDegree = msg->cur_angle;
		AngleDegree2 = msg->cur_angle2;
		center_select_ = msg->center_select;
		lc_center_follow_ = msg->lc_center_follow;
		e_values_ = msg->e_values;
		K1_ = msg->k1;
		K2_ = msg->k2;
		est_dist_ = msg->est_dist;
		est_vel_ = msg->est_vel;
	}
}

void ScaleTruckController::RearSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg)
{
	{
		std::scoped_lock lock(r_lane_mutex_);
		r_lane_coef_.coef = msg->coef;
		//r_center_select_ = msg->center_select;
		r_est_dist_ = msg->est_dist;
		r_est_vel_ = msg->est_vel;
	}
}

void ScaleTruckController::YoloSubCallback(const ros2_msg::msg::Boundingbox::SharedPtr msg)
{
	{
		std::scoped_lock lock(bbox_mutex_);
		name_ = msg->name;
		if ((msg->x > 0 && msg->x < 640) && \
				(msg->y > 0 && msg->y < 480) && \
				(msg->w > 0 && msg->w < 640) && \
				(msg->h > 0 && msg->h < 480)){
			x_ = msg->x;
			y_ = msg->y;
			w_ = msg->w;
			h_ = msg->h;
			isbboxReady_ = 1;	// isbboxObject? 1:Yes,  2:No, 3:No_Msg 
		}
		else if(name_ == "no_object") {
			isbboxReady_ = 2;
		}
		else isbboxReady_ = 3;
	}
}

void ScaleTruckController::RearYoloSubCallback(const ros2_msg::msg::Boundingbox::SharedPtr msg)
{
	// isbboxObject? 1:Yes,  2:No, 3:No_Msg 
	{
		std::scoped_lock lock(bbox_mutex_);
		r_name_ = msg->name;
		if ((msg->x > 0 && msg->x < 640) && \
				(msg->y > 0 && msg->y < 480) && \
				(msg->w > 0 && msg->w < 640) && \
				(msg->h > 0 && msg->h < 480)){
			rx_ = msg->x;
			ry_ = msg->y;
			rw_ = msg->w;
			rh_ = msg->h;
			r_isbboxReady_ = 1;
		}
		else if(name_ == "no_object") {
			r_isbboxReady_ = 2;
		}
		else r_isbboxReady_ = 3;
	}
}

void ScaleTruckController::objectCallback(const std_msgs::msg::Float32MultiArray &msg) 
{
	{
		std::scoped_lock lock(object_mutex_);
		Obstacle_ = msg;
	}
}

void ScaleTruckController::LrcSubCallback(const ros2_msg::msg::Lrc2xav::SharedPtr msg)
{
	{
		std::scoped_lock lock(vel_mutex_, rep_mutex_);
		CurVel_ = msg->cur_vel;
		TargetVel_ = msg->tar_vel;
		TargetDist_ = msg->tar_dist;
	}
}

void ScaleTruckController::CmdSubCallback(const ros2_msg::msg::Cmd2xav::SharedPtr msg)
{
	ros2_msg::msg::Yoloflag yolo_flag_msg;
	{
		std::scoped_lock lock(rep_mutex_);
		/******/
		/* LV */
		/******/
		if(index_ == 0) {   
			TargetVel_ = msg->tar_vel;
			TargetDist_ = msg->tar_dist;

			cmd_lv_lc_right_ = msg->lv_lc_right; 
			if(cmd_lv_lc_right_){
				prev_lane_coef_ = lane_coef_; 
				//lc_right_flag_ = true;
			}

			cmd_lv_lc_left_ = msg->lv_lc_left;
			if(cmd_lv_lc_left_) {
				prev_lane_coef_ = lane_coef_; 
				//lc_left_flag_ = true;
			}

			if(msg->fv2_lc_right) {
				yolo_flag_msg.r_run_yolo = r_run_yolo_flag_ = true; 
			}

			if(msg->fv2_lc_left) {
				yolo_flag_msg.r_run_yolo = r_run_yolo_flag_ = true; 
			}
		}
		/*******/
		/* FV1 */
		/*******/
		else if(index_ == 1) {   
			cmd_fv1_lc_right_ = msg->fv1_lc_right; 
			if(cmd_fv1_lc_right_){
				prev_lane_coef_ = lane_coef_; 
				//lc_right_flag_ = true;
			}

			cmd_fv1_lc_left_ = msg->fv1_lc_left;
			if(cmd_fv1_lc_left_) {
				prev_lane_coef_ = lane_coef_; 
				//lc_left_flag_ = true;
			}

			if(msg->fv2_lc_right) {
				yolo_flag_msg.r_run_yolo = r_run_yolo_flag_ = true; 
			}

			if(msg->fv2_lc_left) {
				yolo_flag_msg.r_run_yolo = r_run_yolo_flag_ = true; 
			}
		}
		/*******/
		/* FV2 */
		/*******/
		else if(index_ == 2) { 
			cmd_fv2_lc_right_ = msg->fv2_lc_right;
			if(cmd_fv2_lc_right_) {
				req_flag_ = true;
				prev_lane_coef_ = lane_coef_; 
				//lc_right_flag_ = true;
				yolo_flag_msg.f_run_yolo = f_run_yolo_flag_ = true; 
				yolo_flag_msg.r_run_yolo = r_run_yolo_flag_ = true; 
			}
			
			cmd_fv2_lc_left_ = msg->fv2_lc_left;
			if(cmd_fv2_lc_left_) {
				req_flag_ = true;
				prev_lane_coef_ = lane_coef_; 
				//lc_left_flag_ = true;
				yolo_flag_msg.f_run_yolo = f_run_yolo_flag_ = true; 
				yolo_flag_msg.r_run_yolo = r_run_yolo_flag_ = true; 
			}
		}

		lv_cur_dist_ = msg->lv_cur_dist;
		fv1_cur_dist_ = msg->fv1_cur_dist;
		fv2_cur_dist_ = msg->fv2_cur_dist;

		lv_est_dist_ = msg->lv_est_dist;
		fv1_est_dist_ = msg->fv1_est_dist;
		fv2_est_dist_ = msg->fv2_est_dist;

		lv_r_est_dist_ = msg->lv_r_est_dist;
		fv1_r_est_dist_ = msg->fv1_r_est_dist;
		fv2_r_est_dist_ = msg->fv2_r_est_dist;

		lv_est_vel_ = msg->lv_est_vel;
		fv1_est_vel_ = msg->fv1_est_vel;
		fv2_est_vel_ = msg->fv2_est_vel;

		lv_r_est_vel_ = msg->lv_r_est_vel;
		fv1_r_est_vel_ = msg->fv1_r_est_vel;
		fv2_r_est_vel_ = msg->fv2_r_est_vel;

		lv_bbox_ready_ = msg->lv_bbox_ready;
		fv1_bbox_ready_ = msg->fv1_bbox_ready;
		fv2_bbox_ready_ = msg->fv2_bbox_ready;

		lv_r_bbox_ready_ = msg->lv_r_bbox_ready;
		fv1_r_bbox_ready_ = msg->fv1_r_bbox_ready;
		fv2_r_bbox_ready_ = msg->fv2_r_bbox_ready;

		runYoloPublisher_->publish(yolo_flag_msg);
	}
}


} /* namespace scale_truck_control */


int main(int argc, char* argv[]){
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = std::make_shared<scale_truck_control::ScaleTruckController>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}




