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
	{
		std::scoped_lock lock(dist_mutex_);
		msg.steer_angle = AngleDegree_;
	}

	LrcPublisher_->publish(msg);
	controlThread_.join();

	RCLCPP_INFO(this->get_logger(), "[ScaleTruckController] Stop.");
	//printf("[ScaleTruckController] Stop.");
}

bool ScaleTruckController::readParameters() {
	/************/
	/* SV1 Index*/
	/************/
	this->get_parameter_or("params/index", index_, 3);
	this->get_parameter_or("params/xav_log_path", log_path_, std::string("/home/avees/ros2_ws/logfiles/"));

	/***************/
	/* View Option */
	/***************/
	this->get_parameter_or("image_view/enable_console_output", enableConsoleOutput_, true);

	return true;
}

void ScaleTruckController::init() 
{
	RCLCPP_INFO(this->get_logger(), "[ScaleTruckController] init()");
	gettimeofday(&init_, NULL);

	std::string LaneTopicName;
	int LaneQueueSize;
	std::string LrcSubTopicName;
	int LrcSubQueueSize;

	std::string LrcPubTopicName;
	int LrcPubQueueSize;
	std::string CmdPubTopicName;
	int CmdPubQueueSize;
	std::string LanePubTopicName;
	int LanePubQueueSize;

	/******************************/
	/* Ros Topic Subscribe Option */
	/******************************/
	this->get_parameter_or("subscribers/lane_to_xavier/topic", LaneTopicName, std::string("lane2xav_msg"));
	this->get_parameter_or("subscribers/lane_to_xavier/queue_size", LaneQueueSize, 1);
	this->get_parameter_or("subscribers/lrc_to_xavier/topic", LrcSubTopicName, std::string("lrc2xav_msg"));
	this->get_parameter_or("subscribers/lrc_to_xavier/queue_size", LrcSubQueueSize, 1);

	/****************************/
	/* Ros Topic Publish Option */
	/****************************/
	this->get_parameter_or("publishers/xavier_to_lrc/topic", LrcPubTopicName, std::string("xav2lrc_msg"));
	this->get_parameter_or("publishers/xavier_to_lrc/queue_size", LrcPubQueueSize, 1);
	this->get_parameter_or("publishers/xavier_to_lane/topic", LanePubTopicName, std::string("xav2lane_msg"));
	this->get_parameter_or("publishers/xavier_to_lane/queue_size", LanePubQueueSize, 1);

	/************************/
	/* Ros Topic Subscriber */
	/************************/
	LaneSubscriber_ = this->create_subscription<ros2_msg::msg::Lane2xav>(LaneTopicName, LaneQueueSize, std::bind(&ScaleTruckController::LaneSubCallback, this, std::placeholders::_1));

	/***********************/
	/* Ros Topic Publisher */
	/***********************/
	LrcPublisher_ = this->create_publisher<ros2_msg::msg::Xav2lrc>(LrcPubTopicName, LrcPubQueueSize);  
	LanePublisher_=this->create_publisher<ros2_msg::msg::Xav2lane>(LanePubTopicName,LanePubQueueSize); 

	/**********************/
	/* Safety Start Setup */
	/**********************/
	distance_ = 10.f;

	/**********************************/
	/* Control & Communication Thread */
	/**********************************/
	controlThread_ = std::thread(&ScaleTruckController::spin, this);
}

void ScaleTruckController::objectdetectInThread() 
{
	float dist, dist_tmp;
	dist_tmp = 10.1f;
	ros2_msg::msg::Xav2lane Lane_;
	//dist_tmp = 0.8f;
	distance_ = dist_tmp;

	/*****************************/
	/* Dynamic ROI Distance Data */
	/*****************************/
	{
		std::scoped_lock lock(lane_mutex_);
		if(dist_tmp < 1.24f && dist_tmp > 0.30f) // 1.26 ~ 0.28
		{
			Lane_.cur_dist = (int)((1.24f - dist_tmp)*490.0f)+40;
		}
		else {
			Lane_.cur_dist = 0;
		}
		//Lane_.cur_vel = CurVel_;
		Lane_.cur_vel = 0.6; // K1_ = K2 = K_ = 0.15

		LanePublisher_->publish(Lane_);
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

		objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);
		objectdetect_thread.join();

		{
			std::scoped_lock lock(dist_mutex_);
			msg.steer_angle = AngleDegree_; 
		}
		LrcPublisher_->publish(msg);   

		std::this_thread::sleep_for(std::chrono::milliseconds(5));

		if(!isNodeRunning_) {
			controlDone_ = true;
			rclcpp::shutdown();
		}

//		recordData(init_);

		if(enableConsoleOutput_)
			displayConsole();
	}
}

void ScaleTruckController::displayConsole() {
	fflush(stdout);
	printf("\033[2J");
	printf("\033[1;1H");
	printf("Index             : %d", index_);
	printf("\033[2;1H");
	printf("Angle             : %2.3f degree\n", AngleDegree_);
}

//void ScaleTruckController::recordData(struct timeval startTime){
//	struct timeval currentTime;
//	char file_name[] = "SCT_log00.csv";
//	static char file[128] = {0x00, };
//	char buf[256] = {0x00,};
//	static bool flag = false;
//	double diff_time;
//	std::ifstream read_file;
//	std::ofstream write_file;
//	if(!flag){
//		for(int i = 0; i < 100; i++){
//			file_name[7] = i/10 + '0';  //ASCII
//			file_name[8] = i%10 + '0';
//			sprintf(file, "%s%s", log_path_.c_str(), file_name);
//			read_file.open(file);
//			if(read_file.fail()){  //Check if the file exists
//				read_file.close();
//				write_file.open(file);
//				break;
//			}
//			read_file.close();
//		}
//		write_file << "time,tar_vel,cur_vel,est_vel,tar_dist,cur_dist,est_dist,r_run_yolo_flag,y_run_yolo_flag" << std::endl; //seconds
//		flag = true;
//	}
//	if(flag){
//		std::scoped_lock lock(dist_mutex_);
//		gettimeofday(&currentTime, NULL);
//		diff_time = ((currentTime.tv_sec - startTime.tv_sec)) + ((currentTime.tv_usec - startTime.tv_usec)/1000000.0);
//		sprintf(buf, "%.10e,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f, %d, %d", diff_time, TargetVel_, CurVel_, est_vel_, TargetDist_, distance_, est_dist_, r_run_yolo_flag_, f_run_yolo_flag_);
//		write_file.open(file, std::ios::out | std::ios::app);
//		write_file << buf << std::endl;
//	}
//	write_file.close();
//}

void ScaleTruckController::LaneSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg)
{
	{
		std::scoped_lock lock(lane_mutex_);
		AngleDegree_ = msg->cur_angle;
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




