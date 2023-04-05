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

  scale_truck_control_ros2::msg::Xav2lrc msg;
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
  this->get_parameter_or("params/fv_stop_dist", FVstopDist_, 0.5f); // m

  /***************/
  /* View Option */
  /***************/
  this->get_parameter_or("image_view/enable_console_output", enableConsoleOutput_, true);

  return true;
}

void ScaleTruckController::init() 
{
  RCLCPP_INFO(this->get_logger(), "[ScaleTruckController] init()");
  //printf("[ScaleTruckController] init()");

  std::string LaneTopicName;
  int LaneQueueSize;
  std::string objectTopicName;
  int objectQueueSize;
  std::string LrcSubTopicName;
  int LrcSubQueueSize;
  std::string CmdSubTopicName;
  int CmdSubQueueSize;

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
  this->get_parameter_or("subscribers/obstacle_reading/topic", objectTopicName, std::string("raw_obstacles"));
  this->get_parameter_or("subscribers/obstacle_reading/queue_size", objectQueueSize, 100);
  this->get_parameter_or("subscribers/lrc_to_xavier/topic", LrcSubTopicName, std::string("lrc2xav_msg"));
  this->get_parameter_or("subscribers/lrc_to_xavier/queue_size", LrcSubQueueSize, 1);
  this->get_parameter_or("subscribers/cmd_to_xavier/topic", CmdSubTopicName, std::string("/cmd2xav_msg"));
  this->get_parameter_or("subscribers/cmd_to_xavier/queue_size", CmdSubQueueSize, 1);

  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  this->get_parameter_or("publishers/xavier_to_lrc/topic", LrcPubTopicName, std::string("xav2lrc_msg"));
  this->get_parameter_or("publishers/xavier_to_lrc/queue_size", LrcPubQueueSize, 1);
  this->get_parameter_or("publishers/xavier_to_cmd/topic", CmdPubTopicName, std::string("/xav2cmd_msg"));
  this->get_parameter_or("publishers/xavier_to_cmd/queue_size", CmdPubQueueSize, 1);
  this->get_parameter_or("publishers/xavier_to_lane/topic", LanePubTopicName, std::string("xav2lane_msg"));
  this->get_parameter_or("publishers/xavier_to_lane/queue_size", LanePubQueueSize, 1);

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  LrcSubscriber_ = this->create_subscription<scale_truck_control_ros2::msg::Lrc2xav>(LrcSubTopicName, LrcSubQueueSize, std::bind(&ScaleTruckController::LrcSubCallback, this, std::placeholders::_1));

  CmdSubscriber_ = this->create_subscription<scale_truck_control_ros2::msg::CmdData>(CmdSubTopicName, CmdSubQueueSize, std::bind(&ScaleTruckController::CmdSubCallback, this, std::placeholders::_1));

  objectSubscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(objectTopicName, objectQueueSize, std::bind(&ScaleTruckController::objectCallback, this, std::placeholders::_1));

  LaneSubscriber_ = this->create_subscription<scale_truck_control_ros2::msg::CmdData>(LaneTopicName, LaneQueueSize, std::bind(&ScaleTruckController::LaneSubCallback, this, std::placeholders::_1));

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  LrcPublisher_ = this->create_publisher<scale_truck_control_ros2::msg::Xav2lrc>(LrcPubTopicName, LrcPubQueueSize);  
  CmdPublisher_ = this->create_publisher<scale_truck_control_ros2::msg::CmdData>(CmdPubTopicName, CmdPubQueueSize);  
  LanePublisher_ = this->create_publisher<scale_truck_control_ros2::msg::CmdData>(LanePubTopicName, LanePubQueueSize);  

  /**********************/
  /* Safety Start Setup */
  /**********************/
  distance_ = 10.f;
  distAngle_ = 0;

  lane_coef.coef.resize(3);

  /************/
  /* CMD Data */
  /************/
  cmd_data_ = new scale_truck_control_ros2::msg::CmdData;
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
void ScaleTruckController::reply(scale_truck_control_ros2::msg::CmdData* cmd)
{
  while(isNodeRunning_){
    {
      if(cmd->tar_index == 20){
        {
          std::scoped_lock lock(vel_mutex_, dist_mutex_);
          cmd->cur_vel = CurVel_;
          cmd->cur_dist = actDist_;
          cmd->cur_angle = AngleDegree_;
	}
      }
    }
    {
      std::scoped_lock lock(lane_mutex_);
      cmd->coef.resize(3);
      cmd->coef[0].a = lane_coef.coef[0].a;
      cmd->coef[0].b = lane_coef.coef[0].b;
      cmd->coef[0].c = lane_coef.coef[0].c;
      cmd->coef[1].a = lane_coef.coef[1].a;
      cmd->coef[1].b = lane_coef.coef[1].b;
      cmd->coef[1].c = lane_coef.coef[1].c;
      cmd->coef[2].a = lane_coef.coef[2].a;
      cmd->coef[2].b = lane_coef.coef[2].b;
      cmd->coef[2].c = lane_coef.coef[2].c;

//      cmd->coef[0].a  = 1.0f;
//      cmd->coef[0].b  = 1.0f;
//      cmd->coef[0].c  = 1.0f;
//      cmd->coef[1].a = 1.0f;
//      cmd->coef[1].b = 1.0f;
//      cmd->coef[1].c = 1.0f;
//      cmd->coef[2].a = 1.0f;
//      cmd->coef[2].b = 1.0f;
//      cmd->coef[2].c = 1.0f;
   }
   CmdPublisher_->publish(*cmd);

   std::this_thread::sleep_for(std::chrono::milliseconds(2));
 }
}

void ScaleTruckController::objectdetectInThread() 
{
  float dist, dist_tmp;
  dist_tmp = 10.1f;
  scale_truck_control_ros2::msg::CmdData Lane_;
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
        dist = sqrt(pow(Obstacle_.data[i], 2)+pow(Obstacle_.data[i+1], 2));
        if(dist_tmp >= dist)
        {
           dist_tmp = dist;
        }
    }	
  }
  actDist_ = dist_tmp;

  if(ObjCircles_ != 0)
  {
    distance_ = dist_tmp;
  }

  /*****************************/
  /* Dynamic ROI Distance Data */
  /*****************************/
  {
    std::scoped_lock lock(rep_mutex_, lane_mutex_, vel_mutex_);
    if(dist_tmp < 1.24f && dist_tmp > 0.30f) // 1.26 ~ 0.28
    {
      Lane_.cur_dist = (int)((1.24f - dist_tmp)*490.0f)+20;
    }
    else {
      Lane_.cur_dist = 0;
    }
    // xav -> lane (dist, vel)
    Lane_.cur_vel = CurVel_;
    LanePublisher_->publish(Lane_);
  }
  if(index_ == 0){  //LV
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
  else{  //FVs
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

void ScaleTruckController::spin() 
{
  double diff_time=0.0;
  int cnt = 0;

  scale_truck_control_ros2::msg::Xav2lrc msg;
  std::thread objectdetect_thread;

  while(!controlDone_ && rclcpp::ok()) {
    struct timeval start_time, end_time;
    gettimeofday(&start_time, NULL);

    objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);
    objectdetect_thread.join();

    msg.tar_vel = ResultVel_;  //Xavier to LRC and LRC to OpenCR
    {
      std::scoped_lock lock(dist_mutex_);
      msg.steer_angle = AngleDegree_; // get from objectThread
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

    if(enableConsoleOutput_)
      displayConsole();

    gettimeofday(&end_time, NULL);
    diff_time += ((end_time.tv_sec - start_time.tv_sec) * 1000.0) + ((end_time.tv_usec - start_time.tv_usec) / 1000.0);
    cnt++;

    CycleTime_ = diff_time / (double)cnt;

    printf("cnt: %d\n", cnt);
    if (cnt > 3000){
            diff_time = 0.0;
            cnt = 0;
    }

  }
}

void ScaleTruckController::displayConsole() {
  printf("\033[2J");
  printf("\033[1;1H");
  printf("Angle           : %2.3f degree\n", AngleDegree_);
  printf("Refer Vel       : %3.3f m/s\n", RefVel_);
  printf("Send Vel        : %3.3f m/s\n", ResultVel_);
  printf("Tar/Cur Vel     : %3.3f / %3.3f m/s\n", TargetVel_, CurVel_);
  printf("Tar/Cur Dist    : %3.3f / %3.3f m\n", TargetDist_, distance_);
//  printf("\nK1/K2           : %3.3f / %3.3f", laneDetector_.K1_, laneDetector_.K2_);
//  if(ObjCircles_ > 0) {
//    printf("\nCirs            : %d", ObjCircles_);
//    printf("\nDistAng         : %2.3f degree", distAngle_);
//  }
//  if(ObjSegments_ > 0) {
//    printf("\nSegs            : %d", ObjSegments_);
//  }
  printf("Cycle Time      : %3.3f ms\n", CycleTime_);
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
    //write_file << "time,tar_vel,act_dist,est_dist" << endl; //seconds
    write_file << "time,tar_vel,angle,beta" << std::endl; //seconds
    flag = true;
  }
  if(flag){
    std::scoped_lock lock(dist_mutex_);
    gettimeofday(&currentTime, NULL);
    diff_time = ((currentTime.tv_sec - startTime.tv_sec)) + ((currentTime.tv_usec - startTime.tv_usec)/1000000.0);
    //sprintf(buf, "%.10e,%.3f,%.3f", diff_time, TargetVel_, AngleDegree_);
    write_file.open(file, std::ios::out | std::ios::app);
    write_file << buf << std::endl;
  }
  write_file.close();
}

void ScaleTruckController::LaneSubCallback(const scale_truck_control_ros2::msg::CmdData::SharedPtr msg)
{
  {
    std::scoped_lock lock(lane_mutex_);
    lane_coef.coef = msg->coef;
    AngleDegree_ = msg->cur_angle;
  }
}

void ScaleTruckController::objectCallback(const std_msgs::msg::Float32MultiArray &msg) 
{
  {
    std::scoped_lock lock(object_mutex_);
    Obstacle_ = msg;
  }
}

void ScaleTruckController::LrcSubCallback(const scale_truck_control_ros2::msg::Lrc2xav::SharedPtr msg)
{
  {
    std::scoped_lock lock(vel_mutex_);
    CurVel_ = msg->cur_vel;
  }
}

void ScaleTruckController::CmdSubCallback(const scale_truck_control_ros2::msg::CmdData::SharedPtr msg)
{
  {
    std::scoped_lock lock(rep_mutex_);
    if(msg->tar_index == 0){  //LV 
      TargetVel_ = msg->tar_vel;
      TargetDist_ = msg->tar_dist;
    }
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




