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

  /*****************/
  /* Pure Puresuit */
  /*****************/
  this->get_parameter_or("params/Lw", Lw_, 0.34f);
  this->get_parameter_or("params/LdOffset", Ld_offset_, 0.0f);

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
  std::string TruckSubTopicName;
  int TruckSubQueueSize;

  std::string LrcPubTopicName;
  int LrcPubQueueSize;
  std::string CmdPubTopicName;
  int CmdPubQueueSize;
  std::string LanePubTopicName;
  int LanePubQueueSize;
  std::string TruckPubTopicName;
  int TruckPubQueueSize;

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
  this->get_parameter_or("subscribers/truck_to_truck/topic", TruckSubTopicName, std::string("/truck2truck_msg"));
  this->get_parameter_or("subscribers/truck_to_truck/queue_size", TruckSubQueueSize, 1);

  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  this->get_parameter_or("publishers/xavier_to_lrc/topic", LrcPubTopicName, std::string("xav2lrc_msg"));
  this->get_parameter_or("publishers/xavier_to_lrc/queue_size", LrcPubQueueSize, 1);
  this->get_parameter_or("publishers/xavier_to_cmd/topic", CmdPubTopicName, std::string("xav2cmd_msg"));
  this->get_parameter_or("publishers/xavier_to_cmd/queue_size", CmdPubQueueSize, 1);
  this->get_parameter_or("publishers/xavier_to_lane/topic", LanePubTopicName, std::string("xav2lane_msg"));
  this->get_parameter_or("publishers/xavier_to_lane/queue_size", LanePubQueueSize, 1);
  this->get_parameter_or("publishers/each_truck/topic", TruckPubTopicName, std::string("/truck2truck_msg"));
  this->get_parameter_or("publishers/each_truck/queue_size", TruckPubQueueSize, 1);

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  LrcSubscriber_ = this->create_subscription<ros2_msg::msg::Lrc2xav>(LrcSubTopicName, LrcSubQueueSize, std::bind(&ScaleTruckController::LrcSubCallback, this, std::placeholders::_1));

  CmdSubscriber_ = this->create_subscription<ros2_msg::msg::CmdData>(CmdSubTopicName, CmdSubQueueSize, std::bind(&ScaleTruckController::CmdSubCallback, this, std::placeholders::_1));

  objectSubscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(objectTopicName, objectQueueSize, std::bind(&ScaleTruckController::objectCallback, this, std::placeholders::_1));

  LaneSubscriber_ = this->create_subscription<ros2_msg::msg::CmdData>(LaneTopicName, LaneQueueSize, std::bind(&ScaleTruckController::LaneSubCallback, this, std::placeholders::_1));

  if(index_ == 0 || index_ == 1){
    TruckSubscriber_ = this->create_subscription<ros2_msg::msg::CmdData>(TruckSubTopicName, TruckSubQueueSize, std::bind(&ScaleTruckController::TruckSubCallback, this, std::placeholders::_1));
  }

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  LrcPublisher_ = this->create_publisher<ros2_msg::msg::Xav2lrc>(LrcPubTopicName, LrcPubQueueSize);  
  CmdPublisher_ = this->create_publisher<ros2_msg::msg::CmdData>(CmdPubTopicName, CmdPubQueueSize);  
  LanePublisher_ = this->create_publisher<ros2_msg::msg::CmdData>(LanePubTopicName, LanePubQueueSize);  
  if(index_ == 1 || index_ == 2){
    TruckPublisher_ = this->create_publisher<ros2_msg::msg::CmdData>(TruckPubTopicName, TruckPubQueueSize);  
  }

  /**********************/
  /* Safety Start Setup */
  /**********************/
  distance_ = 10.f;
  distAngle_ = 0;

  lane_coef_.coef.resize(3);

  /************/
  /* CMD Data */
  /************/
  cmd_data_ = new ros2_msg::msg::CmdData;
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
void ScaleTruckController::reply(ros2_msg::msg::CmdData* cmd)
{
  while(isNodeRunning_){
    {
      if(cmd->tar_index == 20){
        {
          std::scoped_lock lock(vel_mutex_, dist_mutex_);
          cmd->cur_vel = CurVel_;
          cmd->cur_dist = distance_;
          cmd->cur_angle = AngleDegree_;
	}
      }
    }
    {
      std::scoped_lock lock(lane_mutex_);
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
   }
   {
     std::scoped_lock lock(rep_mutex_);
//     cmd->lv_lc_right = lv_lc_right_;
//     cmd->fv1_lc_right = fv1_lc_right_;
//     cmd->fv2_lc_right = fv2_lc_right_;
   }

   CmdPublisher_->publish(*cmd);

   std::this_thread::sleep_for(std::chrono::milliseconds(2));
 }
}

float ScaleTruckController::laneChange()
{
  float tx_ = 0.0f, ty_ = 0.0f;
  int i = 480, lane_diff = 0; //height
  static int cnt = 10;
  ros2_msg::msg::CmdData msg;
  ros2_msg::msg::Xav2lrc lv_data;

  {
    std::scoped_lock lock(lane_mutex_);
    tx_ = target_x_;
    ty_ = target_y_;

    if(fv2_lc_right_){
      ppAngle_ = purePuresuit(tx_, ty_);
      lane_diff = ((prev_lane_coef_.coef[2].a * pow(i, 2)) + (prev_lane_coef_.coef[2].b * i) + prev_lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c);

      if(abs(lane_diff) < 30 && abs(lane_diff) > 0) {
	cnt -= 1;
	if(cnt == 0) {
          fv2_lc_right_ = false;
	  lc_flag_ = false;
          msg.fv1_lc_right = true;  
	  TruckPublisher_->publish(msg);
	}
      }
      else
        cnt = 10;
    }
    else if(fv1_lc_right_){
      ppAngle_ = purePuresuit(tx_, ty_);
      lane_diff = ((prev_lane_coef_.coef[2].a * pow(i, 2)) + (prev_lane_coef_.coef[2].b * i) + prev_lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c);

      if(abs(lane_diff) < 30 && abs(lane_diff) > 0) {
	cnt -= 1;
	if(cnt == 0) {
          fv1_lc_right_ = false;
	  lc_flag_ = false;
          msg.lv_lc_right = true;  
	  TruckPublisher_->publish(msg);
	}
      }
      else
        cnt = 10;
    }
    else if(lv_lc_right_){
      ppAngle_ = purePuresuit(tx_, ty_);
      lane_diff = ((prev_lane_coef_.coef[2].a * pow(i, 2)) + (prev_lane_coef_.coef[2].b * i) + prev_lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c);

      if(abs(lane_diff) < 30 && abs(lane_diff) > 0) {
	cnt -= 1;
	if(cnt == 0) {
          lv_lc_right_ = false;
	  lc_flag_ = false;
	}
      }
      else
        cnt = 10;
    }
    else 
      printf("Not find what vehicle go to lane change"); 
  }

  return ppAngle_;
}

float ScaleTruckController::purePuresuit(float tx_, float ty_)
{
  float Ld_, angle_A_, ampersand_, ppAngle;

  Ld_ = sqrt(pow(tx_, 2) + pow(ty_, 2)) + Ld_offset_;
  angle_A_ = atanf(ty_/(tx_));
  ampersand_ = atanf(2*sin(angle_A_)/Ld_) * (180.0f/M_PI);
  ppAngle = ampersand_;

  return ppAngle;
}

void ScaleTruckController::objectdetectInThread() 
{
  float dist, dist_tmp;
  dist_tmp = 10.1f;
  ros2_msg::msg::CmdData Lane_;
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
  if(!lc_flag_){
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
  else {
    ResultVel_ = TargetVel_;
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
      std::scoped_lock lock(rep_mutex_);
      if(lc_flag_) { // lane change
        AngleDegree_ = laneChange();
//        AngleDegree_ = AngleDegree2;
      }
      else{ 
        AngleDegree_ = AngleDegree;
      }
    }
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
  printf("ppAngle         : %2.3f degree\n", ppAngle_);
  printf("center_select   : %d\n", center_select_);
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
    sprintf(buf, "%.10e,%.3f,%.3f", diff_time, TargetVel_, AngleDegree_);
    write_file.open(file, std::ios::out | std::ios::app);
    write_file << buf << std::endl;
  }
  write_file.close();
}

void ScaleTruckController::TruckSubCallback(const ros2_msg::msg::CmdData::SharedPtr msg)
{
  {
    std::scoped_lock lock(rep_mutex_);
    if(index_ == 0) {
      lv_lc_right_ = msg->lv_lc_right;
      if(lv_lc_right_){
        prev_lane_coef_ = lane_coef_; // for compare prev_center vs. cur_center after lane change
        lc_flag_ = true;
      }
    }    
    else if(index_ == 1) {
      fv1_lc_right_ = msg->fv1_lc_right;
      if(fv1_lc_right_){
        prev_lane_coef_ = lane_coef_; // for compare prev_center vs. cur_center after lane change
        lc_flag_ = true;
      }
    }  
  }
}

void ScaleTruckController::LaneSubCallback(const ros2_msg::msg::CmdData::SharedPtr msg)
{
  {
    std::scoped_lock lock(lane_mutex_);
    lane_coef_.coef = msg->coef;
    AngleDegree = msg->cur_angle;
    AngleDegree2 = msg->cur_angle2;
    center_select_ = msg->center_select;
    target_x_ = msg->target_x;
    target_y_ = msg->target_y;
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
    std::scoped_lock lock(vel_mutex_);
    CurVel_ = msg->cur_vel;
  }
}

void ScaleTruckController::CmdSubCallback(const ros2_msg::msg::CmdData::SharedPtr msg)
{
  {
    std::scoped_lock lock(rep_mutex_);
    if(msg->tar_index == 0){  // LV 
      TargetVel_ = msg->tar_vel;
      TargetDist_ = msg->tar_dist;
    }
    else if(msg->tar_index == 2){ // FV2
      fv2_lc_right_ = msg->fv2_lc_right;
      if(fv2_lc_right_){
        prev_lane_coef_ = lane_coef_; // for compare prev_center vs. cur_center after lane change
        lc_flag_ = true;
      }
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




