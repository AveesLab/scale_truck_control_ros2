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

  RCLCPP_INFO(this->get_logger(), "waitKeyDelay1 : %d\n", waitKeyDelay_);
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

  RCLCPP_INFO(this->get_logger(), "[ScaleTruckController] Stop.");
}

bool ScaleTruckController::readParameters() {
  /***************/
  /* View Option */
  /***************/
  this->get_parameter_or("image_view/enable_opencv", viewImage_, true);
  this->get_parameter_or("image_view/wait_key_delay", waitKeyDelay_, 3);
  this->get_parameter_or("image_view/enable_console_output", enableConsoleOutput_, true);

  /*******************/
  /* Velocity Option */
  /*******************/
  this->get_parameter_or("params/index", index_, 0);
  this->get_parameter_or("params/target_vel", TargetVel_, 0.5f); // m/s
  this->get_parameter_or("params/safety_vel", SafetyVel_, 0.3f); // m/s
  this->get_parameter_or("params/fv_max_vel", FVmaxVel_, 0.8f); // m/s
  this->get_parameter_or("params/ref_vel", RefVel_, 0.0f); // m/s

  /*******************/
  /* Distance Option */
  /*******************/
  this->get_parameter_or("params/lv_stop_dist", LVstopDist_, 0.5f); // m
  this->get_parameter_or("params/fv_stop_dist", FVstopDist_, 0.5f); // m
  this->get_parameter_or("params/safety_dist", SafetyDist_, 1.5f); // m
  this->get_parameter_or("params/target_dist", TargetDist_, 0.8f); // m

  /***********************/
  /* Pure Pursuit Option */
  /***********************/
  this->get_parameter_or("params/Lw", Lw_, 0.34f);
  this->get_parameter_or("params/LdOffset", Ld_offset_, 0.0f);
  this->get_parameter_or("params/LdOffset2", Ld_offset2_, 0.0f);

  RCLCPP_INFO(this->get_logger(), "waitKeyDelay : %d\n", waitKeyDelay_);

  return true;
}

void ScaleTruckController::init() 
{
  RCLCPP_INFO(this->get_logger(), "[ScaleTruckController] init()");

  std::string imageTopicName;
  int imageQueueSize;
  std::string objectTopicName;
  int objectQueueSize;
  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string LrcPubTopicName;
  int LrcPubQueueSize;
  std::string CmdPubTopicName;
  int CmdPubQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("subscribers/camera_reading/topic", imageTopicName, std::string("/usb_cam/image_raw"));
  this->get_parameter_or("subscribers/camera_reading/queue_size", imageQueueSize, 1);
  this->get_parameter_or("subscribers/obstacle_reading/topic", objectTopicName, std::string("/raw_obstacles"));
  this->get_parameter_or("subscribers/obstacle_reading/queue_size", objectQueueSize, 100);
  this->get_parameter_or("subscribers/lrc_to_xavier/topic", XavSubTopicName, std::string("/lrc2xav_msg"));
  this->get_parameter_or("subscribers/lrc_to_xavier/queue_size", XavSubQueueSize, 1);

  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  this->get_parameter_or("publishers/xavier_to_lrc/topic", LrcPubTopicName, std::string("/xav2lrc_msg"));
  this->get_parameter_or("publishers/xavier_to_lrc/queue_size", LrcPubQueueSize, 1);
  this->get_parameter_or("publishers/xavier_to_cmd/topic", CmdPubTopicName, std::string("/xav2cmd_msg"));
  this->get_parameter_or("publishers/xavier_to_cmd/queue_size", CmdPubQueueSize, 1);

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  XavSubscriber_ = this->create_subscription<scale_truck_control_ros2::msg::Lrc2xav>(XavSubTopicName, XavSubQueueSize, std::bind(&ScaleTruckController::XavSubCallback, this, std::placeholders::_1));
  //objectSubscriber_ = this->create_subscription<scale_truck_control_ros2::msg::Ocr2lrc>(objectTopicName, objectQueueSize, std::bind(&ScaleTruckController::objectCallback, this, std::placeholders::_1));
//  laneSubscriber_ = nodeHandle_.subscribe(laneTopicName, laneQueueSize, &ScaleTruckController::LaneCallback, this);

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  LrcPublisher_ = this->create_publisher<scale_truck_control_ros2::msg::Xav2lrc>(LrcPubTopicName, LrcPubQueueSize);  
  CmdPublisher_ = this->create_publisher<scale_truck_control_ros2::msg::Xav2lrc>(CmdPubTopicName, CmdPubQueueSize);  

  /**********************/
  /* Safety Start Setup */
  /**********************/
  distance_ = 10.f;
  distAngle_ = 0;
  
  /**********************************/
  /* Control & Communication Thread */
  /**********************************/
  controlThread_ = std::thread(&ScaleTruckController::spin, this);
  
  /* PC와 통신 */
//  tcpThread_ = std::thread(&ScaleTruckController::reply, this); 
}

bool ScaleTruckController::getImageStatus(void)
{
  std::scoped_lock lock(image_mutex_);
  return imageStatus_;
}

/***************/
/* cmd publish */
/***************/
//void ScaleTruckController::reply(){
//  scale_truck_control_ros2::msg::Xav2cmd cmd;
//  
//  while(isNodeRunning_){
//    {
//      std::scoped_lock lock(vel_mutex_, dist_mutex_);
//      cmd.cur_vel = CurVel_;
//      cmd.cur_dist = actDist_;
//      cmd.cur_angle = AngleDegree_;
//    }
//    {
//      std::scoped_lock lock(lane_mutex_);
//      cmd->coef[0].a = laneDetector_.lane_coef_.left.a;
//      cmd->coef[0].b = laneDetector_.lane_coef_.left.b;
//      cmd->coef[0].c = laneDetector_.lane_coef_.left.c;
//      cmd->coef[1].a = laneDetector_.lane_coef_.right.a;
//      cmd->coef[1].b = laneDetector_.lane_coef_.right.b;
//      cmd->coef[1].c = laneDetector_.lane_coef_.right.c;
//      cmd->coef[2].a = laneDetector_.lane_coef_.center.a;
//      cmd->coef[2].b = laneDetector_.lane_coef_.center.b;
//      cmd->coef[2].c = laneDetector_.lane_coef_.center.c;
//    }
//   CmdPublisher_->publish(cmd);
//
//   std::this_thread::sleep_for(std::chrono::milliseconds(2));
//  }
//}


/* LaneDetection Node에 CurVel_ 전달해야함. */
/* 카메라 오류 검출은 Lane Node에서 진행 */
/* Angle값 받음. LRC에서 받아도 될듯? 일단 여기에서 진행 */

void* ScaleTruckController::lanedetectInThread() {
//  static int cnt = 10;
//  Mat dst;
//  std::vector<Mat>channels;
//  int count = 0;
//  if((!camImageTmp_.empty()) && (cnt != 0) && (TargetVel_ != 0))
//  {
//    bitwise_xor(camImageCopy_,camImageTmp_, dst);
//    split(dst, channels);
//    for(int ch = 0; ch<dst.channels();ch++) {
//      count += countNonZero(channels[ch]);
//    }
//    if(count == 0 && Beta_)
//      cnt -= 1;
//    else
//      cnt = 10;
//  }
//  float AngleDegree;
//  camImageTmp_ = camImageCopy_.clone();
//  //laneDetector_.get_steer_coef(CurVel_);
//  //AngleDegree = laneDetector_.display_img(camImageTmp_, waitKeyDelay_, viewImage_);
//  if(cnt == 0){
//    AngleDegree_ = -distAngle_;
//  }
//  else
//    AngleDegree_ = AngleDegree;
}




void* ScaleTruckController::objectdetectInThread() {
  float dist, angle;
  float dist_tmp, angle_tmp;

  dist_tmp = 10.f;
  /**************/
  /* Lidar Data */
  /**************/
//  {
//    std::scoped_lock lock(lane_mutex_, object_mutex_);
//    ObjSegments_ = Obstacle_.segments.size();
//    ObjCircles_ = Obstacle_.circles.size();
//
//  }
//
//  for(int i = 0; i < ObjCircles_; i++)
//  {
//    //dist = sqrt(pow(Obstacle_.circles[i].center.x,2)+pow(Obstacle_.circles[i].center.y,2));
//    dist = -Obstacle_.circles[i].center.x - Obstacle_.circles[i].true_radius;
//    angle = atanf(Obstacle_.circles[i].center.y/Obstacle_.circles[i].center.x)*(180.0f/M_PI);
//    if(dist_tmp >= dist) {
//      dist_tmp = dist;
//      angle_tmp = angle;
//    }
//  }
//  actDist_ = dist_tmp;
//
//  if(ObjCircles_ != 0)
//  {
//    distance_ = dist_tmp;
//    distAngle_ = angle_tmp;
//  }
  /*****************************/
  /* Dynamic ROI Distance Data */
  /*****************************/
  {
    std::scoped_lock lock(rep_mutex_, lane_mutex_);
    /* LaneDetection Node에 ROI거리, 트리거=true 보낸다.  */
    if(dist_tmp < 1.24f && dist_tmp > 0.30f) // 1.26 ~ 0.28
    {
//      laneDetector_.distance_ = (int)((1.24f - dist_tmp)*490.0f)+20;
    }
    else {
//      laneDetector_.distance_ = 0;
    }
    droi_ready_ = true;
    cv_.notify_one(); // 현규형은 안썼음 일단 대기. 
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

  const auto wait_duration = std::chrono::milliseconds(2000);
  while(!getImageStatus()) {
    printf("Waiting for image.\n");
    if(!isNodeRunning_) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }

  scale_truck_control_ros2::msg::Xav2lrc msg;
  std::thread lanedetect_thread;
  std::thread objectdetect_thread;

  const auto wait_image = std::chrono::milliseconds(20);

  while(!controlDone_ && rclcpp::ok()) {
    struct timeval start_time, end_time;
    gettimeofday(&start_time, NULL);

    lanedetect_thread = std::thread(&ScaleTruckController::lanedetectInThread, this);
    objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);

    lanedetect_thread.join();
    objectdetect_thread.join();

    msg.tar_vel = ResultVel_;  //Xavier to LRC and LRC to OpenCR
    {
      std::scoped_lock lock(dist_mutex_);
      msg.steer_angle = AngleDegree_; // get from objectThread
      msg.cur_dist = distance_;       // ''
    }
    {
      std::scoped_lock lock(rep_mutex_);
      msg.tar_dist = TargetDist_; 
    }    

    LrcPublisher_->publish(msg);   

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
//  static std::string ipAddr = ZMQ_SOCKET_.getIPAddress();

  printf("\033[2J");
  printf("\033[1;1H");
  printf("\nAngle           : %2.3f degree", AngleDegree_);
  printf("\nRefer Vel       : %3.3f m/s", RefVel_);
  printf("\nSend Vel        : %3.3f m/s", ResultVel_);
  printf("\nTar/Cur Vel     : %3.3f / %3.3f m/s", TargetVel_, CurVel_);
  printf("\nTar/Cur Dist    : %3.3f / %3.3f m", TargetDist_, distance_);
//  printf("\nK1/K2           : %3.3f / %3.3f", laneDetector_.K1_, laneDetector_.K2_);
//  if(ObjCircles_ > 0) {
//    printf("\nCirs            : %d", ObjCircles_);
//    printf("\nDistAng         : %2.3f degree", distAngle_);
//  }
//  if(ObjSegments_ > 0) {
//    printf("\nSegs            : %d", ObjSegments_);
//  }
  printf("\nCycle Time      : %3.3f ms", CycleTime_);
  printf("\n");
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
  std::string log_path = "/home/jetson/catkin_ws/logfiles/";
  if(!flag){
    for(int i = 0; i < 100; i++){
      file_name[7] = i/10 + '0';  //ASCII
      file_name[8] = i%10 + '0';
      sprintf(file, "%s%s", log_path.c_str(), file_name);
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

//void ScaleTruckController::laneSubCallback(const scale_truck_control_ros2::msg::Lane2xav::SharedPtr msg)
//{
//  /* Callback from LaneDetetion Node   */
//  {
//    std::scoped_lock lock(lane_mutex_);
//    lane_coef_ = msg->lane_coef;
//    AngleDegree_ = msg->AngleDegree;
//  }
//}

//void ScaleTruckController::objectCallback(const obstacle_detector::Obstacles &msg) {
//  /* Callback from ObjectDetetion Node   */
//  {
//    std::scoped_lock lock(object_mutex_);
//    Obstacle_ = msg;
//  }
//}

void ScaleTruckController::XavSubCallback(const scale_truck_control_ros2::msg::Lrc2xav::SharedPtr msg)
{
  {
    std::scoped_lock lock(vel_mutex_);
    CurVel_ = msg->cur_vel;

    RCLCPP_INFO(this->get_logger(), "CurVel_ : %.3f\n", CurVel_);
  }
}

//void ScaleTruckController::CmdSubCallback(const scale_truck_control_ros2::msg::Cmd2xav::SharedPtr msg)
//{
//  /* Callback from Command PC   */
//  {
//    std::scoped_lock lock(rep_mutex_);
//    if(index_ == 0){  //LV 
//      TargetVel_ = msg->tar_vel;
//      TargetDist_ = msg->tar_dist;
//    }
//  }
//}


} /* namespace scale_truck_control */


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<scale_truck_control::ScaleTruckController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




