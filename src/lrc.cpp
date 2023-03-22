#include "lrc/lrc.hpp"

using namespace std::chrono_literals;

namespace LocalResiliencyCoordinator{

LocalRC::LocalRC(void) 
	: Node("LRC", rclcpp::NodeOptions()
                      .allow_undeclared_parameters(true)
		      .automatically_declare_parameters_from_overrides(true))
{
  init();
  RCLCPP_INFO(this->get_logger(), "epsilon_: '%.3f'", epsilon_);
}

LocalRC::~LocalRC(){
  isNodeRunning_ = false; 
}

void LocalRC::init(){

  isNodeRunning_ = true;

  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string OcrSubTopicName;
  int OcrSubQueueSize;
  std::string LVSubTopicName;
  int LVSubQueueSize;
  std::string XavPubTopicName;
  int XavPubQueueSize;
  std::string OcrPubTopicName;
  int OcrPubQueueSize;
  std::string FVsPubTopicName;
  int FVsPubQueueSize;

  this->get_parameter_or("params/truck_info", index_, 0);

  this->get_parameter_or("LrcParams/lrc_log_path", log_path_, std::string("/home/jetson/catkin_ws/logfiles/"));
  this->get_parameter_or("LrcParams/epsilon", epsilon_, 1.0f);
  this->get_parameter_or("LrcParams/lu_ob_A", a_, 0.6817f);
  this->get_parameter_or("LrcParams/lu_ob_B", b_, 0.3183f);
  this->get_parameter_or("LrcParams/lu_ob_L", l_, 0.2817f);
  this->get_parameter_or("LrcParams/enable_console_output", EnableConsoleOutput_, true);
  /******************************/
  /* ROS Topic Subscribe Option */
  /******************************/
  /* 어디서 날린걸 sub한다.  */
  this->get_parameter_or("LrcSubPub/xavier_to_lrc/topic", XavSubTopicName, std::string("/xav2lrc_msg"));
  this->get_parameter_or("LrcSubPub/xavier_to_lrc/queue_size", XavSubQueueSize, 1);
  this->get_parameter_or("LrcSubPub/ocr_to_lrc/topic", OcrSubTopicName, std::string("/ocr2lrc_msg"));
  this->get_parameter_or("LrcSubPub/ocr_to_lrc/queue_size", OcrSubQueueSize, 1);
  this->get_parameter_or("LrcSubPub/LV_to_FVs/topic", LVSubTopicName, std::string("/LV_lrc2FVs_msg"));
  this->get_parameter_or("LrcSubPub/LV_to_FVs/queue_size", LVSubQueueSize, 1);

  /******************************/
  /* ROS Topic Publish Option */
  /******************************/
  /* 어디로 pub한다. */
  this->get_parameter_or("LrcSubPub/lrc_to_xavier/topic", XavPubTopicName, std::string("/lrc2xav_msg"));
  this->get_parameter_or("LrcSubPub/lrc_to_xavier/queue_size", XavPubQueueSize, 1);
  this->get_parameter_or("LrcSubPub/lrc_to_ocr/topic", OcrPubTopicName, std::string("/lrc2ocr_msg"));
  this->get_parameter_or("LrcSubPub/lrc_to_ocr/queue_size", OcrPubQueueSize, 1);
  this->get_parameter_or("LrcSubPub/lrc_to_FVs/topic", FVsPubTopicName, std::string("/LV_lrc2FVs_msg"));
  this->get_parameter_or("LrcSubPub/lrc_to_FVs/queue_size", FVsPubQueueSize, 1);

  /************************/
  /* ROS Topic Subscriber */
  /************************/
  OcrSubscriber_ = this->create_subscription<scale_truck_control_ros2::msg::Ocr2lrc>(OcrSubTopicName, OcrSubQueueSize, std::bind(&LocalRC::OcrCallback, this, std::placeholders::_1));

  XavSubscriber_ = this->create_subscription<scale_truck_control_ros2::msg::Lrc2xav>(XavSubTopicName, XavSubQueueSize, std::bind(&LocalRC::XavCallback, this, std::placeholders::_1));

//  if (index_ == 11 || index_ == 12){
//    LVSubscriber_ = this->create_subscription<scale_truck_control_ros2::msg::LV2FVs>(LVSubTopicName, LVSubQueueSize, std::bind(&LocalRC::LVCallback, this, std::placeholders::_1));
//  }


  /************************/
  /* ROS Topic Publisher */
  /************************/
  XavPublisher_ = this->create_publisher<scale_truck_control_ros2::msg::Lrc2xav>(XavPubTopicName, XavPubQueueSize);  
  OcrPublisher_ = this->create_publisher<scale_truck_control_ros2::msg::Lrc2ocr>(OcrPubTopicName, OcrPubQueueSize);
//  FVsPublisher_ = this->create_publisher<scale_truck_control_ros2::msg::LV2FVs>(FVsPubTopicName, FVsPubQueueSize);

  /*********************/
  /* spin & udp thread */
  /*********************/
  lrcThread_ = std::thread(&LocalRC::communicate, this);
  if (index_ == 10){
//    udpThread_ = std::thread(&LocalRC::radio, this);
  }
}

bool LocalRC::isNodeRunning(){
  return isNodeRunning_;
}

/****************/
/* FVs Publish  */
/****************/
//void LocalRC::radio()
//{
//  scale_truck_control_ros2::msg::LV2FVs FVs;
//  while(isNodeRunning()){
//    {
//      std::scoped_lock lock(data_mutex_);
//      FVs.tar_vel = tar_vel_;
//      FVs.->tar_dist = tar_dist_;
//    }
//    FVsPublisher_->publish(FVs); 
//    
//    std::this_thread::sleep_for(std::chrono::milliseconds(2));
//  }
//}

void LocalRC::rosPub(){
  scale_truck_control_ros2::msg::Lrc2xav xav;
  scale_truck_control_ros2::msg::Lrc2ocr ocr;
  { 
    std::scoped_lock lock(data_mutex_);
    xav.cur_vel = cur_vel_;
    xav.tar_vel = tar_vel_; // xav에 목표 속도,간격 왜 날리지?
    xav.tar_dist = tar_dist_; // ''
    ocr.index = index_;
    ocr.steer_angle = angle_degree_;
    ocr.cur_dist = cur_dist_;
    ocr.tar_dist = tar_dist_;
    ocr.tar_vel = tar_vel_;
    ocr.est_vel = est_vel_;
  }
  XavPublisher_->publish(xav);
  OcrPublisher_->publish(ocr);
}

void LocalRC::recordData(struct timeval *startTime){
  struct timeval currentTime;
  char file_name[] = "LocalRC_log00.csv";
  static char file[128] = {0x00, };
  char buf[256] = {0x00,};
  static bool flag = false;
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
    write_file << "Time,Tar_dist,Cur_dist,Tar_vel,Ref_vel,Cur_vel,Lrc_mode" << std::endl; //seconds
    flag = true;
  }
  else{
    std::scoped_lock lock(data_mutex_);
    gettimeofday(&currentTime, NULL);
    time_ = ((currentTime.tv_sec - startTime->tv_sec)) + ((currentTime.tv_usec - startTime->tv_usec)/1000000.0);
    //sprintf(buf, "%.10e,%.3f,%.3f,%.3f,%.3f,%.3f,%d", time_, tar_dist_, cur_dist_, tar_vel_, ref_vel_, cur_vel_);
    write_file.open(file, std::ios::out | std::ios::app);
    write_file << buf << std::endl;
  }
  write_file.close();
}

void LocalRC::printStatus(){
  if (EnableConsoleOutput_){
    printf("\033[2J");
    printf("\033[1;1H");
    printf("\nPredict Velocity:\t%.3f", est_vel_);
    printf("\nTarget Velocity:\t%.3f", tar_vel_);
    printf("\nCurrent Velocity:\t%.3f", cur_vel_);
    printf("\nTarget Distance:\t%.3f", tar_dist_);
    printf("\nCurrent Distance:\t%.3f", cur_dist_);
    printf("\nEstimated Value:\t%.3f", fabs(cur_vel_ - hat_vel_));
    printf("\n");
  }
}


void LocalRC::communicate(){
  struct timeval startTime, endTime;
  gettimeofday(&startTime, NULL);
  while(rclcpp::ok()){
    //encoderCheck();
    //updateMode(crc_mode_);
    rosPub();
    //printStatus();

    //recordData(&startTime);

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    if(!isNodeRunning()){
      rclcpp::shutdown();
      break;
    }
  }
}









// test lrc->ocr
void LocalRC::Lrc2ocrCallback(void) 
{
  auto msg = scale_truck_control_ros2::msg::Lrc2ocr();
  msg.tar_vel = 1.0;                                                     
  msg.tar_dist = 0.8;                                                    
  msg.cur_dist = 0.62;                                                   
  msg.steer_angle = 5.0;                                                   
  msg.est_vel = 1.0;                                                   
  //RCLCPP_INFO(this->get_logger(), "Publishing: '%.3f'", msg.tar_vel);
  OcrPublisher_->publish(msg);
}

void LocalRC::XavCallback(const scale_truck_control_ros2::msg::Xav2lrc::SharedPtr msg)
{
  std::scoped_lock lock(data_mutex_);
  angle_degree_ = msg->steer_angle;
  cur_dist_ = msg->cur_dist;
  if(index_ == 10){  //only LV LRC
    tar_dist_ = msg->tar_dist;
    tar_vel_ = msg->tar_vel;
  }
}


void LocalRC::OcrCallback(const scale_truck_control_ros2::msg::Ocr2lrc::SharedPtr msg)   
{
  std::scoped_lock lock(data_mutex_);
  ref_vel_ = msg->ref_vel;
  cur_vel_ = msg->cur_vel;
  sat_vel_ = msg->u_k;  //saturated velocity

  RCLCPP_INFO(this->get_logger(), "I heard: '%.3f'", msg->ref_vel);     
  RCLCPP_INFO(this->get_logger(), "I heard: '%.3f'", msg->cur_vel);     
}

/***************/
/* FVs from LV */
/***************/
//void LocalRC::LVCallback(const scale_truck_control_ros2::msg::LV2FVs::SharedPtr msg)
//{
//  std::scoped_lock lock(data_mutex_);
//  tar_vel_ = msg->tar_vel;
//  tar_dist_ = msg->tar_dist;
//}







} /* namespace scale_truck_control */

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalResiliencyCoordinator::LocalRC>());
    rclcpp::shutdown();
    return 0;
}

